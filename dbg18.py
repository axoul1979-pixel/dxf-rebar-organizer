import re, math, pickle
import pipeline_v11 as P
from pipeline_v11 import *
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from beambar_engine import text_bbox, seg_intersects_bbox
from hatch_engine import get_hatch_polys
insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
ins, blocks = load_all('karaisk05_OR0.dxf')
hatch_polys = get_hatch_polys('karaisk05_OR0.dxf')
P.REBAR_NAMES = set(n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n))
sp = P.get_slab_polys('karaisk05_OR0.dxf')

name='FL0_SLABBAR18'; victim='FL0_BEAM_TEXT16'
dx,dy = insert_final[name]
lines,_ = block_lines_local(blocks[name])
(aux,auy),(alo,ahi) = spine_and_bounds(lines)
nx,ny = -auy,aux
print(f'ράβδος: άξονας ({aux:.2f},{auy:.2f}) -> κάθετο nudge στη διεύθυνση ({nx:.2f},{ny:.2f})')
# επιτρεπόμενο κουτί
bxs=[p+dx for s in lines for p in (s[0],s[2])]; bys=[p+dy for s in lines for p in (s[1],s[3])]
print(f'σίδερο τώρα: x {min(bxs):.2f}..{max(bxs):.2f} y {min(bys):.2f}..{max(bys):.2f}  SLAB6 bbox y 4.78..6.05')
# κείμενο θύματος
vdx,vdy = insert_final.get(victim,(0,0))
vb=[text_bbox(x+vdx,y+vdy,w,h,rot) for x,y,w,h,rot in block_text_bboxes(blocks[victim])]
u=union_bbox([(x+vdx,y+vdy,w,h,rot) for x,y,w,h,rot in block_text_bboxes(blocks[victim])])
print(f'{victim} boxes: x {u[0]:.2f}..{u[2]:.2f}  y {u[1]:.2f}..{u[3]:.2f}')
# δοκιμή nudges
home = (min(bys)+max(bys))/2
ok_any=False
for s in [0.05*k for k in range(0,33)]:
    for sign in ((1,) if s==0 else (1,-1)):
        ndy = dy+ny*s*sign*(1 if ny else 0) + 0  # γενικό
        ndx2,ndy2 = dx+nx*s*sign, dy+ny*s*sign
        lines_new=[(x1+ndx2,y1+ndy2,x2+ndx2,y2+ndy2) for x1,y1,x2,y2 in lines]
        nys=[p for seg in lines_new for p in (seg[1],seg[3])]
        inslab = 4.78-0.03<=min(nys) and max(nys)<=6.05+0.03
        hits_victim = any(seg_intersects_bbox(seg[:4], b) for seg in lines_new for b in vb)
        if inslab and not hits_victim and not ok_any:
            print(f'πρώτο s που καθαρίζει το θύμα ΚΑΙ μένει στην πλάκα: s={s:.2f} sign={sign:+d} -> y={min(nys):.2f}..{max(nys):.2f}')
            ok_any=True
if not ok_any:
    print('ΔΕΝ υπάρχει κάθετο nudge εντός πλάκας που να καθαρίζει το θύμα!')
