import re, math, pickle
import pipeline_v11 as P
from pipeline_v11 import *
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from beambar_engine import text_bbox, seg_intersects_bbox
from hatch_engine import get_hatch_polys, bbox_poly_overlap
insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
ins, blocks = load_all('karaisk05_OR0.dxf')
hatch_polys = get_hatch_polys('karaisk05_OR0.dxf')
P.REBAR_NAMES = set(n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n))

name='FL0_SLABBAR18'
dx,dy = insert_final[name]
tx,ty = text_local_final.get(name,(0,0))
lines,_ = block_lines_local(blocks[name])
bl = block_text_bboxes(blocks[name])
print('τρέχον offset', (dx,dy), 'text_local', (tx,ty))
# obstacles όπως στο repair
obst=[]
for n2 in blocks:
    if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE)\d*$',n2) and 'TEXT' not in n2:
        o=ins.get(n2,(0,0))
        for x1,y1,x2,y2 in block_lines_local(blocks[n2])[0]:
            obst.append((x1+o[0],y1+o[1],x2+o[0],y2+o[1],n2))
for rname in P.REBAR_NAMES:
    if rname==name: continue
    rd = insert_final.get(rname,(0,0))
    if re.match(r'FL-?\d+_BEAMBAR',rname) and rd[0]<=-49: continue
    for x1,y1,x2,y2 in block_lines_local(blocks[rname])[0]:
        obst.append((x1+rd[0],y1+rd[1],x2+rd[0],y2+rd[1],rname))
# placed boxes
from compute_beamtext_slabmarker import slab_marker_boxes
placed=[]
for n2 in blocks:
    if n2==name: continue
    if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$',n2): b2=block_text_bboxes(blocks[n2])
    elif re.match(r'FL-?\d+_SLAB\d+$',n2): b2=slab_marker_boxes(blocks[n2])
    else: continue
    d2=insert_final.get(n2,(0,0)); t2=text_local_final.get(n2,(0,0))
    if re.match(r'FL-?\d+_BEAMBAR',n2) and d2[0]<=-49: continue
    for x,y,w,h,rot in (b2 or []): placed.append(text_bbox(x+d2[0]+t2[0],y+d2[1]+t2[1],w,h,rot))

for s,sign in ((0.20,-1),(0.25,-1),(0.30,-1),(0.15,-1),(0.35,-1),(0.20,1)):
    ndx,ndy = dx+0*s*sign, dy+1*s*sign
    reasons=[]
    if not P._slabbar_bounds_ok(name, lines, ndx, ndy): reasons.append('slab-bounds')
    okf = is_ok_full(bl, ndx, ndy, obst, hatch_polys, placed, (name,))
    okr = is_ok_relaxed(bl, ndx, ndy, obst, hatch_polys, placed, (name,), max_crossings=1)
    if not okf:
        # ανάλυση αιτίας
        for x,y,w,h,rot in bl:
            bb=text_bbox(x+ndx,y+ndy,w,h,rot)
            for poly,pn in hatch_polys:
                hp=0.25
                if bbox_poly_overlap((bb[0]-hp,bb[1]-hp,bb[2]+hp,bb[3]+hp),poly): reasons.append('hatch:'+pn)
            for i,ob in enumerate(placed):
                if not(bb[2]<ob[0] or ob[2]<bb[0] or bb[3]<ob[1] or ob[3]<bb[1]): reasons.append('textbox')
        crossed = count_line_crossings(bl, ndx, ndy, obst, (name,))
        if crossed: reasons.append('lines:'+','.join(sorted(crossed)))
    print(f's={s:.2f} sign={sign:+d}: full={okf} relaxed={okr}  αιτίες: {sorted(set(reasons)) or "-"}')
