import re, math, pickle
import pipeline_v11 as P
from pipeline_v11 import *
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from beambar_engine import text_bbox, seg_intersects_bbox
from hatch_engine import get_hatch_polys, bbox_poly_overlap

insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
src='karaisk05_OR0.dxf'
ins, blocks = load_all(src)
hatch_polys = get_hatch_polys(src)
P.REBAR_NAMES = set(n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n))

name='FL0_BEAM_TEXT12'; own_beam='FL0_BEAM12'
bl = block_text_bboxes(blocks[name])
beam_lines,_ = block_lines_local(blocks[own_beam])
ux,uy,lo,hi = beam_span(beam_lines)
px,py=-uy,ux
perp = [x*px+y*py for x1,y1,x2,y2 in beam_lines for x,y in [(x1,y1),(x2,y2)]]
print('beam12 axis=(%.2f,%.2f) span %.2f..%.2f (μήκος %.2f), πλάτος %.2f'%(ux,uy,lo,hi,hi-lo,max(perp)-min(perp)))
hb = union_bbox(bl)
print('text union bbox', ['%.2f'%v for v in hb], ' -> κατά μήκος θέση %.2f'%(((hb[0]+hb[2])/2)*ux+((hb[1]+hb[3])/2)*uy))
# hatch στο δικό του block?
own_h=[pn for _,pn in hatch_polys if pn==own_beam]
print('hatch στο', own_beam, ':', len(own_h))

# στατιστικά απόρριψης σε όλο το εύρος slide
obst=[]
for n2 in blocks:
    if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE|BEAMBAR|SLABBAR)\d*$',n2) and 'TEXT' not in n2:
        d2 = insert_final.get(n2, ins.get(n2,(0,0)))
        if n2 not in insert_final: d2 = ins.get(n2,(0,0))
        if re.match(r'FL-?\d+_BEAMBAR',n2) and d2[0]<=-49: continue
        for x1,y1,x2,y2 in block_lines_local(blocks[n2])[0]:
            obst.append((x1+d2[0],y1+d2[1],x2+d2[0],y2+d2[1],n2))
placed=[]
for n2 in blocks:
    if n2==name: continue
    if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$',n2):
        b2=block_text_bboxes(blocks[n2])
    elif re.match(r'FL-?\d+_SLAB\d+$',n2):
        from compute_beamtext_slabmarker import slab_marker_boxes
        b2=slab_marker_boxes(blocks[n2])
    else: continue
    d2=insert_final.get(n2,(0,0)); t2=text_local_final.get(n2,(0,0))
    if re.match(r'FL-?\d+_BEAMBAR',n2) and d2[0]<=-49: continue
    for x,y,w,h,rot in (b2 or []):
        placed.append(text_bbox(x+d2[0]+t2[0],y+d2[1]+t2[1],w,h,rot))

home_t = ((hb[0]+hb[2])/2)*ux + ((hb[1]+hb[3])/2)*uy
stats={'span':0,'hatch':0,'text':0,'cross':0,'ok':0}
best=None
for s in [0.03*k for k in range(0,67)]:
    for sign in ((1,) if s==0 else (1,-1)):
        t = home_t + s*sign
        if t<lo or t>hi: stats['span']+=1; continue
        ddx,ddy = ux*s*sign, uy*s*sign
        # ανά αιτία
        bad=None
        for x,y,w,h,rot in bl:
            bx=text_bbox(x+ddx,y+ddy,w,h,rot)
            for poly,pn in hatch_polys:
                hp = 0.0 if pn==own_beam else 0.25
                if bbox_poly_overlap((bx[0]-hp,bx[1]-hp,bx[2]+hp,bx[3]+hp),poly): bad='hatch';break
            if bad: break
            for ob in placed:
                if not(bx[2]<ob[0] or ob[2]<bx[0] or bx[3]<ob[1] or ob[3]<bx[1]): bad='text';break
            if bad: break
        if bad: stats[bad]+=1; continue
        crossed = count_line_crossings(bl, ddx, ddy, obst, (own_beam,))
        if len(crossed)>1: stats['cross']+=1
        else:
            stats['ok']+=1
            if best is None: best=(round(ddx,2),round(ddy,2),sorted(crossed))
print(stats, '\nπρώτη αποδεκτή:', best)
