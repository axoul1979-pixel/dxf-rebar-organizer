import re, math, pickle
import pipeline_v11 as P
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes
from beambar_engine import text_bbox

insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
src='karaisk05_OR0.dxf'
ins, blocks = load_all(src)

for name in ('FL0_SLABBAR10','FL0_SLABBAR11','FL0_BEAM_TEXT12'):
    dx,dy = insert_final.get(name,(0,0)); tx,ty = text_local_final.get(name,(0,0))
    bl = block_text_bboxes(blocks[name])
    for x,y,w,h,rot in bl:
        bb = text_bbox(x+dx+tx,y+dy+ty,w,h,rot)
        print(f'{name:18s} rot={rot:5.1f} bb=({bb[0]:.2f},{bb[1]:.2f})-({bb[2]:.2f},{bb[3]:.2f})  w={bb[2]-bb[0]:.2f} h={bb[3]-bb[1]:.2f}')
    lines,_ = block_lines_local(blocks[name])
    if lines:
        xs=[p for s in lines for p in (s[0],s[2])]; ys=[p for s in lines for p in (s[1],s[3])]
        print(f'{"":18s} bar extent x {min(xs)+dx:.2f}..{max(xs)+dx:.2f}  y {min(ys)+dy:.2f}..{max(ys)+dy:.2f}')
