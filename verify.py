import re, math, pickle
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from beambar_engine import text_bbox, seg_intersects_bbox
insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
src='karaisk05_OR0.dxf'
ins, blocks = load_all(src)

all_lines=[]
for n in blocks:
    if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE|BEAMBAR|SLABBAR)\d*$',n) and 'TEXT' not in n:
        d2 = insert_final.get(n) if n in insert_final else ins.get(n,(0,0))
        if re.match(r'FL-?\d+_BEAMBAR',n) and d2[0]<=-49: continue
        for x1,y1,x2,y2 in block_lines_local(blocks[n])[0]:
            all_lines.append((x1+d2[0],y1+d2[1],x2+d2[0],y2+d2[1],n))

def own_of(n):
    m=re.match(r'(FL-?\d+_)(BEAM|COLUMN)_TEXT(\d+)$',n)
    if m: return m.group(1)+m.group(2)+m.group(3)
    return n

def report(n):
    bl = block_text_bboxes(blocks[n])
    dx,dy = insert_final.get(n,(0,0)); tx,ty = text_local_final.get(n,(0,0))
    ub = union_bbox([(x+dx+tx,y+dy+ty,w,h,rot) for x,y,w,h,rot in bl])
    crossers=set()
    for x,y,w,h,rot in bl:
        bb = text_bbox(x+dx+tx,y+dy+ty,w,h,rot)
        for s in all_lines:
            if s[4] in (n, own_of(n)): continue
            if seg_intersects_bbox(s[:4], bb): crossers.add(s[4])
    print(f'{n:18s} κέντρο ({(ub[0]+ub[2])/2:6.2f},{(ub[1]+ub[3])/2:6.2f})  offset=({dx+tx:+.2f},{dy+ty:+.2f})  γραμμές στο ΠΛΗΡΕΣ κουτί: {sorted(crossers) if crossers else "ΚΑΜΙΑ"}')

for n in ['FL0_COLUMN_TEXT5','FL0_COLUMN_TEXT11','FL0_SLABBAR7','FL0_SLABBAR10','FL0_SLABBAR11','FL0_BEAMBAR4']:
    report(n)
