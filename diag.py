import re, math, pickle
import pipeline_v11 as P
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from beambar_engine import text_bbox, seg_intersects_bbox
from compute_beamtext_slabmarker import slab_marker_boxes

src='karaisk05_OR0.dxf'
insert_final, text_local_final = P.process_all(src)
pickle.dump((insert_final, text_local_final), open('offs.pkl','wb'))
ins, blocks = load_all(src)

def boxes_of(n):
    if re.match(r'FL-?\d+_SLAB\d+$', n):
        bl = slab_marker_boxes(blocks[n])
    else:
        bl = block_text_bboxes(blocks[n])
    dx,dy = insert_final.get(n,(0,0))
    if re.match(r'FL-?\d+_BEAMBAR',n) and dx<=-49: return []
    tx,ty = text_local_final.get(n,(0,0))
    return [text_bbox(x+dx+tx,y+dy+ty,w,h,rot) for x,y,w,h,rot in (bl or [])]

text_names = [n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$',n) or re.match(r'FL-?\d+_SLAB\d+$',n)]
# όλες οι γραμμές (δομικά + σίδερα στις τελικές θέσεις)
all_lines=[]
for n in blocks:
    if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE|BEAMBAR|SLABBAR)\d*$',n) and 'TEXT' not in n:
        dx,dy = insert_final.get(n, ins.get(n,(0,0)))
        if n not in insert_final: dx,dy = ins.get(n,(0,0))
        if re.match(r'FL-?\d+_BEAMBAR',n) and dx<=-49: continue
        for x1,y1,x2,y2 in block_lines_local(blocks[n])[0]:
            all_lines.append((x1+dx,y1+dy,x2+dx,y2+dy,n))

def own_of(n):
    m=re.match(r'(FL-?\d+_)(BEAM|COLUMN)_TEXT(\d+)$',n)
    if m: return m.group(1)+m.group(2)+m.group(3)
    m=re.match(r'(FL-?\d+_)(BEAMBAR|SLABBAR)(\d+)$',n)
    if m: return n
    return None

pairs_line=set(); pairs_text=set()
for n in text_names:
    own=own_of(n)
    for bb in boxes_of(n):
        x1,y1,x2,y2=bb
        # core (όπως στο count_line_crossings)
        h=y2-y1; m=min(h*0.5,(x2-x1)*0.3)
        core=(x1+m,y1+m,x2-m,y2-m) if (x2-x1>2*m and y2-y1>2*m) else bb
        for sx1,sy1,sx2,sy2,onm in all_lines:
            if onm==n or onm==own: continue
            if seg_intersects_bbox((sx1,sy1,sx2,sy2), core):
                pairs_line.add((n,onm))
        for n2 in text_names:
            if n2<=n: continue
            for ob in boxes_of(n2):
                if not(x2<ob[0] or ob[2]<x1 or y2<ob[1] or ob[3]<y1):
                    pairs_text.add((n,n2))
print('\n== ΓΡΑΜΜΗ μέσα σε κείμενο (core):', len(pairs_line))
for p in sorted(pairs_line): print('  ', p[0], '<-', p[1])
print('== ΚΕΙΜΕΝΟ πάνω σε κείμενο:', len(pairs_text))
for p in sorted(pairs_text): print('  ', p[0], 'x', p[1])
