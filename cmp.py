import re, math
import pipeline_v11 as P
from engine import load_all
from compute_column_text import block_text_bboxes, union_bbox, translate_bbox
from perimeter import build_footprint, column_centers, column_outward_dirs, bbox_outside
from pipeline_v11 import get_slab_polys

src='karaisk05_OR0.dxf'
ins,blocks = load_all(src)
fp = build_footprint(blocks, ins, get_slab_polys(src)); cc = column_centers(blocks, ins)
dirs = column_outward_dirs(blocks, ins, fp)
NAMES=['A','NE','N','NW','W','SW','S','SE']
GR={'A':'Α','NE':'ΒΑ','N':'Β','NW':'ΒΔ','W':'Δ','SW':'ΝΔ','S':'Ν','SE':'ΝΑ'}

def run(flag):
    P.PERIM_OUTWARD = flag
    a,_ = P.process_all(src)
    return a
new = run(True); old = run(False)

def descr(n, res):
    bb = union_bbox(block_text_bboxes(blocks[n])); dx,dy = res[n]
    fb = translate_bbox(bb,dx,dy); c=((fb[0]+fb[2])/2,(fb[1]+fb[3])/2)
    num = re.search(r'\d+$', n).group()
    cx,cy = cc['FL0_COLUMN'+num]
    ang = math.degrees(math.atan2(c[1]-cy, c[0]-cx))%360
    return GR[NAMES[int((ang+22.5)//45)%8]], math.hypot(c[0]-cx,c[1]-cy), bbox_outside(fp,fb)

print()
print('          ΠΑΛΙΟ (πλησιέστερο)        ΝΕΟ (προς τα έξω)')
nd=0
for n in sorted([b for b in blocks if re.match(r'FL0_COLUMN_TEXT\d+$',b)], key=lambda s:int(re.search(r'\d+$',s).group())):
    num = re.search(r'\d+$', n).group()
    o=descr(n,old); w=descr(n,new)
    changed = old[n]!=new[n]
    nd += changed
    tag = 'περιμ.' if ('FL0_COLUMN'+num) in dirs else 'εσωτ. '
    print(f'K{num:<3s} {tag} {o[0]:>2s} {o[1]:5.2f}μ έξω={str(o[2]):5s} | {w[0]:>2s} {w[1]:5.2f}μ έξω={str(w[2]):5s}' + ('   <<< άλλαξε' if changed else ''))
print(f'\nάλλαξαν {nd}/12')
