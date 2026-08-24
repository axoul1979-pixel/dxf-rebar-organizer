import math
import pipeline_v11 as P
from test_perimeter import rect_block
from perimeter import build_footprint, outward_direction
from test_placement import mtext_block

# Κτίριο 12x8, K1 στην αριστερή παρειά. Τώρα ο χώρος ΑΜΕΣΩΣ δυτικά είναι
# πιασμένος (ως x=-1.6) ενώ ΜΕΣΑ στο κτίριο υπάρχει τρύπα στο x=+1.0:
# η "πλησιέστερη ελεύθερη θέση" είναι ΜΕΣΑ. Αυτό ακριβώς θέλουμε να αλλάξει.
blocks = {'FL0_SLAB1': rect_block(0,0,12,8),
          'FL0_COLUMN1': rect_block(-0.3,3.7,0.3,4.3,'column'),
          'FL0_COLUMN_TEXT1': mtext_block(0.0,4.0)}
ins = {n:(0,0) for n in blocks}
fp = build_footprint(blocks, ins)
outward,_ = outward_direction(0.0,4.0,fp)

placed=[]
y=2.0
while y<6.0:                      # φράγμα ΔΥΤΙΚΑ, από x=-1.6 ως x=-0.1
    x=-1.6
    while x<-0.1:
        placed.append((x,y,x+0.3,y+0.25)); x+=0.35
    y+=0.30

obstacle_lines=[(0,0,12,0,'FL0_SLAB1'),(12,0,12,8,'FL0_SLAB1'),
                (12,8,0,8,'FL0_SLAB1'),(0,8,0,0,'FL0_SLAB1')]
seed=(0.0,0.0)
old = P.radial_place_full('FL0_COLUMN_TEXT1',blocks,obstacle_lines,[],placed,('FL0_COLUMN1',),seed=seed)
new,mode = P.place_column_text('FL0_COLUMN_TEXT1','FL0_COLUMN1',blocks,obstacle_lines,[],placed,seed,outward,fp)
print(f'ΠΑΛΙΟ: dx={old[0]:+.2f} dy={old[1]:+.2f} -> {"ΜΕΣΑ στο κτίριο" if old[0]>0 else "έξω"}')
print(f'ΝΕΟ  : dx={new[0]:+.2f} dy={new[1]:+.2f} mode={mode} -> {"ΜΕΣΑ στο κτίριο" if new[0]>0 else "έξω"}')
