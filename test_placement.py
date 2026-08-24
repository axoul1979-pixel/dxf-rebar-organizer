import math
import pipeline_v11 as P
from test_perimeter import rect_block
from perimeter import build_footprint, outward_direction

def mtext_block(x, y, content='60/60 12Φ20 ΣΦ10/10', h=0.12):
    return [(0, 'MTEXT'), (8, 'column_name'), (10, str(x)), (20, str(y)),
            (40, str(h)), (50, '0'), (1, content)]

# Κτίριο 12x8 με την κολώνα K1 στην ΑΡΙΣΤΕΡΗ παρειά -> έξοδος προς τα Δυτικά.
blocks = {
    'FL0_SLAB1': rect_block(0, 0, 12, 8),
    'FL0_COLUMN1': rect_block(-0.3, 3.7, 0.3, 4.3, 'column'),
    'FL0_COLUMN_TEXT1': mtext_block(0.0, 4.0),
}
ins = {n: (0, 0) for n in blocks}

fp = build_footprint(blocks, ins)
outward, sector = outward_direction(0.0, 4.0, fp)
print(f'έξοδος K1: {math.degrees(math.atan2(outward[1],outward[0]))%360:.1f}° '
      f'(αναμενόμενο 180°), τόξο {sector:.0f}°')

# Το εσωτερικό είναι ΓΕΜΑΤΟ κείμενα δοκών/πλακών: μια πυκνή σχάρα κατειλημμένων
# κουτιών από x=0.5 ως x=6, ώστε η "πλησιέστερη ελεύθερη θέση" να είναι σαφώς
# μέσα στο κτίριο αλλά πιο πέρα, ενώ αμέσως δυτικά υπάρχει καθαρός χώρος.
placed = []
y = 2.0
while y < 6.0:
    x = 0.5
    while x < 6.0:
        placed.append((x, y, x+0.45, y+0.25))
        x += 0.5
    y += 0.35

obstacle_lines = []
for x1, y1, x2, y2 in [(0, 0, 12, 0), (12, 0, 12, 8), (12, 8, 0, 8), (0, 8, 0, 0)]:
    obstacle_lines.append((x1, y1, x2, y2, 'FL0_SLAB1'))

seed = (0.0, 0.0)  # το κείμενο κάθεται ήδη στην κολώνα

res_old = P.radial_place_full('FL0_COLUMN_TEXT1', blocks, obstacle_lines, [], placed,
                               ('FL0_COLUMN1',), seed=seed)
res_new, mode = P.place_column_text('FL0_COLUMN_TEXT1', 'FL0_COLUMN1', blocks,
                                     obstacle_lines, [], placed, seed, outward, fp)

print(f'\nΠΑΛΙΟ (radial_place_full): dx={res_old[0]:+.2f} dy={res_old[1]:+.2f}  '
      f'-> {"ΜΕΣΑ στο κτίριο" if res_old[0] > 0 else "έξω"}')
print(f'ΝΕΟ  (place_column_text) : dx={res_new[0]:+.2f} dy={res_new[1]:+.2f}  mode={mode}')

assert res_new[0] < 0, 'το νέο κείμενο ΔΕΝ βγήκε δυτικά!'
assert mode == 'outside'
print('\nOK: το κείμενο της περιμετρικής κολώνας βγήκε έξω από το περίγραμμα.')

# Εσωτερική κολώνα -> καμία αλλαγή συμπεριφοράς
blocks['FL0_COLUMN2'] = rect_block(5.7, 3.7, 6.3, 4.3, 'column')
blocks['FL0_COLUMN_TEXT2'] = mtext_block(6.0, 4.0)
ins['FL0_COLUMN2'] = (0, 0); ins['FL0_COLUMN_TEXT2'] = (0, 0)
fp2 = build_footprint(blocks, ins)
print('εσωτερική K2 ->', outward_direction(6.0, 4.0, fp2), '(αναμενόμενο None)')
