import math
from perimeter import build_footprint, column_outward_dirs, outward_direction, bbox_outside

def rect_block(x1, y1, x2, y2, layer='slab_poly'):
    """Φτιάχνει pairlist block με 4 LINE που κλείνουν ορθογώνιο."""
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    out = []
    for i in range(4):
        a = pts[i]; b = pts[(i+1) % 4]
        out += [(0, 'LINE'), (8, layer),
                (10, str(a[0])), (20, str(a[1])),
                (11, str(b[0])), (21, str(b[1]))]
    return out

def compass(v):
    ang = math.degrees(math.atan2(v[1], v[0])) % 360
    names = ['Α', 'ΒΑ', 'Β', 'ΒΔ', 'Δ', 'ΝΔ', 'Ν', 'ΝΑ']
    return names[int((ang + 22.5) // 45) % 8]

# --- Κάτοψη σχήματος Γ ------------------------------------------------------
#  y=12 +--------+
#       |        |
#  y=6  |        +--------+
#       |                 |
#  y=0  +-----------------+
#      x=0      x=8      x=16
blocks = {}
ins = {}
for i, (x1, y1, x2, y2) in enumerate([(0, 0, 8, 6), (8, 0, 16, 6), (0, 6, 8, 12)]):
    blocks[f'FL0_SLAB{i+1}'] = rect_block(x1, y1, x2, y2)
    ins[f'FL0_SLAB{i+1}'] = (0, 0)

cols = {
    'K_ΚΓΩ_κάτω_αρ': (0, 0),      # γωνία κάτω-αριστερά -> ΝΔ
    'K_ΚΓΩ_κάτω_δε': (16, 0),     # γωνία κάτω-δεξιά    -> ΝΑ
    'K_ΠΛΕΥΡΑ_κάτω': (8, 0),      # μέση κάτω πλευράς   -> Ν
    'K_ΠΛΕΥΡΑ_αρ':   (0, 6),      # μέση αριστ. πλευράς -> Δ
    'K_ΚΓΩ_πανω_αρ': (0, 12),     # γωνία πάνω-αριστερά -> ΒΔ
    'K_ΕΣΟΧΗ':       (8, 6),      # ΕΣΩΤΕΡΙΚΗ γωνία του Γ -> πρέπει ΝΑ/Α, όχι None
    'K_ΕΣΩΤΕΡΙΚΗ':   (4, 3),      # καθαρά μέσα -> None
    'K_ΠΑΝΩ_ΔΕΞΙΑ':  (8, 12),     # γωνία στο πάνω σκέλος -> ΒΑ
}
for name, (cx, cy) in cols.items():
    blocks[name.replace('K_', 'FL0_COLUMN')] = rect_block(cx-0.3, cy-0.3, cx+0.3, cy+0.3, 'column')
    ins[name.replace('K_', 'FL0_COLUMN')] = (0, 0)

fp = build_footprint(blocks, ins)
print(f'Αποτύπωμα: {len(fp["rects"])} ορθογώνια\n')

for name, (cx, cy) in cols.items():
    res = outward_direction(cx, cy, fp)
    if res is None:
        print(f'{name:16s} -> ΕΣΩΤΕΡΙΚΗ (καμία έξοδος)')
    else:
        v, sector = res
        print(f'{name:16s} -> {compass(v):2s}  '
              f'({math.degrees(math.atan2(v[1],v[0]))%360:6.1f}°, ανοιχτό τόξο {sector:5.1f}°)')

# --- έλεγχος bbox_outside ---------------------------------------------------
print()
inside_box = (3.0, 2.0, 5.0, 3.0)      # μέσα στην πλάκα
outside_box = (-4.0, -3.0, -2.0, -2.0)  # έξω κάτω-αριστερά
edge_box = (-0.05, 2.0, 1.0, 3.0)       # πατάει την παρειά
print('κουτί μέσα στην πλάκα  -> outside;', bbox_outside(fp, inside_box))
print('κουτί έξω από το κτίριο -> outside;', bbox_outside(fp, outside_box))
print('κουτί στην παρειά       -> outside;', bbox_outside(fp, edge_box))
