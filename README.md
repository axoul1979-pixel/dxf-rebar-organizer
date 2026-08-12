# DXF Rebar Auto-Tidy — Πηγαίος Κώδικας

## Πώς να το τρέξεις σε νέο αρχείο
```python
import re
from pipeline_v11 import process_all
from beambar_engine import get_inserts
from patcher import patch_dxf, patch_block_mtext
from patch_slab_marker import patch_slab_marker_geometry
from patch_style import patch_layer_colors, patch_style
from analyze import get_all_blocks, entities_from_pairs, to_dict

input_path = 'input.dxf'
insert_final, text_local_final = process_all(input_path, is_training_file=False)

ins_native = dict((n,(x,y)) for n,x,y in get_inserts(input_path))
fixed = dict(insert_final)
for name in list(fixed.keys()):
    if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', name):
        nx, ny = ins_native.get(name, (0.0,0.0))
        dx, dy = fixed[name]
        fixed[name] = (dx - nx, dy - ny)

patch_dxf(input_path, 'stage1.dxf', fixed)

blocks = get_all_blocks(input_path)
block_layer_offsets = {}
for name,(dx,dy) in text_local_final.items():
    ents = entities_from_pairs(blocks[name])
    layers = {to_dict(e).get(8,[''])[0] for e in ents if e[0][1]=='MTEXT'}
    block_layer_offsets[name] = (dx,dy,layers)
patch_block_mtext('stage1.dxf', 'stage2.dxf', block_layer_offsets)

marker_deltas = {name:(dx,dy) for name,(dx,dy) in text_local_final.items() if re.match(r'FL-?\d+_SLAB\d+$',name)}
patch_slab_marker_geometry('stage2.dxf', 'stage3.dxf', marker_deltas)

patch_layer_colors('stage3.dxf', 'stage4.dxf')
patch_style('stage4.dxf', 'output.dxf', hatch_scale=0.02, orig_hatch_scale=0.1)
```

`is_training_file=True` ΜΟΝΟ για το αρχικό `input.dxf`/`output.dxf` ζευγάρι.

## Πρόθεμα ορόφου
Υποστηρίζονται και αρνητικοί όροφοι (`FL-1_`, `FL-2_`) — το regex σε όλο τον κώδικα είναι
`FL-?\d+_`, όχι `FL\d+_`.

## Αυτόματα, μόνιμα μέρος του process_all() (δεν χρειάζεται να τα ζητήσεις):
- **REPAIR PASS**: λύνει επιπλέον συγκρούσεις ράβδου-με-ράβδο μετά την αρχική τοποθέτηση
- **CIRCLE REPAIR**: εντοπίζει κυκλάκια πλάκας που πέφτουν πάνω σε ράβδο οπλισμού και τα
  μικρομετακινεί (μαζί με το κείμενό τους) μέχρι 1μ ακτίνα, σεβόμενο το όριο της πλάκας

## Χρώματα layer (patch_layer_colors)
| Layer | Χρώμα | ACI |
|---|---|---|
| slab_poly | cyan | 4 |
| slab_center | κίτρινο | 2 |
| beam_prefix_name_beton | κίτρινο | 2 |
| slab_name | κίτρινο | 2 |
| beambar_name | κίτρινο | 2 |
| slabbar_name | κίτρινο | 2 |
| slabbar_line | magenta | 6 |
| slab_prefix_name | μπλε | 5 |

## ΓΝΩΣΤΑ ΑΝΟΙΧΤΑ ΘΕΜΑΤΑ (Αύγουστος 2026)
1. **compute_beambar3.py**: ~80% ακρίβεια στο ground truth. Ψάχνει **μόνο** ανάμεσα σε
   δοκούς — ΔΕΝ εξετάζει κολώνες ως πιθανό στόχο αγκύρωσης ράβδου. Δοκιμάστηκε προσθήκη
   κολωνών αλλά χάλασε το ground truth (17/21→11/21) — χρειάζεται προσεκτικότερο
   ξαναχτίσιμο, όχι έτοιμο ακόμα.
2. **compute_slabbar3.py**: χαμηλή αξιοπιστία (~25%).
3. **BEAM_TEXT σε πολύ στενή δοκό**: μένει στη φυσική θέση αν δεν χωράει, ποτέ εκτός
   δοκού χωρίς ρητή επιβεβαίωση χρήστη.
4. **global_repair.py**: προαιρετικό ΕΠΙΠΛΕΟΝ πέρασμα (πέρα από το ενσωματωμένο repair/
   circle-repair) — greedy τοπική επίλυση, καλείται ξεχωριστά όταν χρειάζεται παραπάνω.
