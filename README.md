# DXF Rebar Auto-Tidy — Πηγαίος Κώδικας

**Δοκιμασμένο και επιβεβαιωμένο** (φρέσκια διεργασία, χωρίς cache) σε 4 διαφορετικά
πραγματικά αρχεία πριν το τελευταίο πακετάρισμα — δες "ΤΕΛΕΥΤΑΙΟΣ ΕΛΕΓΧΟΣ" στο τέλος.

## Πώς να το τρέξεις σε νέο αρχείο
```python
import re
from pipeline_v11 import process_all
from beambar_engine import get_inserts
from patcher import patch_dxf, patch_block_mtext
from patch_slab_marker import patch_slab_marker_geometry
from patch_style import patch_layer_colors, patch_style, patch_hatch_scale_by_layer
from analyze import get_all_blocks, entities_from_pairs, to_dict

input_path = 'input.dxf'
insert_final, text_local_final = process_all(input_path, is_training_file=False)

# native-offset correction (μόνο για COLUMN_TEXT, που έχει pre-baked offset στο αρχείο)
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

# προαιρετικό: αλλαγή scale hatch σε ΣΥΓΚΕΚΡΙΜΕΝΟ layer (όχι όλα)
# patch_hatch_scale_by_layer('output.dxf', 'output2.dxf', 'ονομα_layer', new_scale=0.03, orig_scale=0.02)
```

`is_training_file=True` ΜΟΝΟ για το αρχικό `input.dxf`/`output.dxf` ζευγάρι.

## Πρόθεμα ορόφου
Υποστηρίζονται αρνητικοί όροφοι (`FL-1_`, `FL-2_`) — το regex σε όλο τον κώδικα είναι
`FL-?\d+_`, όχι `FL\d+_`.

## Αυτόματα, μόνιμα μέρος του process_all() (δεν χρειάζεται να τα ζητήσεις):
- **COLUMN_TEXT seeding**: η αναζήτηση θέσης ξεκινάει από την ΠΡΑΓΜΑΤΙΚΗ θέση της
  κολώνας, όχι από το τοπικό μηδέν του κειμένου (που μπορεί να είναι 2-3μ μακριά χωρίς
  λόγο, απλά επειδή δεν υπήρχε σύγκρουση εκεί)
- **REPAIR PASS**: λύνει συγκρούσεις ράβδου-με-ράβδο μετά την αρχική τοποθέτηση
- **CIRCLE REPAIR**: κυκλάκι πλάκας πάνω σε ράβδο → μικρομετακίνηση κυκλακιού+κειμένου
- **CROSS-CATEGORY REPAIR**: ράβδος (γραμμή Ή κείμενό της) πέφτει πάνω σε ΟΠΟΙΟΔΗΠΟΤΕ
  άλλο κείμενο (beam_text, column_text, δείκτης πλάκας, άλλη ράβδος) → αυτόματη διόρθωση

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

Custom layers (π.χ. `beam_name_beton_found_shear_wall`) γίνονται με απλή patch script
(δες τη συνομιλία/transcript για παραδείγματα) ή με `patch_hatch_scale_by_layer` για
hatch scale σε συγκεκριμένο layer.

## Αρχεία και ρόλος τους
- `pipeline_v11.py` — κύριο pipeline
  - `is_ok_full` — αυστηρός έλεγχος: μηδενική ανοχή σε ράβδο/στατική γραμμή (περιθώριο
    ασφαλείας 0.03)· hatch με περιθώριο 0.25 για ξένο hatch, 0 για το δικό του στοιχείου
  - `is_ok_relaxed` / `count_line_crossings` — επιτρέπει έως 1 γραμμή να διαπερνά
    (ποτέ hatch/άλλο κείμενο), αν πέφτει στον "πυρήνα" ανάγνωσης
  - `radial_place_full(..., seed=(dx,dy))` — σπειροειδής αναζήτηση με ΡΥΘΜΙΖΟΜΕΝΟ
    κέντρο (`seed`) όχι πάντα (0,0) — καθαρή θέση πρώτα, μετά χαλαρή, σε ΚΑΘΕ ακτίνα
  - `beam_text_slide` — ΑΥΣΤΗΡΑ μέσα στο ορθογώνιο της δοκού, ποτέ εκτός· default:
    μένει στη φυσική θέση αν δεν χωράει
- `compute_beambar3.py` — αντιστοίχιση ράβδου↔δοκού: **combined score** =
  `distance + (1-overlap_frac)*2.0 + hook_bonus` σε ΟΛΟΥΣ τους υποψήφιους μαζί (όχι
  tie-break μόνο σε κοντινούς) — μια δοκός με μεγάλη κάλυψη εύρους αλλά ελαφρώς
  μεγαλύτερη απόσταση νικάει μια κοντινή με μηδενική κάλυψη. ΔΕΝ εξετάζει κολώνες ως
  στόχο (δοκιμάστηκε, χάλασε ground truth 17/21→11/21, αναιρέθηκε)
- `compute_slabbar3.py` — αντίστοιχο για slabbar (χαμηλότερη αξιοπιστία, ~25%)
- `compute_column_text.py`, `compute_beamtext_slabmarker.py` — βοηθητικές
- `hatch_engine.py` — `bbox_poly_overlap` σωστός AABB έλεγχος (η παλιά έκδοση έχανε
  επικαλύψεις σε σχήμα "+")
- `patcher.py`, `patch_slab_marker.py`, `patch_style.py` — εγγραφή αλλαγών στο DXF
  (`patch_style.py` περιέχει `patch_layer_colors`, `patch_style`,
  `patch_hatch_scale_by_layer`)
- `engine.py`, `beambar_engine.py`, `analyze.py`, `parse_dxf.py` — DXF parsing
- `global_repair.py` — ΠΡΟΑΙΡΕΤΙΚΟ επιπλέον πέρασμα (πέρα από το ενσωματωμένο),
  greedy τοπική επίλυση, καλείται ξεχωριστά όταν χρειάζεται παραπάνω

## ΓΝΩΣΤΑ ΑΝΟΙΧΤΑ ΘΕΜΑΤΑ (Αύγουστος 2026)
1. **compute_beambar3.py**: ~80% ακρίβεια στο ground truth. Καμία υποστήριξη για
   ράβδους που αγκυρώνουν σε κολώνα αντί για δοκό.
2. **compute_slabbar3.py**: χαμηλή αξιοπιστία (~25%).
3. **BEAM_TEXT σε πολύ στενή/ανύπαρκτη δοκό**: μένει στη φυσική θέση, ποτέ εκτός
   δοκού χωρίς ρητή επιβεβαίωση χρήστη. Μερικά αρχεία export έχουν ΚΕΝΑ blocks BEAM
   (καμία γεωμετρία) — τότε το BEAM_TEXT δεν έχει τίποτα να ελέγξει, μένει native.
4. **CROSS-CATEGORY REPAIR** δεν πιάνει το 100% — τυπικά ~50-70% μείωση συγκρούσεων.

## ΤΕΛΕΥΤΑΙΟΣ ΕΛΕΓΧΟΣ πριν το πακετάρισμα
Τρέχει καθαρά (φρέσκια python διεργασία, χωρίς __pycache__) σε:
`input.dxf`, `karaisk_input.dxf`, `pogon06_or0.dxf`, `pogon06_or-1.dxf` — όλα ΟΚ.
