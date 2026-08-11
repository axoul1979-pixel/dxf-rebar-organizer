# DXF Rebar Auto-Tidy — Πηγαίος Κώδικας

## Πώς να το τρέξεις σε νέο αρχείο
```python
import re, pickle
from pipeline_v11 import process_all
from beambar_engine import get_inserts
from patcher import patch_dxf, patch_block_mtext
from patch_slab_marker import patch_slab_marker_geometry
from patch_style import patch_layer_colors, patch_style
from analyze import get_all_blocks, entities_from_pairs, to_dict

input_path = 'input.dxf'
insert_final, text_local_final = process_all(input_path, is_training_file=False)

# native-offset correction (μόνο για COLUMN_TEXT, που έχει pre-baked offset στο αρχείο)
ins_native = dict((n,(x,y)) for n,x,y in get_inserts(input_path))
fixed = dict(insert_final)
for name in list(fixed.keys()):
    if re.match(r'FL\d+_COLUMN_TEXT\d+$', name):
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

marker_deltas = {name:(dx,dy) for name,(dx,dy) in text_local_final.items() if re.match(r'FL\d+_SLAB\d+$',name)}
patch_slab_marker_geometry('stage2.dxf', 'stage3.dxf', marker_deltas)

# χρώματα layer: όλα τα 8 σε ένα κλήση, βλέπε patch_style.patch_layer_colors για τη λίστα
patch_layer_colors('stage3.dxf', 'stage4.dxf')

patch_style('stage4.dxf', 'output.dxf', hatch_scale=0.02, orig_hatch_scale=0.1)
```

`is_training_file=True` ενεργοποιείται ΜΟΝΟ για το αρχικό `input.dxf`/`output.dxf` ζευγάρι
(χρησιμοποιεί τις πραγματικές τιμές του output.dxf ως βάση για beambar/slabbar στο *ίδιο*
αρχείο — ΠΟΤΕ μην το βάλεις True σε άλλο αρχείο).

## Χρώματα layer (patch_style.patch_layer_colors)
Προεπιλεγμένη παλέτα (μπορείς να δώσεις δικό σου `color_map` dict):
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

Layers που δεν υπάρχουν σε ένα συγκεκριμένο αρχείο παραλείπονται αθόρυβα (δεν κάθε
αρχείο έχει όλα αυτά τα layers).

## Προαιρετικό: επιπλέον πέρασμα επισκευής (multi-pass repair)
Το `global_repair.py` περιέχει βοηθητικές συναρτήσεις για ένα ΔΕΥΤΕΡΟ πέρασμα που
προσπαθεί να λύσει επιπλέον συγκρούσεις μετά το `process_all()`, δοκιμάζοντας να
μετακινήσει είτε το ένα είτε το άλλο από δύο συγκρουόμενα στοιχεία (κείμενο, σίδερο,
ή κυκλάκι πλάκας — με σεβασμό στο όριο της πλάκας για slabbar). Δεν είναι μέρος του
βασικού `process_all()` — καλείται ξεχωριστά. Δες τη συνομιλία (transcript) για το
ακριβές usage pattern (function: `find_conflicts`, `try_move_text_only`,
`try_move_marker`, `try_move_bar`).

## Αρχεία και ρόλος τους
- `pipeline_v11.py` — κύριο pipeline: τοποθέτηση, έλεγχος επικάλυψης, repair pass
  - `is_ok_full` — αυστηρός έλεγχος (μηδενική ανοχή σε ράβδο/στατική γραμμή, με μικρό
    περιθώριο ασφαλείας 0.03· hatch με περιθώριο 0.25 για ξένο hatch, 0 για το δικό του
    στοιχείου — π.χ. label δίπλα στη δική του κολώνα)
  - `is_ok_relaxed` / `count_line_crossings` — χαλαρός έλεγχος, επιτρέπει έως 1 γραμμή
    να διαπερνά (ποτέ hatch/άλλο κείμενο), αν πέφτει στον "πυρήνα" ανάγνωσης
  - `radial_place_full` — ενιαία σπειροειδής αναζήτηση (καθαρή θέση πρώτα, μετά χαλαρή,
    σε ΚΑΘΕ ακτίνα πριν προχωρήσει παραπέρα — εγγυάται πραγματικά την πλησιέστερη έγκυρη
    θέση, όχι απλώς την πρώτη που βρίσκεται)
  - `beam_text_slide` — ολίσθηση beam_text ΑΥΣΤΗΡΑ μέσα στο ορθογώνιο της δοκού
    (κατά μήκος + πλάτος, πλήρες κουτί όχι μόνο σημείο αναφοράς), ποτέ εκτός· default:
    μένει στη φυσική θέση αν δεν χωράει (καμία αυτόματη εξαίρεση)
- `compute_beambar3.py` — αντιστοίχιση ράβδου↔δοκού (απόσταση + κάλυψη εύρους ≥50% ως
  tie-break εντός 0.35 απόστασης + κατεύθυνση άγκιστρου· Άνω/Κάτω κατεύθυνση μόνο σε
  οριζόντιες δοκούς — σε κάθετες δοκούς η έννοια "πάνω/κάτω" δεν έχει νόημα)
- `compute_slabbar3.py` — αντίστοιχο για slabbar (χαμηλότερη αξιοπιστία, ~25% ground truth)
- `compute_column_text.py`, `compute_beamtext_slabmarker.py` — βοηθητικές συναρτήσεις κειμένου
- `hatch_engine.py` — `bbox_poly_overlap` (ΔΙΟΡΘΩΜΕΝΟ: σωστός AABB έλεγχος ορθογωνίου-με-
  ορθογώνιο· η παλιά έκδοση έχανε επικαλύψεις σε σχήμα "+")
- `patcher.py`, `patch_slab_marker.py`, `patch_style.py` — εγγραφή αλλαγών πίσω στο DXF
  (θέση κειμένου, θέση κυκλακιού/crosshair πλάκας, χρώμα layer, κλίμακα hatch)
- `engine.py`, `beambar_engine.py`, `analyze.py`, `parse_dxf.py` — χαμηλού επιπέδου DXF
  parsing (`text_width` στο beambar_engine.py: εκτίμηση πλάτους ανά χαρακτήρα, βαθμονομημένη)
- `global_repair.py` — προαιρετικό δεύτερο πέρασμα επισκευής (βλ. πάνω)

## ΓΝΩΣΤΑ ΑΝΟΙΧΤΑ ΘΕΜΑΤΑ (Αύγουστος 2026)
1. **compute_beambar3.py αντιστοίχιση δοκού**: ~80% ακρίβεια σε ground truth. Ακραίες
   περιπτώσεις (ράβδος πολύ μακριά από τη σωστή της δοκό, έστω και 4+ μέτρα) χρειάζονται
   ανθρώπινη επιβεβαίωση.
2. **compute_slabbar3.py**: χαμηλή αξιοπιστία (~25%). Χρησιμοποιείται μόνο ως αρχικό
   σημείο· το repair pass κάνει τη μεγαλύτερη δουλειά διόρθωσης.
3. **BEAM_TEXT σε πολύ στενή δοκό**: αν το κείμενο (πολλές γραμμές) δεν χωράει στο πλάτος
   της δοκού, μένει στη φυσική θέση (ίσως με επικάλυψη) αντί να βγει έξω — αυστηρός
   κανόνας κατόπιν ρητής οδηγίας χρήστη. Καμία αυτόματη εξαίρεση χωρίς ρητή επιβεβαίωση.
4. **Multi-pass repair (`global_repair.py`)**: greedy τοπική επίλυση, όχι globικός
   επιλυτής. Φτάνει σε πλατό γύρω στο 50% μείωση συγκρούσεων σε πυκνά σημεία.
5. Δες τη συνομιλία (transcript) για την πλήρη ιστορία bugs που βρέθηκαν/διορθώθηκαν.
