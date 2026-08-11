# DXF Rebar Auto-Tidy — Πηγαίος Κώδικας

## Πώς να το τρέξεις σε νέο αρχείο
```python
from pipeline_v11 import process_all
insert_final, text_local_final = process_all('/path/to/input.dxf', is_training_file=False)
```
`is_training_file=True` ενεργοποιείται ΜΟΝΟ για το αρχικό `input.dxf`/`output.dxf` ζευγάρι
(χρησιμοποιεί τις πραγματικές τιμές του output.dxf ως βάση για beambar/slabbar στο *ίδιο*
αρχείο — ΠΟΤΕ μην το βάλεις True σε άλλο αρχείο, θα βάλει τυχαίες/λάθος τιμές).

## Στη συνέχεια, για να παραχθεί το τελικό .dxf:
Δες το τέλος του `pipeline_v11.py` script section, ή το πρόσφατο μήνυμα στη συνομιλία
("build" function) που κάνει: `patch_dxf` → `patch_block_mtext` → `patch_slab_marker_geometry`
→ χρωματισμοί layer → `patch_style` (hatch scale).

## Αρχεία και ρόλος τους
- `pipeline_v11.py` — κύριο pipeline, όλη η λογική τοποθέτησης, ελέγχου επικάλυψης, repair pass
- `compute_beambar3.py` — αντιστοίχιση ράβδου↔δοκού (απόσταση + κάλυψη εύρους + κατεύθυνση άγκιστρου)
- `compute_slabbar3.py` — αντίστοιχο για slabbar (λιγότερο αξιόπιστο, ~25% ακρίβεια σε ground truth)
- `compute_column_text.py`, `compute_beamtext_slabmarker.py` — βοηθητικές συναρτήσεις κειμένου
- `hatch_engine.py` — εξαγωγή/έλεγχος πολυγώνων hatch (bbox-overlap διορθωμένο)
- `patcher.py`, `patch_slab_marker.py`, `patch_style.py` — εγγραφή αλλαγών πίσω στο DXF
- `engine.py`, `beambar_engine.py`, `analyze.py` — χαμηλού επιπέδου DXF parsing

## ΓΝΩΣΤΑ ΑΝΟΙΧΤΑ ΘΕΜΑΤΑ (Αύγουστος 2026)
1. **compute_beambar3.py αντιστοίχιση δοκού**: ~80% ακρίβεια σε ground truth. Ακραίες
   περιπτώσεις (ράβδος πολύ μακριά από τη σωστή της δοκό, π.χ. BEAMBAR8 στο karaisk
   αρχείο, 4.4μ απόσταση) χρειάζονται ακόμα ανθρώπινη επιβεβαίωση.
2. **compute_slabbar3.py**: χαμηλή αξιοπιστία (~25% όταν ελέγχθηκε σε ground truth).
   Χρησιμοποιείται μόνο ως αρχικό σημείο, βασίζεται πολύ στο repair pass για διόρθωση.
3. **"1 ανεκτή διέλευση γραμμής"**: εφαρμόζεται μόνο στο `radial_place_full` (column_text
   και ό,τι το καλεί) — ΔΕΝ εφαρμόζεται ακόμα σε beambar/slabbar/beam_text text-slide.
4. **Multi-pass repair (`global_repair.py`)**: greedy τοπική επίλυση, όχι globικός
   επιλυτής. Φτάνει σε πλατό γύρω στο 50% μείωση συγκρούσεων σε πυκνά σημεία.
5. Δες τη συνομιλία (transcript) για την πλήρη ιστορία bugs που βρέθηκαν/διορθώθηκαν —
   πολλά ήταν θεμελιώδη (π.χ. λάθος αλγόριθμος bbox-overlap που δεν έπιανε επικαλύψεις
   σε σχήμα "+").
