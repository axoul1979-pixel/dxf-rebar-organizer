"""
final_pass.py - Τελικό πέρασμα ΜΕΤΑ την τακτοποίηση (τρέχει πάνω στην έξοδο
του run_any.py), εφαρμόζει ΚΑΝΟΝΙΣΜΟΣ.md §0-§3, §8 για τους οπλισμούς πλάκας
(SLABBAR): κάθετη μετακίνηση ράβδου εντός των ορίων της πλάκας της, μετά
ολίσθηση+μικρή κάθετη μετατόπιση της ετικέτας ώστε να ακουμπάει τη ράβδο.

ΚΡΙΣΙΜΟ (v2): ο έλεγχος "γραμμή κόβει κείμενο" χρησιμοποιεί την ΙΔΙΑ ακριβώς
συνάρτηση `parallel_cut_full` του pipeline_v11.py (cos>0.94 από τη φορά
ανάγνωσης - Κανόνας 5 του README) που χρησιμοποιεί και το audit_output.py. Η
πρώτη έκδοση είχε τη δική της, πιο χονδρή, εκδοχή («κάθε τομή μετράει») που
βελτιστοποιούσε για λάθος στόχο και χειροτέρευε το AUDIT_TOTAL (7 -> 27). Ο
έλεγχος ζώνης χρησιμοποιεί το ΙΔΙΟ slab_region.py με το audit_output.py, ώστε
οι δύο "κριτές" να συμφωνούν πάντα.

Χρήση:
    python3 final_pass.py ΤΑΚΤΟΠΟΙΗΜΕΝΟ.dxf ΕΞΟΔΟΣ.dxf
"""
import sys, re, math
from collections import defaultdict
from analyze import entities_from_pairs, to_dict
from engine import load_all, block_lines_local
from beambar_engine import text_bbox, seg_intersects_bbox
from hatch_engine import get_hatch_polys, bbox_poly_overlap
from compute_column_text import block_text_bboxes, union_bbox, translate_bbox
from slab_region import all_slab_regions, bar_reference_point
from patcher import patch_dxf, patch_block_mtext
from patch_slab_marker import patch_slab_marker_geometry
from compute_beamtext_slabmarker import slab_marker_boxes
import pipeline_v11 as P11

STEP = 0.10
MAX_STEPS = 25
LBL_STEP = 0.05
LBL_MAX_ALONG = 1.20
LBL_MAX_PERP = 0.35
PAD = 0.0

SLABBAR_RE = r'FL-?\d+_SLABBAR\d+$'
# §3 «η ετικέτα κάθεται δίπλα στη ράβδο, όχι πάνω της» ισχύει για ΚΑΘΕ ετικέτα
# οπλισμού - και των δοκών (BEAMBAR), όχι μόνο των πλακών
REBAR_RE = r'FL-?\d+_(SLABBAR|BEAMBAR)\d+$'
BEAMTEXT_RE = r'(FL-?\d+)_BEAM_TEXT(\d+)$'
TEXT_OWNER_RE = r'(BEAM_TEXT|COLUMN_TEXT|BEAMBAR|SLABBAR|SLAB)\d+$'


def _world_lines(blocks, ins, name):
    ox, oy = ins.get(name, (0, 0))
    lines, _ = block_lines_local(blocks[name])
    return [(x1+ox, y1+oy, x2+ox, y2+oy) for x1, y1, x2, y2 in lines]


def _axis_and_perp(lines):
    best = None; bl = -1
    for x1, y1, x2, y2 in lines:
        l = math.hypot(x2-x1, y2-y1)
        if l > bl:
            bl = l; best = (x1, y1, x2, y2)
    x1, y1, x2, y2 = best
    ux, uy = (x2-x1)/bl, (y2-y1)/bl
    return (ux, uy), (-uy, ux)


def _translate_lines(lines, dx, dy):
    return [(x1+dx, y1+dy, x2+dx, y2+dy) for x1, y1, x2, y2 in lines]


class Model:
    def __init__(self, input_path, orig_path=None):
        self.path = input_path
        self.ins, self.blocks = load_all(input_path)
        # το ΠΡΩΤΟΤΥΠΟ (προ τακτοποίησης) χρειάζεται για τον §4: το κριτήριο
        # δεν είναι «κεντραρισμένο» (στο FESPA ΟΛΑ τα κείμενα δοκού έχουν
        # σταθερή απόκλιση ~-0.05 από τον άξονα - σκόπιμη), αλλά «πόσο ΚΑΘΕΤΑ
        # μετακινήθηκε σε σχέση με την αρχική του θέση»: §4 λέει 99% μόνο
        # ολίσθηση ΚΑΤΑ ΜΗΚΟΣ.
        self.orig_ins = None
        if orig_path:
            try:
                self.orig_ins, _ = load_all(orig_path)
            except Exception:
                self.orig_ins = None
        self.hatch_polys = get_hatch_polys(input_path)
        self.slab_regions = all_slab_regions(orig_path or input_path)

        self.text_local = {}
        for name, pl in self.blocks.items():
            if not re.search(TEXT_OWNER_RE, name):
                continue
            boxes = block_text_bboxes(pl)
            if not boxes:
                continue
            ox, oy = self.ins.get(name, (0, 0))
            self.text_local[name] = [(x+ox, y+oy, w, h, rot) for x, y, w, h, rot in boxes]

        self.bar_lines = {}
        for name in self.blocks:
            if re.match(REBAR_RE, name):
                self.bar_lines[name] = _world_lines(self.blocks, self.ins, name)

        # obstacle_lines στη μορφή που περιμένει το pipeline_v11.py:
        # (x1,y1,x2,y2,name) - δομικά + ΟΛΟΙ οι οπλισμοί (BEAMBAR+SLABBAR),
        # ώστε ο ίδιος έλεγχος (is_ok_full/parallel_cut_full) να ισχύει παντού.
        self.obstacle_lines = P11.build_obstacle_lines(self.blocks, self.ins)
        for name in self.blocks:
            if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', name):
                for x1, y1, x2, y2 in _world_lines(self.blocks, self.ins, name):
                    self.obstacle_lines.append((x1, y1, x2, y2, name))

        # κυκλάκια δεικτών πλάκας (slab_center CIRCLE) ως κουτιά-εμπόδια: το
        # is_ok_full ελέγχει placed_boxes και γραμμές, όχι κύκλους - χωρίς αυτό
        # το πέρασμα έβαζε ετικέτες πάνω σε κυκλάκια (2 νέες παραβιάσεις Γ στο
        # AISXYL00, βάση Γ=0) ενώ ο ελεγκτής τις έπιανε. Ίδιος κριτής παντού.
        self.circle_boxes = {}
        for name, pl in self.blocks.items():
            if not re.match(r'FL-?\d+_SLAB\d+$', name):
                continue
            ox, oy = self.ins.get(name, (0, 0))
            for e in entities_from_pairs(pl):
                if e[0][1] != 'CIRCLE':
                    continue
                d = to_dict(e)
                if d.get(8, [''])[0] != 'slab_center':
                    continue
                cx = float(d[10][0])+ox; cy = float(d[20][0])+oy; r = float(d[40][0])
                self.circle_boxes.setdefault(name, []).append((cx-r, cy-r, cx+r, cy+r))

        # ΑΡΧΙΚΗ κάθετη απόσταση ετικέτας-ράβδου, από το αρχείο-ΜΗΤΡΑ. Το FESPA
        # την ορίζει σταθερή (μετρημένο: 0,141 / 0,101 / 0,061) και ΠΟΤΕ δεν
        # αφήνει την ετικέτα πάνω στη γραμμή της. Άρα δεν την «υπολογίζουμε»
        # εμείς: τη ΔΙΑΤΗΡΟΥΜΕ. Επιτρέπεται ΜΟΝΟ ολίσθηση κατά μήκος (§3/§4).
        self.orig_perp = {}
        if orig_path:
            try:
                oins, oblocks = load_all(orig_path)
                for nm in oblocks:
                    if not re.match(REBAR_RE, nm):
                        continue
                    oox, ooy = oins.get(nm, (0, 0))
                    ols, _ = block_lines_local(oblocks[nm])
                    if not ols:
                        continue
                    wl = [(a+oox, b+ooy, c+oox, d+ooy) for a, b, c, d in ols]
                    bst = max(wl, key=lambda q: math.hypot(q[2]-q[0], q[3]-q[1]))
                    L = math.hypot(bst[2]-bst[0], bst[3]-bst[1])
                    if L < 1e-9:
                        continue
                    oux, ouy = (bst[2]-bst[0])/L, (bst[3]-bst[1])/L
                    opx, opy = -ouy, oux
                    barp = ((bst[0]+bst[2])/2)*opx + ((bst[1]+bst[3])/2)*opy
                    obs = block_text_bboxes(oblocks[nm])
                    if not obs:
                        continue
                    bb = None
                    for x_, y_, w_, h_, r_ in obs:
                        t_ = text_bbox(x_+oox, y_+ooy, w_, h_, r_)
                        bb = t_ if bb is None else (min(bb[0], t_[0]), min(bb[1], t_[1]),
                                                     max(bb[2], t_[2]), max(bb[3], t_[3]))
                    cp = ((bb[0]+bb[2])/2)*opx + ((bb[1]+bb[3])/2)*opy
                    self.orig_perp[nm] = cp - barp
            except Exception:
                self.orig_perp = {}

        self.deltas = defaultdict(lambda: [0.0, 0.0])
        self.label_deltas = defaultdict(lambda: [0.0, 0.0])
        self.marker_deltas = defaultdict(lambda: [0.0, 0.0])
        self.marker_names = set(n for n in self.blocks if re.match(r'FL-?\d+_SLAB\d+$', n))
        # ΔΟΜΙΚΕΣ γραμμές (δοκοί/κολώνες) ξεχωριστά: μια τέτοια γραμμή που περνά
        # ΚΑΘΕΤΑ μέσα από τα γράμματα τα κόβει εξίσου, αλλά δεν την έπιανε το
        # κριτήριο «παράλληλης τομής» (cos>0.94) - 11 ετικέτες έμεναν αόρατα
        # κομμένες στο AISXYL00.
        self.struct_only = []
        for _n in self.blocks:
            if re.match(r'FL-?\d+_(BEAM|COLUMN)\d+$', _n) and 'TEXT' not in _n:
                self.struct_only.extend(_world_lines(self.blocks, self.ins, _n))
        # ΚΑΝΟΝΑΣ ΠΕΡΙΜΕΤΡΙΚΩΝ (υπερισχύει της ιδιοκτησίας): για μια περιμετρική
        # κολώνα το κείμενο ανήκει ΕΞΩ. Χωρίς αυτό, το πέρασμα ιδιοκτησίας
        # τραβούσε πίσω μέσα ό,τι είχε σωστά βγει έξω (τεκμηριωμένο: Κ7).
        try:
            from perimeter import build_footprint, column_outward_dirs, point_inside
            self.footprint = build_footprint(self.blocks, self.ins)
            self.outward = column_outward_dirs(self.blocks, self.ins, self.footprint)
            self._point_inside = point_inside
        except Exception:
            self.footprint = None
            self.outward = {}
            self._point_inside = None

    def is_outside(self, bb):
        """True αν το κουτί κείμενου βρίσκεται έξω από το αποτύπωμα του κτιρίου."""
        if self.footprint is None or self._point_inside is None:
            return None
        cx, cy = (bb[0]+bb[2])/2, (bb[1]+bb[3])/2
        try:
            return not self._point_inside(self.footprint, cx, cy)
        except Exception:
            return None

    def refresh_obstacle_lines_for(self, name, new_lines):
        """Ενημερώνει τις γραμμές ενός block μέσα στο obstacle_lines μετά από
        μετακίνηση ράβδου, ώστε οι επόμενοι έλεγχοι να βλέπουν τη ΝΕΑ θέση."""
        self.obstacle_lines = [s for s in self.obstacle_lines if s[4] != name]
        for x1, y1, x2, y2 in new_lines:
            self.obstacle_lines.append((x1, y1, x2, y2, name))

    def placed_boxes_excluding(self, exclude):
        out = []
        for name, boxes in self.text_local.items():
            if name in exclude:
                continue
            for x, y, w, h, rot in boxes:
                out.append(text_bbox(x, y, w, h, rot))
        for name, cbs in self.circle_boxes.items():
            if name in exclude:
                continue
            out.extend(cbs)
        return out

    def line_cuts_any_text(self, line, exclude):
        seg = (line[0], line[1], line[2], line[3], '__CAND__')
        for name, boxes in self.text_local.items():
            if name in exclude:
                continue
            if P11.parallel_cut_full(boxes, 0.0, 0.0, [seg]):
                return True
        return False

    def lines_cut_any_text(self, lines, exclude):
        return any(self.line_cuts_any_text(l, exclude) for l in lines)

    def label_ok(self, boxes_local, dx, dy, exclude):
        """ΟΛΟΚΛΗΡΟΣ ο έλεγχος (hatch+κενά+παράλληλη τομή) με την ίδια ακριβώς
        συνάρτηση που χρησιμοποιεί ο κύριος αγωγός - δεν ξαναφτιάχνω κομμάτια."""
        placed = self.placed_boxes_excluding(exclude)
        return P11.is_ok_full(boxes_local, dx, dy, self.obstacle_lines, self.hatch_polys,
                               placed, exclude_line_names=exclude)

    def marker_ok(self, name, dx, dy):
        """Ο δείκτης πλάκας (κείμενα + κυκλάκι μαζί) σε νέα θέση: κανένα κείμενο
        άλλου block να μην πέφτει μέσα στο κυκλάκι ή πάνω στα κείμενά του, και
        καμία γραμμή να μην κόβει τα κείμενά του. Το ΔΙΚΟ του slab_poly δεν
        μετράει (είναι το περίγραμμα της πλάκας του, κάθεται μέσα του εξ ορισμού)."""
        excl = {name}
        boxes_local = [(x, y, w, h, rot) for x, y, w, h, rot in self.text_local.get(name, [])]
        placed = self.placed_boxes_excluding(excl)
        if not P11.is_ok_full(boxes_local, dx, dy, self.obstacle_lines, self.hatch_polys,
                               placed, exclude_line_names=excl):
            return False
        for cb in self.circle_boxes.get(name, []):
            m = (cb[0]+dx, cb[1]+dy, cb[2]+dx, cb[3]+dy)
            for other, boxes in self.text_local.items():
                if other == name:
                    continue
                for x, y, w, h, rot in boxes:
                    b = text_bbox(x, y, w, h, rot)
                    if not (m[2] < b[0] or b[2] < m[0] or m[3] < b[1] or b[3] < m[1]):
                        return False
        return True

    def box_on_hatch_bb(self, box):
        """§7: το hatch είναι εμπόδιο για ΟΛΑ τα κείμενα (πραγματικό πολύγωνο)."""
        for poly, pn in self.hatch_polys:
            if bbox_poly_overlap(box, poly):
                return True
        return False

    def label_touches_bar(self, box, lines, tol=0.03):
        """ΚΑΝΟΝΙΣΜΟΣ §3: η ετικέτα κάθεται ΔΙΠΛΑ στη ράβδο, όχι ΠΑΝΩ της.
        «Ακουμπάει» σημαίνει: η γραμμή περνά από τη ΖΩΝΗ ΑΚΡΗΣ του κουτιού
        (επαφή), αλλά ΔΕΝ μπαίνει στον πυρήνα όπου είναι οι χαρακτήρες.
        Η πρώτη έκδοση απαιτούσε τομή με τον ΠΥΡΗΝΑ - δηλαδή ζητούσε από τη
        γραμμή να κόβει τα γράμματα - και κεντράρισε 18 ετικέτες ΠΑΝΩ στη
        ράβδο τους (εντοπίστηκε από τον μηχανικό σε πραγματικό AutoCAD)."""
        outer = (box[0]-tol, box[1]-tol, box[2]+tol, box[3]+tol)
        if not any(seg_intersects_bbox(seg[:4], outer) for seg in lines):
            return False  # δεν ακουμπάει καν
        ix1, iy1, ix2, iy2 = box[0]+tol, box[1]+tol, box[2]-tol, box[3]-tol
        if ix1 < ix2 and iy1 < iy2:
            if any(seg_intersects_bbox(seg[:4], (ix1, iy1, ix2, iy2)) for seg in lines):
                return False  # μέσα στους χαρακτήρες: κομμένη, δεν διαβάζεται
        return True


def process_slabbar(model, name):
    lines0 = model.bar_lines[name]
    label_boxes0 = model.text_local.get(name)
    if not label_boxes0:
        return
    label_box0 = text_bbox(*(list(label_boxes0[0])))
    for x, y, w, h, rot in label_boxes0[1:]:
        b = text_bbox(x, y, w, h, rot)
        label_box0 = (min(label_box0[0], b[0]), min(label_box0[1], b[1]),
                      max(label_box0[2], b[2]), max(label_box0[3], b[3]))

    ref = bar_reference_point(lines0)
    region = None
    if ref:
        for rname, bb in model.slab_regions.items():
            if bb[0] <= ref[0] <= bb[2] and bb[1] <= ref[1] <= bb[3]:
                region = bb; break

    (ux, uy), (px, py) = _axis_and_perp(lines0)

    best_d = 0.0
    best_cut = model.lines_cut_any_text(lines0, {name})
    best_lines = lines0

    # Η ράβδος κινείται (§3, κάθετα εντός πλάκας) και όταν η ΔΙΚΗ ΤΗΣ ετικέτα
    # κόβεται από ΞΕΝΗ ράβδο: αφού η ετικέτα δεν επιτρέπεται να μετακινηθεί
    # κάθετα (διατηρεί την αρχική της απόσταση), ο μόνος τρόπος να καθαρίσει
    # είναι να μετακινηθεί ολόκληρο το ζεύγος ράβδος+ετικέτα.
    if not best_cut and name in model.orig_perp:
        lb0 = translate_bbox(label_box0, 0.0, 0.0)
        inn0 = (lb0[0]+0.03, lb0[1]+0.03, lb0[2]-0.03, lb0[3]-0.03)
        if inn0[0] < inn0[2] and inn0[1] < inn0[3]:
            for _m, _segs in model.bar_lines.items():
                if _m != name and any(seg_intersects_bbox(_s[:4], inn0) for _s in _segs):
                    best_cut = True
                    break

    if region is not None and best_cut:
        for k in range(1, MAX_STEPS+1):
            found = False
            for sign in (1, -1):
                d = k*STEP*sign
                rx = ref[0]+px*d; ry = ref[1]+py*d
                if not (region[0]-PAD <= rx <= region[2]+PAD and region[1]-PAD <= ry <= region[3]+PAD):
                    continue  # §0 ΑΠΑΡΑΒΑΤΟ
                cand_lines = _translate_lines(lines0, px*d, py*d)
                cut = model.lines_cut_any_text(cand_lines, {name})
                if not cut:
                    cl = translate_bbox(label_box0, px*d, py*d)
                    ci = (cl[0]+0.03, cl[1]+0.03, cl[2]-0.03, cl[3]-0.03)
                    if ci[0] < ci[2] and ci[1] < ci[3]:
                        for _m, _segs in model.bar_lines.items():
                            if _m == name:
                                continue
                            if any(seg_intersects_bbox(_s[:4], ci) for _s in _segs):
                                cut = True
                                break
                if not cut:
                    cl2 = translate_bbox(label_box0, px*d, py*d)
                    bshift = [(x_+px*d, y_+py*d, w_, h_, r_)
                               for x_, y_, w_, h_, r_ in model.text_local.get(name, [])]
                    if P11.parallel_cut_full(bshift, 0.0, 0.0, model.obstacle_lines,
                                              exclude_line_names={name}):
                        continue
                    ovl = False
                    for other_, obs_ in model.text_local.items():
                        if other_ == name:
                            continue
                        for x_, y_, w_, h_, r_ in obs_:
                            ob_ = text_bbox(x_, y_, w_, h_, r_)
                            if not (cl2[2] < ob_[0]-0.03 or ob_[2]+0.03 < cl2[0] or
                                     cl2[3] < ob_[1]-0.03 or ob_[3]+0.03 < cl2[1]):
                                ovl = True
                                break
                        if ovl:
                            break
                    if ovl:
                        continue
                    best_d, best_cut, best_lines = d, False, cand_lines
                    found = True
                    break
            if found:
                break

    dx, dy = px*best_d, py*best_d
    if dx or dy:
        model.deltas[name][0] += dx
        model.deltas[name][1] += dy
        model.bar_lines[name] = best_lines
        model.text_local[name] = [(x+dx, y+dy, w, h, rot) for x, y, w, h, rot in model.text_local[name]]
        model.refresh_obstacle_lines_for(name, best_lines)
        label_box0 = translate_bbox(label_box0, dx, dy)

    # --- ετικέτα: ολίσθηση κατά μήκος + μικρή κάθετη μετατόπιση (§3),
    #     αποδεκτή ΜΟΝΟ αν περνάει τον ΠΛΗΡΗ έλεγχο is_ok_full, ώστε να μη
    #     δημιουργεί κανένα νέο πρόβλημα καμίας κατηγορίας. Ανάμεσα σε όσες
    #     θέσεις περνάνε, προτιμάται η πιο κεντραρισμένη στο μήκος της ράβδου.
    lines_now = model.bar_lines[name]
    lo = min(min(seg[0]*ux+seg[1]*uy, seg[2]*ux+seg[3]*uy) for seg in lines_now)
    hi = max(max(seg[0]*ux+seg[1]*uy, seg[2]*ux+seg[3]*uy) for seg in lines_now)
    mid_t = (lo+hi)/2
    cur_cx, cur_cy = (label_box0[0]+label_box0[2])/2, (label_box0[1]+label_box0[3])/2
    cur_t = cur_cx*ux+cur_cy*uy
    cur_p = cur_cx*px+cur_cy*py
    half_w, half_h = (label_box0[2]-label_box0[0])/2, (label_box0[3]-label_box0[1])/2

    boxes_local0 = [(x, y, w, h, rot) for x, y, w, h, rot in model.text_local[name]]

    def candidate_ok_and_touch(ldx, ldy):
        cb = translate_bbox(label_box0, ldx, ldy)
        if not model.label_touches_bar(cb, lines_now):
            return False
        return model.label_ok(boxes_local0, ldx, ldy, {name})

    def score_pos(ldx, ldy):
        """Βαθμολογία θέσης ετικέτας κατά ΚΑΝΟΝΙΣΜΟ §1 (μικρότερο=καλύτερο):
        1) κομμένη από τη ΔΙΚΗ της ράβδο (§3 «δίπλα, όχι πάνω» - αυτό ακριβώς
           βλέπει ο μηχανικός), 2) επικάλυψη με άλλο κείμενο, 3) πάνω σε hatch,
        4) ξεκολλημένη από τη ράβδο, 5) απόσταση από το κέντρο της ράβδου."""
        cb = translate_bbox(label_box0, ldx, ldy)
        tol = 0.03
        inner = (cb[0]+tol, cb[1]+tol, cb[2]-tol, cb[3]-tol)
        cut_by_own = 0
        if inner[0] < inner[2] and inner[1] < inner[3]:
            if any(seg_intersects_bbox(s[:4], inner) for s in lines_now):
                cut_by_own = 1
        outer = (cb[0]-tol, cb[1]-tol, cb[2]+tol, cb[3]+tol)
        detached = 0 if any(seg_intersects_bbox(s[:4], outer) for s in lines_now) else 1
        # επικάλυψη ΚΑΙ οριακό κενό (<0.03) μαζί - ο ελεγκτής τα μετράει και τα
        # δύο (κατηγορίες Α/Γ και Β), οπότε τα βλέπει και η βαθμολογία
        GAP = 0.03
        near = (cb[0]-GAP, cb[1]-GAP, cb[2]+GAP, cb[3]+GAP)
        overl = 0
        for other, obs in model.text_local.items():
            if other == name:
                continue
            for x_, y_, w_, h_, r_ in obs:
                ob = text_bbox(x_, y_, w_, h_, r_)
                if not (near[2] < ob[0] or ob[2] < near[0] or near[3] < ob[1] or ob[3] < near[1]):
                    overl += 1
        for mk_ in model.marker_names:
            for cbx in model.circle_boxes.get(mk_, []):
                if not (near[2] < cbx[0] or cbx[2] < near[0] or near[3] < cbx[1] or cbx[3] < near[1]):
                    overl += 1
        # παράλληλη τομή από ΞΕΝΕΣ γραμμές (κατηγορία Δ) - ίδια συνάρτηση με τον
        # ελεγκτή, ώστε το fallback να μη «λύνει» το Ζ δημιουργώντας Δ
        boxes_shift = [(x_+ldx, y_+ldy, w_, h_, r_) for x_, y_, w_, h_, r_ in boxes_local0]
        pcut = 1 if P11.parallel_cut_full(boxes_shift, 0.0, 0.0, model.obstacle_lines,
                                            exclude_line_names={name}) else 0
        # τομή από ΞΕΝΗ ράβδο σε ΟΠΟΙΑΔΗΠΟΤΕ γωνία (§1: «γραμμή που κόβει
        # κείμενο» - η γωνία δεν αλλάζει το ότι τα γράμματα κόβονται)
        foreign_cut = 0
        if inner[0] < inner[2] and inner[1] < inner[3]:
            for _m, _segs in model.bar_lines.items():
                if _m == name:
                    continue
                if any(seg_intersects_bbox(_s[:4], inner) for _s in _segs):
                    foreign_cut = 1
                    break
        # τομή από ΔΟΜΙΚΗ γραμμή (δοκού/κολώνας) σε οποιαδήποτε γωνία - ίδιο
        # κριτήριο με το _label_badness, ώστε τα δύο περάσματα να μη διαφωνούν
        struct_cut = 0
        if inner[0] < inner[2] and inner[1] < inner[3]:
            if any(seg_intersects_bbox(s_[:4], inner) for s_ in model.struct_only):
                struct_cut = 1
        on_hatch = 1 if model.box_on_hatch_bb(cb) else 0
        t_ = ((cb[0]+cb[2])/2)*ux + ((cb[1]+cb[3])/2)*uy
        return (cut_by_own, overl, foreign_cut, struct_cut, pcut, on_hatch, detached, abs(t_-mid_t))

    best = None  # (ldx, ldy, |t-mid_t|)
    if candidate_ok_and_touch(0.0, 0.0):
        best = (0.0, 0.0, abs(cur_t-mid_t))
    cur_score = score_pos(0.0, 0.0)
    fallback = (0.0, 0.0, cur_score)

    # ΚΑΝΟΝΙΣΜΟΣ §3 «δίπλα, όχι πάνω»: οι μόνες σωστές κάθετες θέσεις είναι οι
    # δύο ΕΦΑΠΤΟΜΕΝΕΣ - το κουτί με την άκρη του πάνω στη γραμμή, από τη μία ή
    # την άλλη πλευρά. Υπολογίζονται ΑΚΡΙΒΩΣ, όχι με τυφλή σάρωση βήματος 0.05
    # που μπορεί να πηδήξει πάνω από τη στενή ζώνη επαφής.
    seg_main = max(lines_now, key=lambda s: math.hypot(s[2]-s[0], s[3]-s[1]))
    bar_p = ((seg_main[0]+seg_main[2])/2)*px + ((seg_main[1]+seg_main[3])/2)*py
    ext_p = half_w*abs(px) + half_h*abs(py)
    # ΚΑΝΟΝΙΣΜΟΣ §1: «γραμμή που κόβει κείμενο» (2ο κακό) είναι ΧΕΙΡΟΤΕΡΟ από
    # «ετικέτα ξεκολλημένη» (3ο κακό). Άρα δεν περιοριζόμαστε στις δύο ακριβώς
    # εφαπτόμενες θέσεις: δοκιμάζουμε και λίγο πιο έξω, ώστε μια ετικέτα που
    # κόβεται και από τις δύο πλευρές να μπορεί να ξεφύγει. Η βαθμολογία
    # (score_pos) κρίνει με τη σειρά της ιεραρχίας ποια θέση είναι η καλύτερη.
    if name in model.orig_perp:
        # Η ΑΡΧΙΚΗ κάθετη απόσταση διατηρείται ΑΚΡΙΒΩΣ - μόνο ολίσθηση κατά
        # μήκος επιτρέπεται. Έτσι η ετικέτα δεν μπορεί ΠΟΤΕ να βρεθεί πάνω
        # στη δική της γραμμή, όπως ακριβώς στο αρχείο-μήτρα.
        p_candidates = [bar_p + model.orig_perp[name]]
    else:
        p_candidates = [bar_p + ext_p - 0.01, bar_p - ext_p + 0.01]

    steps_t = [0.0] + [s*LBL_STEP for n in range(1, int(LBL_MAX_ALONG/LBL_STEP)+1) for s in (n, -n)]
    for p in p_candidates:
        if abs(p - cur_p) > LBL_MAX_PERP + ext_p:
            continue  # υπερβολικά μακριά από την τρέχουσα θέση - εκτός §3
        for dt in steps_t:
            t = cur_t+dt
            if not (lo-0.05 <= t <= hi+0.05):
                continue
            ncx = ux*t+px*p; ncy = uy*t+py*p
            ldx = ncx-cur_cx; ldy = ncy-cur_cy
            sc = score_pos(ldx, ldy)
            if sc < fallback[2]:
                fallback = (ldx, ldy, sc)
            if not candidate_ok_and_touch(ldx, ldy):
                continue
            score = abs(t-mid_t)
            if best is None or score < best[2]:
                best = (ldx, ldy, score)

    # §1/§8: αν καμία θέση δεν είναι απολύτως καθαρή, δεν μένουμε στην κομμένη -
    # παίρνουμε τη ΛΙΓΟΤΕΡΟ ΚΑΚΗ κατά την ιεραρχία, αρκεί να είναι καλύτερη από
    # την τωρινή. («τέλεια ή τίποτα» άφηνε ετικέτες κομμένες από τη ράβδο τους.)
    if best is None and fallback[2] < cur_score:
        best = (fallback[0], fallback[1], 0.0)

    if best is not None and (best[0] or best[1]):
        ldx, ldy = best[0], best[1]
        model.label_deltas[name][0] += ldx
        model.label_deltas[name][1] += ldy
        model.text_local[name] = [(x+ldx, y+ldy, w, h, rot) for x, y, w, h, rot in model.text_local[name]]


def _beam_geometry(model, beam_name):
    ox, oy = model.ins.get(beam_name, (0, 0))
    lines, _ = block_lines_local(model.blocks[beam_name])
    if not lines:
        return None
    world = [(x1+ox, y1+oy, x2+ox, y2+oy) for x1, y1, x2, y2 in lines]
    (ux, uy), (px, py) = _axis_and_perp(world)
    ts = [q0*ux+q1*uy for a, b, c, d in world for q0, q1 in ((a, b), (c, d))]
    ps = [q0*px+q1*py for a, b, c, d in world for q0, q1 in ((a, b), (c, d))]
    return (ux, uy), (px, py), (min(ts), max(ts)), (min(ps), max(ps))


def beam_span(model, beam_name):
    """(u,p), [tmin,tmax], [pmin,pmax] της δοκού από όλες τις γραμμές της."""
    lines = _world_lines(model.blocks, model.ins, beam_name)
    if not lines:
        return None
    (ux, uy), (px, py) = _axis_and_perp(lines)
    ts = []; ps = []
    for a, b, c, d in lines:
        ts += [a*ux+b*uy, c*ux+d*uy]
        ps += [a*px+b*py, c*px+d*py]
    return (ux, uy), (px, py), (min(ts), max(ts)), (min(ps), max(ps))


def process_beam_text(model, name):
    """ΚΑΝΟΝΙΣΜΟΣ §4 - ΜΗΧΑΝΙΣΜΟΣ ΕΠΑΝΑΦΟΡΑΣ: κείμενο δοκού που βρέθηκε ΕΚΤΟΣ
    της δικής του δοκού (το κέντρο του έξω από το πλάτος της) επαναφέρεται:
    99% ολίσθηση κατά μήκος + η ελάχιστη κάθετη μετατόπιση που το ξαναβάζει
    μέσα («σπάνια και δύσκολα» - εδώ είναι ακριβώς η σπάνια περίπτωση, αφού
    χωρίς κάθετη συνιστώσα δεν μπαίνει ποτέ). Η δική του δοκός δεν μετράει ως
    εμπόδιο. ΔΕΝ αγγίζει κείμενα που είναι ήδη εντός - μόνο επαναφορά."""
    m = re.match(BEAMTEXT_RE, name)
    if not m:
        return
    beam = f'{m.group(1)}_BEAM{m.group(2)}'
    if beam not in model.blocks:
        return
    boxes = model.text_local.get(name)
    if not boxes:
        return
    g = beam_span(model, beam)
    if g is None:
        return
    (ux, uy), (px, py), (tmin, tmax), (pmin, pmax) = g
    if pmax - pmin < 0.05 or pmax - pmin > 1.50:
        return  # εκφυλισμένη/παράλογη γεωμετρία - δεν ρισκάρουμε

    b0 = text_bbox(*(list(boxes[0])))
    for x, y, w, h, rot in boxes[1:]:
        bb = text_bbox(x, y, w, h, rot)
        b0 = (min(b0[0], bb[0]), min(b0[1], bb[1]), max(b0[2], bb[2]), max(b0[3], bb[3]))
    cx, cy = (b0[0]+b0[2])/2, (b0[1]+b0[3])/2
    pc = cx*px+cy*py; tc = cx*ux+cy*uy
    boxes_local0 = [(x, y, w, h, rot) for x, y, w, h, rot in boxes]

    # ΣΤΟΧΟΣ κάθετης θέσης (§4: 99% μόνο ολίσθηση ΚΑΤΑ ΜΗΚΟΣ):
    #  (α) αν ξέρουμε το πρωτότυπο, στόχος = η ΑΡΧΙΚΗ κάθετη θέση του κειμένου,
    #      δηλαδή μηδενισμός όποιας κάθετης μετατόπισης έβαλε η τακτοποίηση.
    #      Το FESPA τοποθετεί τα κείμενα δοκού με σταθερή, σκόπιμη απόκλιση
    #      (~-0.05) - «κεντράρισμα» θα ήταν λάθος στόχος.
    #  (β) αλλιώς, μόνο αν είναι εκτός λωρίδας δοκού, το φέρνουμε οριακά μέσα.
    dp0 = None
    if model.orig_ins is not None and name in model.orig_ins:
        d_now = model.ins.get(name, (0.0, 0.0))
        d_cur = (d_now[0]+model.deltas[name][0], d_now[1]+model.deltas[name][1])
        d_org = model.orig_ins[name]
        perp_shift = (d_cur[0]-d_org[0])*px + (d_cur[1]-d_org[1])*py
        if abs(perp_shift) > 0.05:
            dp0 = -perp_shift
    if dp0 is None:
        if pmin <= pc <= pmax:
            return  # εντός και χωρίς κάθετη παρέκκλιση - §4: δεν το αγγίζουμε
        target_p = min(max(pc, pmin+0.03), pmax-0.03)
        dp0 = target_p - pc

    steps_t = [0.0] + [s*LBL_STEP for n in range(1, int(LBL_MAX_ALONG/LBL_STEP)+1) for s in (n, -n)]
    for relaxed in (False, True):
        for dt in steps_t:
            nt = tc+dt
            if not (tmin+0.05 <= nt <= tmax-0.05):
                continue
            ldx = ux*dt+px*dp0; ldy = uy*dt+py*dp0
            excl = {name, beam} | model.marker_names
            placed = model.placed_boxes_excluding(excl)
            if relaxed:
                ok = P11.is_ok_relaxed(boxes_local0, ldx, ldy, model.obstacle_lines,
                                        model.hatch_polys, placed, exclude_line_names=excl,
                                        max_crossings=1)
            else:
                ok = P11.is_ok_full(boxes_local0, ldx, ldy, model.obstacle_lines,
                                     model.hatch_polys, placed, exclude_line_names=excl)
            if ok:
                model.deltas[name][0] += ldx
                model.deltas[name][1] += ldy
                model.text_local[name] = [(x+ldx, y+ldy, w, h, rot)
                                           for x, y, w, h, rot in model.text_local[name]]
                return (name, round(ldx, 2), round(ldy, 2))
    return None


def _slide_along_to_clear_markers(model, name):
    """ΚΑΝΟΝΙΣΜΟΣ §4: το κείμενο δοκού επιτρέπεται να ολισθαίνει ΜΟΝΟ κατά μήκος
    της δοκού του. Εδώ το γλιστράμε κατά μήκος (και μόνο) ώσπου να μην πέφτει σε
    δείκτη πλάκας - κρατώντας την ήδη επαναφερμένη κάθετη θέση του."""
    m = re.match(BEAMTEXT_RE, name)
    if not m:
        return None
    beam = f'{m.group(1)}_BEAM{m.group(2)}'
    if beam not in model.blocks:
        return None
    g = beam_span(model, beam)
    if g is None:
        return None
    (ux, uy), (px, py), (tmin, tmax), _ = g
    boxes0 = [(x, y, w, h, rot) for x, y, w, h, rot in model.text_local.get(name, [])]
    if not boxes0:
        return None
    cx = sum(text_bbox(*b)[0]+text_bbox(*b)[2] for b in boxes0)/(2*len(boxes0))
    cy = sum(text_bbox(*b)[1]+text_bbox(*b)[3] for b in boxes0)/(2*len(boxes0))
    tc = cx*ux+cy*uy

    def clear_of_markers(dx, dy):
        for x, y, w, h, rot in boxes0:
            b = text_bbox(x+dx, y+dy, w, h, rot)
            for mk_ in model.marker_names:
                for cb in model.circle_boxes.get(mk_, []):
                    if not (b[2] < cb[0] or cb[2] < b[0] or b[3] < cb[1] or cb[3] < b[1]):
                        return False
                for mx, my, mw, mh, mr in model.text_local.get(mk_, []):
                    mb = text_bbox(mx, my, mw, mh, mr)
                    if not (b[2] < mb[0] or mb[2] < b[0] or b[3] < mb[1] or mb[3] < b[1]):
                        return False
        return True

    excl = {name, beam}
    for k in range(1, int(LBL_MAX_ALONG/LBL_STEP)+1):
        for s in (1, -1):
            dt = k*LBL_STEP*s
            if not (tmin+0.05 <= tc+dt <= tmax-0.05):
                continue
            ldx, ldy = ux*dt, uy*dt
            if not clear_of_markers(ldx, ldy):
                continue
            placed = model.placed_boxes_excluding(excl)
            if P11.is_ok_full(boxes0, ldx, ldy, model.obstacle_lines, model.hatch_polys,
                               placed, exclude_line_names=excl):
                model.deltas[name][0] += ldx
                model.deltas[name][1] += ldy
                model.text_local[name] = [(x+ldx, y+ldy, w, h, rot)
                                           for x, y, w, h, rot in model.text_local[name]]
                return round(dt, 2)
    return None


def _gap(a, b):
    dx = max(0.0, max(a[0]-b[2], b[0]-a[2]))
    dy = max(0.0, max(a[1]-b[3], b[1]-a[3]))
    return math.hypot(dx, dy)


def process_column_texts(model):
    """ΚΑΝΟΝΙΣΜΟΣ §6: το κείμενο κολώνας πρέπει να είναι ΠΑΝΤΑ πιο κοντά στη
    ΔΙΚΗ του κολώνα παρά σε οποιαδήποτε άλλη, με την απόσταση μετρημένη από το
    ΠΛΗΣΙΕΣΤΕΡΟ ΣΗΜΕΙΟ του κουτιού προς το πλησιέστερο σημείο της κολώνας (όχι
    από κέντρα - ένα φαρδύ κείμενο έχει πάντα το κέντρο του μακριά).
    Αγγίζει ΜΟΝΟ όσα παραβιάζουν την ιδιοκτησία· τα υπόλοιπα μένουν ως έχουν.
    Οι δείκτες πλάκας ΔΕΝ μετρούν ως εμπόδιο εδώ: κινούνται ελεύθερα εντός της
    πλάκας τους (§5) και υποχωρούν στο επόμενο πέρασμα."""
    moved = []
    cols = {}
    for c in model.blocks:
        if re.match(r'FL-?\d+_COLUMN\d+$', c):
            wl = _world_lines(model.blocks, model.ins, c)
            if not wl:
                continue
            xs = [p for s_ in wl for p in (s_[0], s_[2])]
            ys = [p for s_ in wl for p in (s_[1], s_[3])]
            cols[c] = (min(xs), min(ys), max(xs), max(ys))
    names = sorted([n for n in model.blocks if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    for name in names:
        m = re.match(r'(FL-?\d+)_COLUMN_TEXT(\d+)$', name)
        own = f'{m.group(1)}_COLUMN{m.group(2)}'
        if own not in cols:
            continue
        boxes = model.text_local.get(name)
        if not boxes:
            continue
        tb = None
        for x, y, w, h, rot in boxes:
            b = text_bbox(x, y, w, h, rot)
            tb = b if tb is None else (min(tb[0], b[0]), min(tb[1], b[1]),
                                        max(tb[2], b[2]), max(tb[3], b[3]))
        d_own = _gap(tb, cols[own])
        d_for = min((_gap(tb, b) for c, b in cols.items() if c != own), default=1e9)
        if d_own <= d_for:
            continue   # ιδιοκτησία εντάξει - δεν το αγγίζουμε
        # Ο κανόνας των ΠΕΡΙΜΕΤΡΙΚΩΝ ΥΠΕΡΙΣΧΥΕΙ της ιδιοκτησίας: αν η κολώνα
        # είναι περιμετρική και το κείμενό της κάθεται ήδη ΕΞΩ, δεν το τραβάμε
        # πίσω μέσα - ούτε καν για να βελτιώσουμε την ιδιοκτησία.
        is_perim = bool(model.outward.get(own))
        was_outside = model.is_outside(tb)
        if is_perim and was_outside:
            continue

        ocx = (cols[own][0]+cols[own][2])/2
        ocy = (cols[own][1]+cols[own][3])/2
        tcx, tcy = (tb[0]+tb[2])/2, (tb[1]+tb[3])/2
        excl = {name, own} | model.marker_names
        boxes0 = [(x, y, w, h, rot) for x, y, w, h, rot in boxes]
        # 3ο επίπεδο χαλάρωσης: όταν το κείμενο είναι φαρδύ (π.χ. 2,14x0,57)
        # δεν υπάρχει ΚΑΜΙΑ τελείως καθαρή θέση γύρω από την κολώνα του. Τότε
        # γίνεται δεκτή επικάλυψη ΜΟΝΟ με ετικέτα οπλισμού (BEAMBAR/SLABBAR) -
        # ΠΟΤΕ με όνομα δοκού, κολώνας ή πλάκας. Καλύτερα το όνομα της κολώνας
        # στη θέση του, με τίμημα μια ετικέτα οπλισμού, παρά παραπλανητικό.
        rebar_names = set(n_ for n_ in model.text_local if re.search(r'(BEAMBAR|SLABBAR)\d+$', n_))
        best = None
        for level in (0, 1):
            excl_l = excl if level == 0 else (excl | rebar_names)
            placed = model.placed_boxes_excluding(excl_l)
            for rad in [0.05*k for k in range(1, 25)]:      # έως 1,20μ
                for ang in range(0, 360, 10):
                    a = math.radians(ang)
                    nx = ocx + rad*math.cos(a); ny = ocy + rad*math.sin(a)
                    ldx, ldy = nx-tcx, ny-tcy
                    nb = (tb[0]+ldx, tb[1]+ldy, tb[2]+ldx, tb[3]+ldy)
                    nd_own = _gap(nb, cols[own])
                    nd_for = min((_gap(nb, b) for c, b in cols.items() if c != own), default=1e9)
                    if nd_own >= nd_for:
                        continue     # §6: πρέπει να είναι πιο κοντά στη ΔΙΚΗ του
                    if is_perim and model.is_outside(nb) is False:
                        continue     # περιμετρική: η θέση πρέπει να μένει ΕΞΩ
                    if not P11.is_ok_full(boxes0, ldx, ldy, model.obstacle_lines,
                                           model.hatch_polys, placed, exclude_line_names=excl_l):
                        continue
                    if best is None or nd_own < best[2]:
                        best = (ldx, ldy, nd_own)
                if best is not None:
                    break
            if best is not None:
                break
        if best is not None:
            ldx, ldy, nd = best
            model.deltas[name][0] += ldx
            model.deltas[name][1] += ldy
            model.text_local[name] = [(x+ldx, y+ldy, w, h, rot)
                                       for x, y, w, h, rot in model.text_local[name]]
            moved.append((name, round(ldx, 2), round(ldy, 2), round(d_own, 2), round(nd, 2)))
    return moved


def process_slab_markers(model):
    """ΚΑΝΟΝΙΣΜΟΣ §5: ο δείκτης πλάκας (κυκλάκι Πx + h=) μετακινείται ΟΛΟΚΛΗΡΟΣ
    - κυκλάκι, γραμμές και κείμενα μαζί. Το περίγραμμα slab_poly ΔΕΝ μετακινείται
    ποτέ (είναι το όριο της πλάκας). Σειρά προτίμησης: μικρή μετακίνηση έως 0,30
    εντός της πλάκας του, μετά ελεύθερα εντός της πλάκας του.
    Τρέχει ΤΕΛΕΥΤΑΙΟ, αφού κατακάτσουν οπλισμοί και κείμενα - §8: ό,τι κρίνεται
    τελευταίο πρέπει να βλέπει το τελικό τοπίο."""
    moved = []
    names = sorted([n for n in model.blocks if re.match(r'FL-?\d+_SLAB\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    for name in names:
        boxes = model.text_local.get(name)
        cbs = model.circle_boxes.get(name)
        if not boxes or not cbs:
            continue
        region = model.slab_regions.get(name)
        excl = {name}
        boxes_local0 = [(x, y, w, h, rot) for x, y, w, h, rot in boxes]
        if model.marker_ok(name, 0.0, 0.0):
            continue  # καθαρός - δεν τον αγγίζουμε
        if region is None:
            continue  # άγνωστα όρια πλάκας - §5: δεν ρισκάρουμε μετακίνηση

        def within_slab(dx, dy):
            """ΙΔΙΟ κριτήριο με τον έλεγχο ΣΤ του audit_output.py: ΟΛΑ τα κομμάτια
            του δείκτη (ένωση κειμένων ΚΑΙ κυκλάκι) μένουν εντός της πλάκας -
            όχι μόνο το κέντρο, αλλιώς ο δείκτης «βγαίνει» με το μισό του έξω."""
            pieces = []
            xs = []; ys = []
            ox_, oy_ = model.ins.get(name, (0.0, 0.0))
            mdx = model.marker_deltas[name][0] + dx
            mdy = model.marker_deltas[name][1] + dy
            for x, y, w, h, rot in slab_marker_boxes(model.blocks[name]):
                b = text_bbox(x+ox_+mdx, y+oy_+mdy, w, h, rot)
                xs += [b[0], b[2]]; ys += [b[1], b[3]]
            if xs:
                pieces.append((min(xs), min(ys), max(xs), max(ys)))
            for cb in model.circle_boxes.get(name, []):
                pieces.append((cb[0]+dx, cb[1]+dy, cb[2]+dx, cb[3]+dy))
            x_ok = (region[2]-region[0]) >= 0.05
            y_ok = (region[3]-region[1]) >= 0.05
            for pb in pieces:
                if x_ok and (pb[0] < region[0]-0.05 or pb[2] > region[2]+0.05):
                    return False
                if y_ok and (pb[1] < region[1]-0.05 or pb[3] > region[3]+0.05):
                    return False
            return True

        if not within_slab(0.0, 0.0):
            continue  # ήδη εκτός από την τακτοποίηση - δεν το χειροτερεύουμε εμείς

        # §5 προτεραιότητες: 1) μικρή μετακίνηση <=0.30, 2) ΕΛΕΥΘΕΡΑ εντός της
        # πλάκας του «όσο χρειαστεί». Το εύρος βγαίνει από το ΜΕΓΕΘΟΣ της πλάκας,
        # όχι από αυθαίρετο σταθερό όριο: η Π12 έχει πλάτος 2,80μ και το κυκλάκι
        # χωράει άνετα πιο αριστερά.
        _rmax = math.hypot(region[2]-region[0], region[3]-region[1])
        best = None
        for rad in [0.05*k for k in range(1, int(_rmax/0.05)+2)]:
            for ang in range(0, 360, 10):
                a = math.radians(ang)
                dx, dy = rad*math.cos(a), rad*math.sin(a)
                if not within_slab(dx, dy):
                    continue   # §5: ΟΛΟΚΛΗΡΟΣ ο δείκτης εντός της πλάκας του
                if model.marker_ok(name, dx, dy):
                    best = (dx, dy, rad)
                    break
            if best:
                break
        if best:
            dx, dy, _ = best
            model.marker_deltas[name][0] += dx
            model.marker_deltas[name][1] += dy
            model.text_local[name] = [(x+dx, y+dy, w, h, rot) for x, y, w, h, rot in model.text_local[name]]
            model.circle_boxes[name] = [(b[0]+dx, b[1]+dy, b[2]+dx, b[3]+dy) for b in model.circle_boxes[name]]
            moved.append((name, round(dx, 2), round(dy, 2)))
    return moved


def _label_geom(model, name):
    """(u,p,lo,hi, bbox ετικέτας, boxes) της ετικέτας ενός οπλισμού."""
    lines = model.bar_lines.get(name)
    boxes = model.text_local.get(name)
    if not lines or not boxes:
        return None
    (ux, uy), (px, py) = _axis_and_perp(lines)
    bb = None
    for x, y, w, h, rot in boxes:
        b = text_bbox(x, y, w, h, rot)
        bb = b if bb is None else (min(bb[0], b[0]), min(bb[1], b[1]),
                                    max(bb[2], b[2]), max(bb[3], b[3]))
    lo = min(min(s_[0]*ux+s_[1]*uy, s_[2]*ux+s_[3]*uy) for s_ in lines)
    hi = max(max(s_[0]*ux+s_[1]*uy, s_[2]*ux+s_[3]*uy) for s_ in lines)
    return (ux, uy), (px, py), lo, hi, bb, boxes


def _label_badness(model, name, ldx, ldy):
    """Βαθμολογία θέσης ετικέτας οπλισμού κατά ΚΑΝΟΝΙΣΜΟ §1 (μικρότερο=καλύτερο).
    Ίδια κριτήρια με τον ελεγκτή, ώστε ό,τι εγκρίνεται εδώ να περνάει κι εκεί."""
    g = _label_geom(model, name)
    if g is None:
        return None
    (ux, uy), (px, py), lo, hi, bb, boxes = g
    cb = (bb[0]+ldx, bb[1]+ldy, bb[2]+ldx, bb[3]+ldy)
    tol = 0.03
    inner = (cb[0]+tol, cb[1]+tol, cb[2]-tol, cb[3]-tol)
    lines = model.bar_lines[name]
    own_cut = 0
    foreign_cut = 0
    if inner[0] < inner[2] and inner[1] < inner[3]:
        if any(seg_intersects_bbox(s_[:4], inner) for s_ in lines):
            own_cut = 1
        for m_, segs in model.bar_lines.items():
            if m_ == name:
                continue
            if any(seg_intersects_bbox(s_[:4], inner) for s_ in segs):
                foreign_cut = 1
                break
    GAP = 0.03
    near = (cb[0]-GAP, cb[1]-GAP, cb[2]+GAP, cb[3]+GAP)
    overl = 0
    for other, obs in model.text_local.items():
        if other == name:
            continue
        for x_, y_, w_, h_, r_ in obs:
            ob = text_bbox(x_, y_, w_, h_, r_)
            if not (near[2] < ob[0] or ob[2] < near[0] or near[3] < ob[1] or ob[3] < near[1]):
                overl += 1
    for mk_ in model.marker_names:
        for cbx in model.circle_boxes.get(mk_, []):
            if not (near[2] < cbx[0] or cbx[2] < near[0] or near[3] < cbx[1] or cbx[3] < near[1]):
                overl += 1
    bshift = [(x_+ldx, y_+ldy, w_, h_, r_) for x_, y_, w_, h_, r_ in boxes]
    pcut = 1 if P11.parallel_cut_full(bshift, 0.0, 0.0, model.obstacle_lines,
                                       exclude_line_names={name}) else 0
    struct_cut = 0
    if inner[0] < inner[2] and inner[1] < inner[3]:
        if any(seg_intersects_bbox(s_[:4], inner) for s_ in model.struct_only):
            struct_cut = 1
    on_hatch = 1 if model.box_on_hatch_bb(cb) else 0
    t_ = ((cb[0]+cb[2])/2)*ux + ((cb[1]+cb[3])/2)*uy
    mid = (lo+hi)/2
    return (own_cut, overl, foreign_cut, struct_cut, pcut, on_hatch, abs(t_-mid))


def slide_label_only(model, name):
    """ΦΑΣΗ 1/2 του τελικού ελέγχου: ολισθαίνει ΜΟΝΟ την ετικέτα του οπλισμού
    κατά μήκος της ράβδου του. Η ράβδος ΔΕΝ κινείται, η κάθετη απόσταση
    ετικέτας-ράβδου ΔΕΝ αλλάζει (μένει η αρχική του αρχείου-μήτρα).
    Δέχεται νέα θέση μόνο αν είναι ΓΝΗΣΙΩΣ καλύτερη κατά την ιεραρχία §1."""
    g = _label_geom(model, name)
    if g is None:
        return None
    (ux, uy), (px, py), lo, hi, bb, boxes = g
    cur = _label_badness(model, name, 0.0, 0.0)
    if cur is None or cur[:6] == (0, 0, 0, 0, 0, 0):
        return None      # ήδη καθαρή - δεν την αγγίζουμε
    cx, cy = (bb[0]+bb[2])/2, (bb[1]+bb[3])/2
    ct = cx*ux+cy*uy
    best = None
    for k in range(1, int(LBL_MAX_ALONG/LBL_STEP)+1):
        for sgn in (1, -1):
            dt = k*LBL_STEP*sgn
            if not (lo-0.05 <= ct+dt <= hi+0.05):
                continue
            ldx, ldy = ux*dt, uy*dt          # ΜΟΝΟ κατά μήκος
            sc = _label_badness(model, name, ldx, ldy)
            if sc is None:
                continue
            if sc < cur and (best is None or sc < best[2]):
                best = (ldx, ldy, sc)
    if best is None:
        return None
    ldx, ldy, _ = best
    model.label_deltas[name][0] += ldx
    model.label_deltas[name][1] += ldy
    model.text_local[name] = [(x+ldx, y+ldy, w, h, rot)
                               for x, y, w, h, rot in model.text_local[name]]
    return (name, round(ldx, 2), round(ldy, 2))


def final_sweep(model, rounds=2):
    """ΤΕΛΙΚΟΣ ΟΛΙΚΟΣ ΕΛΕΓΧΟΣ, με τη σειρά που όρισε ο μηχανικός:
      1) ετικέτες οπλισμού ΔΟΚΩΝ  - ολίσθηση μόνο της ετικέτας, + επανέλεγχος
      2) ετικέτες οπλισμού ΠΛΑΚΩΝ - ολίσθηση μόνο, με επανέλεγχο ώστε η νέα
         θέση της μιας να μην πέφτει στη νέα θέση της άλλης
      3) κείμενο δοκού - ΜΟΝΟ αν ενοχλεί ΚΑΙ υπάρχει γνήσια καθαρότερη θέση
    Τίποτε άλλο δεν αγγίζεται: ούτε κολώνες, ούτε δοκοί, ούτε ράβδοι."""
    log = {'beambar': [], 'slabbar': [], 'beamtext': []}
    bb_names = sorted([n for n in model.blocks if re.match(r'FL-?\d+_BEAMBAR\d+$', n)],
                       key=lambda n: int(re.search(r'\d+$', n).group()))
    sb_names = sorted([n for n in model.blocks if re.match(SLABBAR_RE, n)],
                       key=lambda n: int(re.search(r'\d+$', n).group()))
    for r in range(rounds):
        moved = 0
        for nm in bb_names:                     # ΦΑΣΗ 1
            res = slide_label_only(model, nm)
            if res:
                log['beambar'].append(res); moved += 1
        for nm in sb_names:                     # ΦΑΣΗ 2
            res = slide_label_only(model, nm)
            if res:
                log['slabbar'].append(res); moved += 1
        if moved == 0:
            break
    # ΦΑΣΗ 3 - κείμενα δοκών, μόνο όσα ενοχλούν και μόνο αν υπάρχει
    # γνήσια καθαρότερη θέση κατά μήκος, εντός των ορίων της δοκού τους
    for nm in sorted([n for n in model.blocks if re.match(BEAMTEXT_RE, n)],
                      key=lambda n: int(re.search(r'\d+$', n).group())):
        res = _slide_beamtext_if_better(model, nm)
        if res:
            log['beamtext'].append(res)
    return log


def _slide_beamtext_if_better(model, name):
    m = re.match(BEAMTEXT_RE, name)
    if not m:
        return None
    beam = f'{m.group(1)}_BEAM{m.group(2)}'
    if beam not in model.blocks:
        return None
    g = beam_span(model, beam)
    boxes = model.text_local.get(name)
    if g is None or not boxes:
        return None
    (ux, uy), (px, py), (tmin, tmax), (pmin, pmax) = g
    boxes0 = [(x, y, w, h, rot) for x, y, w, h, rot in boxes]
    excl = {name, beam}

    def bad(ldx, ldy):
        placed = model.placed_boxes_excluding(excl)
        bshift = [(x_+ldx, y_+ldy, w_, h_, r_) for x_, y_, w_, h_, r_ in boxes0]
        pc = 1 if P11.parallel_cut_full(bshift, 0.0, 0.0, model.obstacle_lines,
                                         exclude_line_names=excl) else 0
        # §7: το hatch είναι εμπόδιο για ΟΛΑ τα κείμενα - «καθαρή θέση» σημαίνει
        # χωρίς γραμμές, χωρίς κείμενο ΚΑΙ χωρίς hatch
        for x_, y_, w_, h_, r_ in bshift:
            if model.box_on_hatch_bb(text_bbox(x_, y_, w_, h_, r_)):
                pc += 1
                break
        ov = 0
        for x_, y_, w_, h_, r_ in bshift:
            b_ = text_bbox(x_, y_, w_, h_, r_)
            for pb in placed:
                if not (b_[2] < pb[0] or pb[2] < b_[0] or b_[3] < pb[1] or pb[3] < b_[1]):
                    ov += 1
        return (ov, pc)

    cur = bad(0.0, 0.0)
    if cur == (0, 0):
        return None       # δεν ενοχλεί - μένει ως έχει
    bb = None
    for x, y, w, h, rot in boxes0:
        b = text_bbox(x, y, w, h, rot)
        bb = b if bb is None else (min(bb[0], b[0]), min(bb[1], b[1]),
                                    max(bb[2], b[2]), max(bb[3], b[3]))
    ct = ((bb[0]+bb[2])/2)*ux + ((bb[1]+bb[3])/2)*uy
    best = None
    for k in range(1, int(LBL_MAX_ALONG/LBL_STEP)+1):
        for sgn in (1, -1):
            dt = k*LBL_STEP*sgn
            if not (tmin+0.05 <= ct+dt <= tmax-0.05):
                continue       # ΠΑΝΤΑ εντός των ορίων της δοκού του
            ldx, ldy = ux*dt, uy*dt         # ΜΟΝΟ ολίσθηση κατά μήκος
            sc = bad(ldx, ldy)
            if sc < cur and (best is None or sc < best[2]):
                best = (ldx, ldy, sc)
    if best is None:
        return None            # καμία καλύτερη θέση - μένει ως έχει
    ldx, ldy, _ = best
    model.deltas[name][0] += ldx
    model.deltas[name][1] += ldy
    model.text_local[name] = [(x+ldx, y+ldy, w, h, rot)
                               for x, y, w, h, rot in model.text_local[name]]
    return (name, round(ldx, 2), round(ldy, 2))


def slide_to_clean_only(model, name):
    """ΤΕΛΙΚΟ ΒΗΜΑ: ολισθαίνει την ετικέτα οπλισμού κατά μήκος της ράβδου της
    ΜΟΝΟ αν υπάρχει ΤΕΛΕΙΩΣ ΚΑΘΑΡΗ νέα θέση - καμία γραμμή, κανένα κείμενο.
    Διαφέρει από τη slide_label_only: εκείνη δέχεται και «λιγότερο κακή» θέση,
    αυτή μόνο απολύτως καθαρή. Αν δεν υπάρχει, η ετικέτα μένει ως έχει.
    Η ράβδος δεν κινείται και η κάθετη απόσταση δεν αλλάζει (§3/§Ι)."""
    g = _label_geom(model, name)
    if g is None:
        return None
    (ux, uy), (px, py), lo, hi, bb, boxes = g
    cur = _label_badness(model, name, 0.0, 0.0)
    if cur is None or cur[:6] == (0, 0, 0, 0, 0, 0):
        return None          # ήδη καθαρή
    cx, cy = (bb[0]+bb[2])/2, (bb[1]+bb[3])/2
    ct = cx*ux+cy*uy
    span = max(LBL_MAX_ALONG, (hi-lo) + 0.20)     # ΟΛΟ το μήκος της ράβδου
    best = None
    for k in range(1, int(span/LBL_STEP)+1):
        for sgn in (1, -1):
            dt = k*LBL_STEP*sgn
            if not (lo-0.05 <= ct+dt <= hi+0.05):
                continue
            ldx, ldy = ux*dt, uy*dt        # ΜΟΝΟ κατά μήκος
            sc = _label_badness(model, name, ldx, ldy)
            if sc is None:
                continue
            if sc[:6] == (0, 0, 0, 0, 0, 0):   # ΤΕΛΕΙΩΣ καθαρή
                if best is None or sc[6] < best[2]:
                    best = (ldx, ldy, sc[6])
        if best is not None:
            break              # η κοντινότερη καθαρή θέση κερδίζει
    if best is None:
        return None
    ldx, ldy, _ = best
    model.label_deltas[name][0] += ldx
    model.label_deltas[name][1] += ldy
    model.text_local[name] = [(x+ldx, y+ldy, w, h, rot)
                               for x, y, w, h, rot in model.text_local[name]]
    return (name, round(ldx, 2), round(ldy, 2))


def final_clean_slide(model):
    """ΤΕΛΙΚΟ ΒΗΜΑ με τη σειρά που όρισε ο μηχανικός:
    ετικέτες ΔΟΚΩΝ -> ετικέτες ΠΛΑΚΩΝ -> ξανά ετικέτες ΔΟΚΩΝ.
    Ολίσθηση μόνο, και μόνο προς ΤΕΛΕΙΩΣ καθαρή θέση."""
    bb_names = sorted([n for n in model.blocks if re.match(r'FL-?\d+_BEAMBAR\d+$', n)],
                       key=lambda n: int(re.search(r'\d+$', n).group()))
    sb_names = sorted([n for n in model.blocks if re.match(SLABBAR_RE, n)],
                       key=lambda n: int(re.search(r'\d+$', n).group()))
    out = {'δοκών_α': [], 'πλακών': [], 'δοκών_β': []}
    for nm in bb_names:
        r = slide_to_clean_only(model, nm)
        if r: out['δοκών_α'].append(r)
    for nm in sb_names:
        r = slide_to_clean_only(model, nm)
        if r: out['πλακών'].append(r)
    for nm in bb_names:
        r = slide_to_clean_only(model, nm)
        if r: out['δοκών_β'].append(r)
    return out


def final_slab_label_clean(model, rounds=2):
    """ΤΕΛΕΥΤΑΙΟ ΒΗΜΑ - ΜΟΝΟ ετικέτες οπλισμού ΠΛΑΚΩΝ (SLABBAR): όσες πέφτουν σε
    γραμμές, κείμενο ή hatch ολισθαίνουν κατά μήκος της ράβδου τους σε ΚΑΘΑΡΗ
    θέση (χωρίς γραμμές, χωρίς κείμενο, χωρίς hatch). Τρέχει ΤΕΛΕΥΤΑΙΟ, αφού
    έχουν κατακάτσει τα πάντα, ώστε να δει το τελικό τοπίο - και επαναλαμβάνεται
    ώστε η νέα θέση της μιας να ελέγχεται απέναντι στη νέα θέση της άλλης.
    Δεν αγγίζει τίποτε άλλο: ούτε ράβδους, ούτε δοκούς, ούτε κολώνες."""
    names = sorted([n for n in model.blocks if re.match(SLABBAR_RE, n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    moved = []
    for r in range(rounds):
        this = []
        for nm in names:
            res = slide_to_clean_only(model, nm)
            if res:
                this.append(res)
        moved.extend(this)
        if not this:
            break
    return moved


# Τα κείμενα που μετράνε γι' αυτό το βήμα: κείμενο δοκού, κείμενο κολώνας,
# κείμενο (ετικέτα) οπλισμού ΚΑΙ κείμενο δείκτη πλάκας (SLABn).
_RELOC_TEXT_RE = r'(BEAM_TEXT|COLUMN_TEXT|BEAMBAR|SLABBAR|SLAB)\d+$'


def _lines_cut_any_text_strict(model, lines, exclude):
    """True αν ΟΠΟΙΑΔΗΠΟΤΕ από τις γραμμές περνά μέσα από τον πυρήνα κειμένου
    δοκού, κειμένου κολώνας ή ετικέτας οπλισμού. Σε οποιαδήποτε γωνία: μια
    γραμμή που περνά κάθετα κόβει τα γράμματα το ίδιο."""
    tol = 0.03
    segs = [(s_[0], s_[1], s_[2], s_[3], '__CAND__') for s_ in lines]
    for other, boxes in model.text_local.items():
        if other in exclude or not re.search(_RELOC_TEXT_RE, other):
            continue
        # (α) γραμμή μέσα στον πυρήνα, σε ΟΠΟΙΑΔΗΠΟΤΕ γωνία
        for x, y, w, h, rot in boxes:
            b = text_bbox(x, y, w, h, rot)
            inner = (b[0]+tol, b[1]+tol, b[2]-tol, b[3]-tol)
            if inner[0] >= inner[2] or inner[1] >= inner[3]:
                continue
            if any(seg_intersects_bbox(s_[:4], inner) for s_ in lines):
                return True
        # (β) ΚΑΙ το κριτήριο «παράλληλης τομής» του ελεγκτή (cos>0.94): αλλιώς
        # η σκανδάλη λέει «καθαρό» για κάτι που ο έλεγχος καταγγέλλει - ένας
        # κριτής, όχι δύο που διαφωνούν.
        if P11.parallel_cut_full(boxes, 0.0, 0.0, segs):
            return True
    return False


def _lines_cut_markers(model, lines):
    """True αν οι γραμμές τέμνουν κείμενο ή κυκλάκι δείκτη πλάκας."""
    tol = 0.03
    for mk_ in model.marker_names:
        for x, y, w, h, rot in model.text_local.get(mk_, []):
            b = text_bbox(x, y, w, h, rot)
            inner = (b[0]+tol, b[1]+tol, b[2]-tol, b[3]-tol)
            if inner[0] < inner[2] and inner[1] < inner[3]:
                if any(seg_intersects_bbox(s_[:4], inner) for s_ in lines):
                    return True
        for cb in model.circle_boxes.get(mk_, []):
            if any(seg_intersects_bbox(s_[:4], cb) for s_ in lines):
                return True
    return False


def _label_hits_reloc_text(model, name, ldx, ldy):
    """True αν η ετικέτα του οπλισμού (στη θέση +ldx,+ldy) επικαλύπτει κείμενο
    δοκού, κειμένου κολώνας ή ετικέτα άλλου οπλισμού."""
    boxes = model.text_local.get(name)
    if not boxes:
        return False
    for x, y, w, h, rot in boxes:
        b = text_bbox(x+ldx, y+ldy, w, h, rot)
        for other, obs in model.text_local.items():
            if other == name or not re.search(_RELOC_TEXT_RE, other):
                continue
            for ox_, oy_, ow_, oh_, orr_ in obs:
                ob = text_bbox(ox_, oy_, ow_, oh_, orr_)
                if not (b[2] < ob[0] or ob[2] < b[0] or b[3] < ob[1] or ob[3] < b[1]):
                    return True
    return False


def _find_region(model, lines):
    """Η πλάκα στην οποία ανήκει ένας οπλισμός. Δοκιμάζει, με σειρά:
      1) το σημείο αναφοράς (άκρο αγκίστρου για προβόλους, αλλιώς κέντρο)
      2) το γεωμετρικό κέντρο της ράβδου
      3) την πλησιέστερη πλάκα, αν απέχει λιγότερο από 0,50μ
    Χωρίς τα εφεδρικά βήματα, ένας οπλισμός του οποίου το άκρο πέφτει πάνω στη
    δοκό «δεν έχει πλάκα» και μένει ακίνητος για πάντα (τεκμηριωμένο: SLABBAR5,
    SLABBAR8 στο DAMAR07). Το όριο της πλάκας παραμένει απαράβατο (§0) - απλώς
    την εντοπίζουμε σωστά."""
    if not lines:
        return None, None
    cands = []
    ref = bar_reference_point(lines)
    if ref:
        cands.append(ref)
    pts = [(s_[0], s_[1]) for s_ in lines] + [(s_[2], s_[3]) for s_ in lines]
    cands.append((sum(p[0] for p in pts)/len(pts), sum(p[1] for p in pts)/len(pts)))
    for cx, cy in cands:
        for rname, bb in model.slab_regions.items():
            if bb[0] <= cx <= bb[2] and bb[1] <= cy <= bb[3]:
                return bb, (cx, cy)
    best = None
    for cx, cy in cands:
        for rname, bb in model.slab_regions.items():
            dx = max(0.0, bb[0]-cx, cx-bb[2])
            dy = max(0.0, bb[1]-cy, cy-bb[3])
            d = math.hypot(dx, dy)
            if d <= 0.50 and (best is None or d < best[0]):
                best = (d, bb, (cx, cy))
    if best:
        return best[1], best[2]
    return None, None


def final_rebar_relocate(model, rounds=2):
    """ΤΕΛΙΚΟ ΒΗΜΑ: όταν η ΓΡΑΜΜΗ ή η ΕΤΙΚΕΤΑ ενός οπλισμού ΠΛΑΚΑΣ τέμνει
    κείμενο δοκού, κείμενο οπλισμού ή κείμενο κολώνας,
    δοκιμάζεται μετατόπιση του οπλισμού κατά τους κανόνες του
    (§3: κάθετα στον άξονά του, σταθερό βήμα, ΠΑΝΤΑ εντός των ορίων της πλάκας
    του - §0). Η ετικέτα ακολουθεί με σταθερή την αρχική κάθετη απόσταση (§Ι).

    Η νέα θέση γίνεται δεκτή ΜΟΝΟ αν ΚΑΙ ΤΑ ΔΥΟ ισχύουν:
      α) καμία γραμμή του οπλισμού δεν τέμνει πλέον κείμενο, και
      β) η ετικέτα του δεν τέμνεται από γραμμή, κείμενο ή hatch.
    Αλλιώς ο οπλισμός μένει ακριβώς εκεί που είναι - καμία μερική λύση."""
    moved = []
    names = sorted([n for n in model.blocks if re.match(SLABBAR_RE, n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    for r in range(rounds):
        this = []
        for name in names:
            lines0 = model.bar_lines.get(name)
            g = _label_geom(model, name)
            if not lines0 or g is None:
                continue
            cur_lines_bad = _lines_cut_any_text_strict(model, lines0, {name})
            cur_lbl_bad = _label_hits_reloc_text(model, name, 0.0, 0.0)
            cur_lbl = _label_badness(model, name, 0.0, 0.0)
            if cur_lbl is None:
                continue
            # Τρίτο σκέλος σκανδάλης: η ΕΤΙΚΕΤΑ του τέμνεται από γραμμή, κείμενο
            # ή hatch. Χωρίς αυτό, το κριτήριο αποδοχής ζητούσε καθαρή ετικέτα
            # αλλά η περίπτωση δεν εξεταζόταν ποτέ (τεκμηριωμένο: η ετικέτα του
            # SLABBAR19 κόβεται από γραμμή του δείκτη SLAB14).
            cur_lbl_cut = (cur_lbl[:6] != (0, 0, 0, 0, 0, 0))
            if not cur_lines_bad and not cur_lbl_bad and not cur_lbl_cut:
                continue          # καθαρός - δεν τον αγγίζουμε

            region, ref = _find_region(model, lines0)
            if region is None:
                continue          # §0: άγνωστα όρια πλάκας - δεν ρισκάρουμε

            (ux, uy), (px, py) = _axis_and_perp(lines0)
            found = None
            for k in range(1, MAX_STEPS+1):
                for sgn in (1, -1):
                    d = k*STEP*sgn
                    rx, ry = ref[0]+px*d, ref[1]+py*d
                    # §0 ΑΠΑΡΑΒΑΤΟ: εντός των ορίων της πλάκας του. Αν το σημείο
                    # αναφοράς είναι ΗΔΗ έξω (π.χ. άκρο προβόλου πάνω στη δοκό),
                    # επιτρέπεται μόνο κίνηση που το φέρνει ΠΙΟ ΚΟΝΤΑ/ΜΕΣΑ - ποτέ
                    # πιο έξω. Χωρίς αυτό ο οπλισμός έμενε παγιδευμένος: και οι 50
                    # υποψήφιες θέσεις έβγαιναν «εκτός» (SLABBAR5, SLABBAR8).
                    def _dout(x_, y_):
                        return math.hypot(max(0.0, region[0]-x_, x_-region[2]),
                                           max(0.0, region[1]-y_, y_-region[3]))
                    d0 = _dout(ref[0], ref[1])
                    dn = _dout(rx, ry)
                    if d0 <= 1e-9:
                        if dn > 1e-9:
                            continue                # ήταν μέσα - μένει μέσα
                    elif dn > d0 - 1e-9:
                        continue                    # ήταν έξω - πρέπει να πλησιάζει
                    cand = _translate_lines(lines0, px*d, py*d)
                    if _lines_cut_any_text_strict(model, cand, {name}):
                        continue                    # (α) οι γραμμές πρέπει να καθαρίσουν
                    saved = model.bar_lines[name]
                    model.bar_lines[name] = cand
                    sc = _label_badness(model, name, px*d, py*d)
                    model.bar_lines[name] = saved
                    if sc is None or sc[:6] != (0, 0, 0, 0, 0, 0):
                        continue                    # (β) και η ετικέτα πρέπει να καθαρίσει
                    found = (d, cand, px*d, py*d)
                    break
                if found:
                    break
            if not found:
                continue
            d, cand, dx, dy = found
            model.deltas[name][0] += dx
            model.deltas[name][1] += dy
            model.bar_lines[name] = cand
            model.text_local[name] = [(x+dx, y+dy, w, h, rot)
                                       for x, y, w, h, rot in model.text_local[name]]
            model.refresh_obstacle_lines_for(name, cand)
            this.append((name, round(dx, 2), round(dy, 2)))
        moved.extend(this)
        if not this:
            break
    return moved


def final_beam_label_clean(model, rounds=2):
    """ΤΕΛΕΥΤΑΙΟ ΒΗΜΑ - ετικέτες οπλισμού ΔΟΚΩΝ (BEAMBAR): αν η ετικέτα τέμνεται
    από γραμμές ή από άλλες ετικέτες, δοκιμάζεται ΟΛΙΣΘΗΣΗ ΜΟΝΟ της ετικέτας
    κατά μήκος της ράβδου της (§3/§Ι: η ράβδος δεν κινείται, η κάθετη απόσταση
    δεν αλλάζει). Δεκτή μόνο αν στη νέα θέση η ετικέτα ΔΕΝ τέμνεται από γραμμή,
    κείμενο ή hatch. Αλλιώς μένει ως έχει."""
    names = sorted([n for n in model.blocks if re.match(r'FL-?\d+_BEAMBAR\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    moved = []
    for r in range(rounds):
        this = []
        for nm in names:
            res = slide_to_clean_only(model, nm)
            if res:
                this.append(res)
        moved.extend(this)
        if not this:
            break
    return moved


def _bar_ok(model, name, d, px, py, lines0):
    """Ελέγχει μια υποψήφια θέση ενός οπλισμού πλάκας: (α) οι γραμμές του δεν
    τέμνουν κείμενο δοκού/κολώνας/οπλισμού/slab, (β) η ετικέτα του δεν τέμνεται
    από γραμμή, κείμενο ή hatch. Επιστρέφει τις νέες γραμμές ή None."""
    cand = _translate_lines(lines0, px*d, py*d)
    if _lines_cut_any_text_strict(model, cand, {name}):
        return None
    saved = model.bar_lines[name]
    model.bar_lines[name] = cand
    sc = _label_badness(model, name, px*d, py*d)
    model.bar_lines[name] = saved
    if sc is None or sc[:6] != (0, 0, 0, 0, 0, 0):
        return None
    return cand


def _within_rule(region, ref, px, py, d):
    """§0: εντός των ορίων της πλάκας· αν το σημείο αναφοράς είναι ήδη έξω,
    επιτρέπεται μόνο κίνηση που το φέρνει πιο κοντά."""
    def dout(x_, y_):
        return math.hypot(max(0.0, region[0]-x_, x_-region[2]),
                           max(0.0, region[1]-y_, y_-region[3]))
    d0 = dout(ref[0], ref[1])
    dn = dout(ref[0]+px*d, ref[1]+py*d)
    if d0 <= 1e-9:
        return dn <= 1e-9
    return dn <= d0 - 1e-9


def _health_snapshot(model):
    """Κατάσταση αναγνωσιμότητας ΟΛΩΝ των κειμένων του σχεδίου: για κάθε
    ετικέτα οπλισμού, κείμενο δοκού και κείμενο κολώνας, πόσα προβλήματα έχει
    (τομή από γραμμή, επικάλυψη με άλλο κείμενο, hatch). Χρησιμοποιείται για να
    ελεγχθεί ότι μια διπλή μετακίνηση δεν έκανε ΑΛΛΟ, γειτονικό στοιχείο
    δυσανάγνωστο - αν το έκανε, η μετακίνηση αναιρείται."""
    tol = 0.03
    snap = {}
    all_lines = list(model.struct_only)
    for nm, segs in model.bar_lines.items():
        all_lines.extend(segs)
    for name, boxes in model.text_local.items():
        if not re.search(r'(BEAM_TEXT|COLUMN_TEXT|BEAMBAR|SLABBAR|SLAB)\d+$', name):
            continue
        own_beam = None
        mm = re.match(r'(FL-?\d+)_BEAM_TEXT(\d+)$', name)
        if mm:
            own_beam = f'{mm.group(1)}_BEAM{mm.group(2)}'
        bad = 0
        for x, y, w, h, rot in boxes:
            b = text_bbox(x, y, w, h, rot)
            inner = (b[0]+tol, b[1]+tol, b[2]-tol, b[3]-tol)
            if inner[0] >= inner[2] or inner[1] >= inner[3]:
                continue
            # γραμμές που κόβουν τον πυρήνα (εξαιρείται η δική του ράβδος/δοκός)
            for seg in all_lines:
                if len(seg) > 4 and seg[4] in (name, own_beam):
                    continue
                if seg_intersects_bbox(seg[:4], inner):
                    bad += 1
                    break
            # επικάλυψη με άλλο κείμενο
            for other, obs in model.text_local.items():
                if other == name:
                    continue
                for ox_, oy_, ow_, oh_, or_ in obs:
                    ob = text_bbox(ox_, oy_, ow_, oh_, or_)
                    if not (b[2] < ob[0] or ob[2] < b[0] or b[3] < ob[1] or ob[3] < b[1]):
                        bad += 1
                        break
                else:
                    continue
                break
            if model.box_on_hatch_bb(b):
                bad += 1
        snap[name] = bad
    return snap


def _health_worse(before, after):
    """Ονόματα στοιχείων που ΧΕΙΡΟΤΕΡΕΨΑΝ (έγιναν λιγότερο αναγνώσιμα)."""
    return [k for k, v in after.items() if v > before.get(k, 0)]


def _try_multi_beamtext(model, a, la, rega, refa, perp, partners, log):
    """Δοκιμάζει να μετακινήσει ΤΑΥΤΟΧΡΟΝΑ όλα τα εμπλεκόμενα κείμενα δοκών
    (ολίσθηση κατά μήκος, §4) μαζί με τον οπλισμό (κάθετα, §3). Δεκτό μόνο αν
    στο τέλος οι γραμμές και η ετικέτα του οπλισμού είναι καθαρές."""
    pax, pay = perp
    infos = []
    for bt in partners[:3]:
        mm = re.match(BEAMTEXT_RE, bt)
        beam = f'{mm.group(1)}_BEAM{mm.group(2)}'
        if beam not in model.blocks:
            return False
        g = beam_span(model, beam)
        saved = list(model.text_local.get(bt, []))
        if g is None or not saved:
            return False
        (ubx, uby), _, (tmin, tmax), _ = g
        bb = None
        for x, y, w, h, rot in saved:
            bx = text_bbox(x, y, w, h, rot)
            bb = bx if bb is None else (min(bb[0], bx[0]), min(bb[1], bx[1]),
                                         max(bb[2], bx[2]), max(bb[3], bx[3]))
        infos.append(dict(name=bt, beam=beam, u=(ubx, uby), lim=(tmin, tmax),
                           saved=saved, ct=((bb[0]+bb[2])/2)*ubx+((bb[1]+bb[3])/2)*uby))
    STEPS_T = [0.0] + [k*0.10*s for k in range(1, 13) for s in (1, -1)]
    STEPS_D = [k*STEP*s for k in range(1, 16) for s in (1, -1)]
    health0 = _health_snapshot(model)
    import itertools
    for combo in itertools.product(STEPS_T, repeat=len(infos)):
        ok_lim = True
        for inf, dt in zip(infos, combo):
            if dt and not (inf['lim'][0]+0.05 <= inf['ct']+dt <= inf['lim'][1]-0.05):
                ok_lim = False; break
        if not ok_lim:
            continue
        for inf, dt in zip(infos, combo):
            ux_, uy_ = inf['u']
            model.text_local[inf['name']] = [(x+ux_*dt, y+uy_*dt, w, h, rot)
                                              for x, y, w, h, rot in inf['saved']]
        for d in STEPS_D:
            if not _within_rule(rega, refa, pax, pay, d):
                continue
            nl = _bar_ok(model, a, d, pax, pay, la)
            if nl is None:
                continue
            for inf, dt in zip(infos, combo):
                if dt:
                    ux_, uy_ = inf['u']
                    model.deltas[inf['name']][0] += ux_*dt
                    model.deltas[inf['name']][1] += uy_*dt
                    log.append((inf['name'], round(dt, 2), a, round(d, 2)))
            model.deltas[a][0] += pax*d
            model.deltas[a][1] += pay*d
            model.bar_lines[a] = nl
            model.text_local[a] = [(x+pax*d, y+pay*d, w, h, rot)
                                    for x, y, w, h, rot in model.text_local[a]]
            model.refresh_obstacle_lines_for(a, nl)
            # ΕΠΑΝΕΛΕΓΧΟΣ: μήπως η διπλή μετακίνηση έκανε ΑΛΛΟ, γειτονικό
            # στοιχείο δυσανάγνωστο; Αν ναι, αναιρείται ολόκληρη.
            worse = _health_worse(health0, _health_snapshot(model))
            if worse:
                model.deltas[a][0] -= pax*d
                model.deltas[a][1] -= pay*d
                model.bar_lines[a] = la
                model.text_local[a] = [(x-pax*d, y-pay*d, w, h, rot)
                                        for x, y, w, h, rot in model.text_local[a]]
                model.refresh_obstacle_lines_for(a, la)
                for inf, dt in zip(infos, combo):
                    if dt:
                        ux_, uy_ = inf['u']
                        model.deltas[inf['name']][0] -= ux_*dt
                        model.deltas[inf['name']][1] -= uy_*dt
                        model.text_local[inf['name']] = inf['saved']
                        if log and log[-1][0] == inf['name']:
                            log.pop()
                while log and log[-1][2:3] == (a,):
                    log.pop()
                continue
            if not any(dt for dt in combo):
                log.append((a, round(pax*d, 2), round(pay*d, 2)))
            return True
        for inf in infos:
            model.text_local[inf['name']] = inf['saved']
    return False


def final_pair_with_beamtext(model, rounds=2):
    """ΖΕΥΓΟΣ ΟΠΛΙΣΜΟΥ + ΚΕΙΜΕΝΟΥ ΔΟΚΟΥ (ΚΑΝΟΝΙΣΜΟΣ §2, «δύσκολη περίπτωση»):
    όταν ένας οπλισμός πλάκας δεν έχει καθαρή θέση ΠΟΥΘΕΝΑ μέσα στην πλάκα του
    επειδή τον εμποδίζει κείμενο δοκού, μετακινούνται ΚΑΙ ΤΑ ΔΥΟ μαζί - το
    κείμενο δοκού ΜΟΝΟ με ολίσθηση κατά μήκος της δοκού του (§4), ο οπλισμός
    κάθετα εντός της πλάκας του (§3). Δεκτό μόνο αν στη νέα θέση: οι γραμμές και
    η ετικέτα του οπλισμού είναι καθαρές ΚΑΙ το κείμενο δοκού δεν χειροτερεύει.
    (Τεκμηριωμένο: BEAM_TEXT23 -0,30 κατά μήκος + SLABBAR21 +0,70 κάθετα.)"""
    moved = []
    names = sorted([n for n in model.blocks if re.match(SLABBAR_RE, n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    bt_all = [n for n in model.blocks if re.match(BEAMTEXT_RE, n)]
    D_STEPS = [k*STEP*s for k in range(1, 16) for s in (1, -1)]
    T_STEPS = [0.0] + [k*LBL_STEP*s for k in range(1, 25) for s in (1, -1)]
    tol = 0.03
    for r in range(rounds):
        this = []
        for a in names:
            la = model.bar_lines.get(a)
            if not la:
                continue
            ba = _label_badness(model, a, 0.0, 0.0)
            if ba is None:
                continue
            if not _lines_cut_any_text_strict(model, la, {a}) and ba[:6] == (0,)*6:
                continue
            health_pair = _health_snapshot(model)
            rega, refa = _find_region(model, la)
            if rega is None:
                continue
            (uax, uay), (pax, pay) = _axis_and_perp(la)
            # λύνεται μόνος του; τότε το αναλαμβάνει το άλλο πέρασμα
            if any(_bar_ok(model, a, d, pax, pay, la) is not None
                    for d in D_STEPS if _within_rule(rega, refa, pax, pay, d)):
                continue
            # ποια κείμενα δοκών εμπλέκονται (τα κόβουν οι γραμμές του ή
            # επικαλύπτουν την ετικέτα του)
            partners = []
            for bt in bt_all:
                for x, y, w, h, rot in model.text_local.get(bt, []):
                    bx = text_bbox(x, y, w, h, rot)
                    inner = (bx[0]+tol, bx[1]+tol, bx[2]-tol, bx[3]-tol)
                    if inner[0] < inner[2] and inner[1] < inner[3] and \
                       any(seg_intersects_bbox(s_[:4], inner) for s_ in la):
                        partners.append(bt)
                        break
            if not partners:
                continue
            solved = False
            # ΟΛΟΙ οι εμπλεκόμενοι μαζί: αν ο οπλισμός κόβει ΔΥΟ κείμενα δοκών,
            # η μετακίνηση ενός μόνο δεν αρκεί - πρέπει να υποχωρήσουν και τα δύο.
            if len(partners) > 1:
                solved = _try_multi_beamtext(model, a, la, rega, refa,
                                              (pax, pay), partners, this)
            for bt in ([] if solved else partners):
                mm = re.match(BEAMTEXT_RE, bt)
                beam = f'{mm.group(1)}_BEAM{mm.group(2)}'
                if beam not in model.blocks:
                    continue
                g = beam_span(model, beam)
                if g is None:
                    continue
                (ubx, uby), _, (tmin, tmax), _ = g
                saved_bt = list(model.text_local.get(bt, []))
                if not saved_bt:
                    continue
                bb0 = None
                for x, y, w, h, rot in saved_bt:
                    bx = text_bbox(x, y, w, h, rot)
                    bb0 = bx if bb0 is None else (min(bb0[0], bx[0]), min(bb0[1], bx[1]),
                                                   max(bb0[2], bx[2]), max(bb0[3], bx[3]))
                ct0 = ((bb0[0]+bb0[2])/2)*ubx + ((bb0[1]+bb0[3])/2)*uby
                for dt in T_STEPS:
                    if dt and not (tmin+0.05 <= ct0+dt <= tmax-0.05):
                        continue     # §4: ΠΑΝΤΑ εντός των ορίων της δοκού του
                    ldx, ldy = ubx*dt, uby*dt
                    model.text_local[bt] = [(x+ldx, y+ldy, w, h, rot)
                                             for x, y, w, h, rot in saved_bt]
                    for d in D_STEPS:
                        if not _within_rule(rega, refa, pax, pay, d):
                            continue
                        nl = _bar_ok(model, a, d, pax, pay, la)
                        if nl is None:
                            continue
                        # το κείμενο δοκού στη νέα του θέση δεν πρέπει να
                        # δημιουργεί δικό του πρόβλημα
                        excl = {bt, beam}
                        placed = model.placed_boxes_excluding(excl | {a})
                        bshift = [(x, y, w, h, rot) for x, y, w, h, rot in model.text_local[bt]]
                        if P11.parallel_cut_full(bshift, 0.0, 0.0, model.obstacle_lines,
                                                  exclude_line_names=excl):
                            continue
                        ovl = False
                        for x, y, w, h, rot in bshift:
                            bx = text_bbox(x, y, w, h, rot)
                            if model.box_on_hatch_bb(bx):
                                ovl = True; break
                            for pb in placed:
                                if not (bx[2] < pb[0] or pb[2] < bx[0] or
                                         bx[3] < pb[1] or pb[3] < bx[1]):
                                    ovl = True; break
                            if ovl:
                                break
                        if ovl:
                            continue
                        # ΔΕΚΤΟ - και τα δύο καθαρά
                        saved_a_lines = model.bar_lines[a]
                        saved_a_text = list(model.text_local[a])
                        if dt:
                            model.deltas[bt][0] += ldx
                            model.deltas[bt][1] += ldy
                        model.deltas[a][0] += pax*d
                        model.deltas[a][1] += pay*d
                        model.bar_lines[a] = nl
                        model.text_local[a] = [(x+pax*d, y+pay*d, w, h, rot)
                                                for x, y, w, h, rot in model.text_local[a]]
                        model.refresh_obstacle_lines_for(a, nl)
                        # ΕΠΑΝΕΛΕΓΧΟΣ γειτονικών - αν κάτι έγινε δυσανάγνωστο,
                        # η διπλή μετακίνηση αναιρείται ολόκληρη
                        if _health_worse(health_pair, _health_snapshot(model)):
                            if dt:
                                model.deltas[bt][0] -= ldx
                                model.deltas[bt][1] -= ldy
                            model.deltas[a][0] -= pax*d
                            model.deltas[a][1] -= pay*d
                            model.bar_lines[a] = saved_a_lines
                            model.text_local[a] = saved_a_text
                            model.refresh_obstacle_lines_for(a, saved_a_lines)
                            continue
                        this.append((bt, round(dt, 2), a, round(d, 2)))
                        solved = True
                        break
                    if solved:
                        break
                    model.text_local[bt] = saved_bt
                if solved:
                    break
        moved.extend(this)
        if not this:
            break
    return moved


def final_pair_relocate(model, rounds=2):
    """ΜΕΤΑΚΙΝΗΣΗ ΖΕΥΓΟΥΣ: όταν ένας οπλισμός πλάκας δεν μπορεί να καθαρίσει
    μόνος του επειδή τον εμποδίζει ΑΛΛΟΣ οπλισμός πλάκας, μετακινούνται ΚΑΙ ΟΙ
    ΔΥΟ μαζί - ο καθένας κατά τους δικούς του κανόνες (κάθετα, εντός της πλάκας
    του). Η λύση γίνεται δεκτή μόνο αν ΚΑΙ ΟΙ ΔΥΟ, στη νέα τους θέση, περνούν
    το πλήρες κριτήριο: γραμμές χωρίς τομή κειμένου ΚΑΙ ετικέτα χωρίς τομή από
    γραμμή/κείμενο/hatch."""
    moved = []
    names = sorted([n for n in model.blocks if re.match(SLABBAR_RE, n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    STEPS = [0.0] + [k*STEP*s for k in range(1, 16) for s in (1, -1)]
    for r in range(rounds):
        this = []
        for a in names:
            la = model.bar_lines.get(a)
            if not la:
                continue
            ba = _label_badness(model, a, 0.0, 0.0)
            if ba is None:
                continue
            if not _lines_cut_any_text_strict(model, la, {a}) and ba[:6] == (0,)*6:
                continue                     # ήδη καθαρός
            # ποιος ΑΛΛΟΣ οπλισμός πλάκας εμπλέκεται;
            partners = []
            tol = 0.03
            for x, y, w, h, rot in model.text_local.get(a, []):
                bx = text_bbox(x, y, w, h, rot)
                inner = (bx[0]+tol, bx[1]+tol, bx[2]-tol, bx[3]-tol)
                if inner[0] >= inner[2] or inner[1] >= inner[3]:
                    continue
                for b_, segs in model.bar_lines.items():
                    if b_ != a and re.match(SLABBAR_RE, b_) and \
                       any(seg_intersects_bbox(s_[:4], inner) for s_ in segs):
                        partners.append(b_)
            for b_ in list(model.bar_lines):
                if b_ == a or not re.match(SLABBAR_RE, b_):
                    continue
                for x, y, w, h, rot in model.text_local.get(b_, []):
                    bx = text_bbox(x, y, w, h, rot)
                    inner = (bx[0]+tol, bx[1]+tol, bx[2]-tol, bx[3]-tol)
                    if inner[0] < inner[2] and inner[1] < inner[3] and \
                       any(seg_intersects_bbox(s_[:4], inner) for s_ in la):
                        partners.append(b_)
            partners = list(dict.fromkeys(partners))
            if not partners:
                continue

            rega, refa = _find_region(model, la)
            if rega is None:
                continue
            (uax, uay), (pax, pay) = _axis_and_perp(la)
            solved = False
            for b in partners:
                lb = model.bar_lines.get(b)
                if not lb:
                    continue
                regb, refb = _find_region(model, lb)
                if regb is None:
                    continue
                (ubx, uby), (pbx, pby) = _axis_and_perp(lb)
                for db in STEPS:
                    if db and not _within_rule(regb, refb, pbx, pby, db):
                        continue
                    savedb = model.bar_lines[b]
                    savedtb = model.text_local.get(b)
                    if db:
                        model.bar_lines[b] = _translate_lines(lb, pbx*db, pby*db)
                        model.text_local[b] = [(x+pbx*db, y+pby*db, w, h, rot)
                                                for x, y, w, h, rot in savedtb]
                    okb = _bar_ok(model, b, 0.0, 0.0, 0.0, model.bar_lines[b])
                    if okb is not None:
                        for da in STEPS:
                            if da and not _within_rule(rega, refa, pax, pay, da):
                                continue
                            if _bar_ok(model, a, da, pax, pay, la) is not None:
                                # δεκτό: και οι δύο καθαροί
                                if db:
                                    model.deltas[b][0] += pbx*db
                                    model.deltas[b][1] += pby*db
                                    model.refresh_obstacle_lines_for(b, model.bar_lines[b])
                                    this.append((b, round(pbx*db, 2), round(pby*db, 2)))
                                if da:
                                    nl = _translate_lines(la, pax*da, pay*da)
                                    model.deltas[a][0] += pax*da
                                    model.deltas[a][1] += pay*da
                                    model.bar_lines[a] = nl
                                    model.text_local[a] = [(x+pax*da, y+pay*da, w, h, rot)
                                                            for x, y, w, h, rot in model.text_local[a]]
                                    model.refresh_obstacle_lines_for(a, nl)
                                    this.append((a, round(pax*da, 2), round(pay*da, 2)))
                                solved = True
                                break
                    if solved:
                        break
                    model.bar_lines[b] = savedb
                    if savedtb is not None:
                        model.text_local[b] = savedtb
                if solved:
                    break
        moved.extend(this)
        if not this:
            break
    return moved


def final_column_text_clean(model, rounds=2):
    """ΤΕΛΙΚΟ ΒΗΜΑ - κείμενα κολώνας: κυκλική περιφορά γύρω από τη ΔΙΚΗ του
    κολώνα, με βήμα ακτίνας, στο ΤΕΛΙΚΟ τοπίο (όλα τα άλλα έχουν ήδη κατακάτσει).
    Σειρά προτίμησης σε ΚΑΘΕ ακτίνα, από την πλησιέστερη προς τα έξω:
      1) ΤΕΛΕΙΑ θέση: καμία γραμμή, κανένα κείμενο, κανένα hatch
      2) θέση όπου γραμμή περνά ΜΟΝΟ ανάμεσα σε ψηφία/γράμματα (§ chargap)
    Η ιδιοκτησία (§6) τηρείται: πάντα πιο κοντά στη δική του παρά σε άλλη."""
    from analyze import entities_from_pairs as _efp, to_dict as _td
    from beambar_engine import strip_mtext_formatting as _smf
    moved = []
    cols = {}
    for c in model.blocks:
        if re.match(r'FL-?\d+_COLUMN\d+$', c):
            wl = _world_lines(model.blocks, model.ins, c)
            if wl:
                xs = [p for s_ in wl for p in (s_[0], s_[2])]
                ys = [p for s_ in wl for p in (s_[1], s_[3])]
                cols[c] = (min(xs), min(ys), max(xs), max(ys))
    names = sorted([n for n in model.blocks if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    for r_ in range(rounds):
        this = []
        for name in names:
            m = re.match(r'(FL-?\d+)_COLUMN_TEXT(\d+)$', name)
            own = f'{m.group(1)}_COLUMN{m.group(2)}'
            if own not in cols or name not in model.text_local:
                continue
            boxes = [(x, y, w, h, rot) for x, y, w, h, rot in model.text_local[name]]
            excl = {name, own}
            placed = model.placed_boxes_excluding(excl)
            if P11.is_ok_full(boxes, 0.0, 0.0, model.obstacle_lines, model.hatch_polys,
                               placed, exclude_line_names=excl):
                continue                       # ήδη τέλειο
            contents = [_smf(_td(e).get(1, [''])[0])
                         for e in _efp(model.blocks[name]) if e[0][1] == 'MTEXT']
            tb = None
            for x, y, w, h, rot in boxes:
                b = text_bbox(x, y, w, h, rot)
                tb = b if tb is None else (min(tb[0], b[0]), min(tb[1], b[1]),
                                            max(tb[2], b[2]), max(tb[3], b[3]))
            tcx, tcy = (tb[0]+tb[2])/2, (tb[1]+tb[3])/2
            ocx, ocy = (cols[own][0]+cols[own][2])/2, (cols[own][1]+cols[own][3])/2
            # Η ΕΓΓΥΤΗΤΑ ΥΠΕΡΙΣΧΥΕΙ (§6: «προέχει να είναι όσο το δυνατόν πιο
            # κοντά»). Η αναζήτηση γίνεται δακτύλιο-δακτύλιο από την πλησιέστερη
            # ακτίνα προς τα έξω, και σε ΚΑΘΕ ακτίνα γίνεται δεκτή είτε τέλεια
            # θέση είτε θέση με γραμμή ΑΝΑΜΕΣΑ στα ψηφία - με προτίμηση την
            # τέλεια αν υπάρχει στην ΙΔΙΑ ακτίνα. Έτσι δεν φεύγει μέτρα μακριά
            # για μια τέλεια θέση όταν δίπλα υπάρχει απολύτως αναγνώσιμη.
            found = None
            rad = 0.05
            while rad <= 2.50 and found is None:
                n_dirs = max(24, int(2*math.pi*rad/0.05))
                cg_here = None
                for k in range(n_dirs):
                    a = 2*math.pi*k/n_dirs
                    nx, ny = ocx + rad*math.cos(a), ocy + rad*math.sin(a)
                    ldx, ldy = nx-tcx, ny-tcy
                    nb = (tb[0]+ldx, tb[1]+ldy, tb[2]+ldx, tb[3]+ldy)
                    nd_own = _gap(nb, cols[own])
                    if any(_gap(nb, b2) < nd_own for c2, b2 in cols.items() if c2 != own):
                        continue               # §6 ιδιοκτησία
                    if P11.is_ok_full(boxes, ldx, ldy, model.obstacle_lines,
                                       model.hatch_polys, placed, exclude_line_names=excl):
                        found = (ldx, ldy, 'τέλεια')
                        break
                    if cg_here is None and contents and P11.is_ok_chargap(
                            boxes, contents, ldx, ldy, model.obstacle_lines,
                            model.hatch_polys, placed, exclude_line_names=excl):
                        cg_here = (ldx, ldy, 'ανάμεσα στα ψηφία')
                if found is None and cg_here is not None:
                    found = cg_here            # ίδια ακτίνα - η εγγύτητα κερδίζει
                rad += 0.05
            if found is None:
                continue
            ldx, ldy, kind = found
            # §8 επανέλεγχος: να μη χαλάσει γειτονικό στοιχείο
            h0 = _health_snapshot(model)
            model.text_local[name] = [(x+ldx, y+ldy, w, h, rot) for x, y, w, h, rot in boxes]
            if _health_worse(h0, _health_snapshot(model)):
                model.text_local[name] = boxes
                continue
            model.deltas[name][0] += ldx
            model.deltas[name][1] += ldy
            this.append((name.replace('_COLUMN_TEXT', ' Κ'), round(ldx, 2), round(ldy, 2), kind))
        moved.extend(this)
        if not this:
            break
    return moved


def final_labels_within_bar(model, rounds=2):
    """ΚΑΝΟΝΙΣΜΟΣ §3: η ετικέτα οπλισμού κάθεται ΚΑΤΑ ΜΗΚΟΣ της ράβδου της -
    «κατά προτίμηση κεντραρισμένη, επιτρέπεται και οπουδήποτε κατά μήκος, και
    κοντά στο ένα άκρο». ΠΟΤΕ πέρα από τα άκρα της. Στο αρχείο-μήτρα καμία δεν
    είναι εκτός· η τακτοποίηση όμως βγάζει μερικές, και δεν υπήρχε τίποτα να
    τις επαναφέρει. Εδώ επαναφέρονται με ολίσθηση ΜΟΝΟ κατά μήκος, επιλέγοντας
    τη λιγότερο κακή θέση εντός ορίων κατά την ιεραρχία §1."""
    moved = []
    names = sorted([n for n in model.blocks if re.match(REBAR_RE, n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    for r_ in range(rounds):
        this = []
        for name in names:
            g = _label_geom(model, name)
            if g is None:
                continue
            (ux, uy), (px, py), lo, hi, bb, boxes = g
            ct = ((bb[0]+bb[2])/2)*ux + ((bb[1]+bb[3])/2)*uy
            if lo-0.05 <= ct <= hi+0.05:
                continue                  # ήδη εντός μήκους
            # στόχος: η κοντινότερη θέση ΕΝΤΟΣ του μήκους της ράβδου
            target = min(max(ct, lo+0.05), hi-0.05)
            best = None
            for extra in [k*LBL_STEP*s_ for k in range(0, 25) for s_ in (1, -1)]:
                t = target + extra
                if not (lo+0.02 <= t <= hi-0.02):
                    continue
                dt = t - ct
                ldx, ldy = ux*dt, uy*dt
                sc = _label_badness(model, name, ldx, ldy)
                if sc is None:
                    continue
                key = (sc[:6], abs(extra))
                if best is None or key < best[2]:
                    best = (ldx, ldy, key)
            if best is None:
                continue
            ldx, ldy, _k = best
            h0 = _health_snapshot(model)
            saved = list(model.text_local[name])
            model.text_local[name] = [(x+ldx, y+ldy, w, h, rot) for x, y, w, h, rot in saved]
            # ο επανέλεγχος αφορά τα ΑΛΛΑ στοιχεία: η ίδια η ετικέτα εξαιρείται,
            # γιατί η επαναφορά της εντός των ορίων της ράβδου (§3) είναι το
            # ζητούμενο - μπορεί η νέα θέση να είναι πιο στριμωγμένη γι' αυτήν,
            # αλλά «εκτός ράβδου» δεν είναι αποδεκτή κατάσταση.
            _w = [k for k in _health_worse(h0, _health_snapshot(model)) if k != name]
            if _w:
                model.text_local[name] = saved
                continue
            model.label_deltas[name][0] += ldx
            model.label_deltas[name][1] += ldy
            this.append((name, round(ldx, 2), round(ldy, 2)))
        moved.extend(this)
        if not this:
            break
    return moved


def run(input_path, output_path, rounds=4, orig_path=None):
    model = Model(input_path, orig_path=orig_path)
    names = sorted([n for n in model.blocks if re.match(REBAR_RE, n)],
                    key=lambda n: (0 if 'SLABBAR' in n else 1,
                                    int(re.search(r'\d+$', n).group())))
    for r in range(rounds):
        before = {n: tuple(model.deltas[n]) for n in names}
        for name in names:
            process_slabbar(model, name)
        if all(tuple(model.deltas[n]) == before[n] for n in names):
            break

    # §4 μηχανισμός επαναφοράς κειμένων δοκού (μετά τους οπλισμούς, ώστε να
    # βλέπει το τελικό τοπίο)
    bt_names = sorted([n for n in model.blocks if re.match(BEAMTEXT_RE, n)],
                       key=lambda n: int(re.search(r'\d+$', n).group()))
    restored = []
    for name in bt_names:
        r_ = process_beam_text(model, name)
        if r_:
            restored.append(r_)
    if restored:
        print('§4 επαναφορά κειμένων δοκού (μηδενισμός κάθετης μετατόπισης):', restored)

    col_moves = process_column_texts(model)
    if col_moves:
        print('§6 ιδιοκτησία κειμένων κολώνας (από/προς απόσταση):', col_moves)

    marker_moves = process_slab_markers(model)
    if marker_moves:
        print('§5 δείκτες πλακών (κυκλάκι+κείμενα μαζί):', marker_moves)

    # §8 ΕΠΑΛΗΘΕΥΣΗ/ROLLBACK: η επαναφορά κειμένου δοκού έγινε αγνοώντας τους
    # δείκτες, με την υπόθεση ότι ο δείκτης θα υποχωρήσει (§5). Αν τελικά ΔΕΝ
    # υποχώρησε (π.χ. δεν χωράει αλλού μέσα στην πλάκα του), η επαναφορά
    # χειροτερεύει: 1 παραβίαση §4 γίνεται 2 (επικάλυψη + κείμενο σε κυκλάκι).
    # Τότε αναιρείται - «καμία μετακίνηση δεκτή αν χειροτερεύει» (§8).
    rolled = []
    slid_ok = []
    for name, ldx, ldy in list(restored):
        bad = False
        for x, y, w, h, rot in model.text_local.get(name, []):
            b = text_bbox(x, y, w, h, rot)
            for mk_ in model.marker_names:
                for cb in model.circle_boxes.get(mk_, []):
                    if not (b[2] < cb[0] or cb[2] < b[0] or b[3] < cb[1] or cb[3] < b[1]):
                        bad = True
                for mx, my, mw, mh, mr in model.text_local.get(mk_, []):
                    mb = text_bbox(mx, my, mw, mh, mr)
                    if not (b[2] < mb[0] or mb[2] < b[0] or b[3] < mb[1] or mb[3] < b[1]):
                        bad = True
        if bad:
            # ΠΡΩΤΑ: το κείμενο δοκού έχει προτεραιότητα να μείνει επαναφερμένο -
            # δοκιμάζουμε ΜΟΝΟ ολίσθηση ΚΑΤΑ ΜΗΚΟΣ (§4) ώστε να ξεφύγει από τον
            # δείκτη, χωρίς να ξαναβγεί κάθετα.
            slid = _slide_along_to_clear_markers(model, name)
            if slid:
                slid_ok.append((name, slid))
                continue
            model.deltas[name][0] -= ldx
            model.deltas[name][1] -= ldy
            model.text_local[name] = [(x-ldx, y-ldy, w, h, rot)
                                       for x, y, w, h, rot in model.text_local[name]]
            rolled.append(name)
    if slid_ok:
        print('§4 ολίσθηση κατά μήκος για αποφυγή δείκτη:', slid_ok)
    if rolled:
        print('§8 αναίρεση επαναφοράς (ο δείκτης δεν μπόρεσε να υποχωρήσει):', rolled)

    clean0 = final_clean_slide(model)
    _t0 = sum(len(v) for v in clean0.values())
    if _t0:
        print('ΚΑΘΑΡΕΣ ΘΕΣΕΙΣ (πριν τον συμβιβασμό) - δοκών:%d, πλακών:%d, δοκών ξανά:%d'
              % (len(clean0['δοκών_α']), len(clean0['πλακών']), len(clean0['δοκών_β'])))

    sweep = final_sweep(model)
    if sweep['beambar'] or sweep['slabbar'] or sweep['beamtext']:
        print('ΤΕΛΙΚΟΣ ΕΛΕΓΧΟΣ 1) ετικέτες δοκών:', len(sweep['beambar']),
              '2) ετικέτες πλακών:', len(sweep['slabbar']),
              '3) κείμενα δοκών:', len(sweep['beamtext']), sweep['beamtext'])

    clean = final_clean_slide(model)
    _tot = sum(len(v) for v in clean.values())
    if _tot:
        print('ΤΕΛΙΚΟ ΒΗΜΑ (ολίσθηση σε ΚΑΘΑΡΗ θέση) - δοκών:%d, πλακών:%d, δοκών ξανά:%d'
              % (len(clean['δοκών_α']), len(clean['πλακών']), len(clean['δοκών_β'])))

    reloc = final_rebar_relocate(model)
    if reloc:
        print('ΜΕΤΑΤΟΠΙΣΗ ΟΠΛΙΣΜΟΥ (γραμμή/ετικέτα -> καθαρά και τα δύο):', len(reloc), reloc)

    pair = final_pair_relocate(model)
    if pair:
        print('ΜΕΤΑΚΙΝΗΣΗ ΖΕΥΓΟΥΣ ΡΑΒΔΩΝ:', len(pair), pair)

    pair_bt = final_pair_with_beamtext(model)
    if pair_bt:
        print('ΖΕΥΓΟΣ ΟΠΛΙΣΜΟΥ + ΚΕΙΜΕΝΟΥ ΔΟΚΟΥ (§2):', len(pair_bt), pair_bt)

    col_clean = final_column_text_clean(model)
    if col_clean:
        print('ΤΕΛΙΚΟ ΒΗΜΑ (κυκλική περιφορά κειμένων κολώνας):', len(col_clean), col_clean)

    within = final_labels_within_bar(model)
    if within:
        print('§3 επαναφορά ετικετών ΕΝΤΟΣ του μήκους της ράβδου τους:', len(within), within)

    slab_clean = final_slab_label_clean(model)
    if slab_clean:
        print('ΤΕΛΕΥΤΑΙΟ ΒΗΜΑ (ετικέτες πλακών -> καθαρή θέση):', len(slab_clean), slab_clean)

    beam_clean = final_beam_label_clean(model)
    if beam_clean:
        print('ΤΕΛΕΥΤΑΙΟ ΒΗΜΑ (ετικέτες δοκών -> καθαρή θέση):', len(beam_clean), beam_clean)

    block_deltas = {n: tuple(d) for n, d in model.deltas.items() if d[0] or d[1]}
    label_deltas = {}
    for n, d in model.label_deltas.items():
        if not (d[0] or d[1]):
            continue
        layers = set()
        for e in entities_from_pairs(model.blocks[n]):
            if e[0][1] == 'MTEXT':
                layers.add(to_dict(e).get(8, [''])[0])
        label_deltas[n] = (d[0], d[1], layers)

    # οι δείκτες: ΟΛΟΚΛΗΡΟΙ - κυκλάκι/γραμμές (patch_slab_marker_geometry) ΚΑΙ
    # τα κείμενά τους (patch_block_mtext), με το ΙΔΙΟ delta, ώστε να μην ξεκολλήσει
    # το περιεχόμενο από το κυκλάκι. Το slab_poly μένει άθικτο.
    marker_geom = {n: tuple(d) for n, d in model.marker_deltas.items() if d[0] or d[1]}
    for n, d in model.marker_deltas.items():
        if not (d[0] or d[1]):
            continue
        layers = set()
        for e in entities_from_pairs(model.blocks[n]):
            if e[0][1] == 'MTEXT':
                layers.add(to_dict(e).get(8, [''])[0])
        label_deltas[n] = (d[0], d[1], layers)

    import os
    tmp1 = output_path + '.s1.dxf'
    tmp2 = output_path + '.s2.dxf'
    n1 = patch_dxf(input_path, tmp1, block_deltas)
    patch_slab_marker_geometry(tmp1, tmp2, marker_geom)
    n2 = patch_block_mtext(tmp2, output_path, label_deltas)
    os.remove(tmp1); os.remove(tmp2)
    print(f'final_pass: {len(block_deltas)} ράβδοι μετακινήθηκαν (insert), '
          f'{len(label_deltas)} ετικέτες ολίσθησαν (internal MTEXT)')
    return block_deltas, label_deltas


if __name__ == '__main__':
    run(sys.argv[1], sys.argv[2], orig_path=(sys.argv[3] if len(sys.argv) > 3 else None))
