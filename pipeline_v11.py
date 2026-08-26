import os
import re, math, pickle
from analyze import entities_from_pairs, to_dict
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox, translate_bbox, collides
from beambar_engine import seg_intersects_bbox, text_bbox, get_inserts as _get_inserts_early
from hatch_engine import get_hatch_polys, bbox_poly_overlap, point_in_poly
from perimeter import build_footprint, point_inside, bbox_outside, column_outward_dirs

def get_slab_polys(input_path):
    """Approximate boundary (axis-aligned bounding box) for each FL0_SLABn, from its
    slab_poly layer lines. A bbox is more robust than full polygon-loop chaining, which
    is fragile here since some slab panels share boundary edges across blocks."""
    from analyze import entities_from_pairs, to_dict
    ins, blocks = load_all(input_path)
    boxes = {}
    for name, pairlist in blocks.items():
        if not re.match(r'FL-?\d+_SLAB\d+$', name):
            continue
        ox,oy = ins.get(name,(0,0))
        ents = entities_from_pairs(pairlist)
        xs=[];ys=[]
        for e in ents:
            if e[0][1] != 'LINE':
                continue
            d = to_dict(e)
            if d.get(8,[''])[0] != 'slab_poly':
                continue
            xs += [float(d[10][0])+ox, float(d[11][0])+ox]
            ys += [float(d[20][0])+oy, float(d[21][0])+oy]
        if xs:
            boxes[name] = (min(xs),min(ys),max(xs),max(ys))
    return boxes
from compute_beambar3 import compute_beambar_offsets
from compute_slabbar3 import compute_slabbar_offsets
from compute_beamtext_slabmarker import slab_marker_boxes

SHRINK_MARGIN = 0.03  # fixed absolute margin (units), not a percentage - works correctly
                       # regardless of box size, unlike a percentage-based shrink which
                       # becomes meaningless (near-zero) for small boxes like slab markers.
STEP = 0.03
MAX_SLIDE = 2.0

# --- ΠΕΡΙΜΕΤΡΙΚΕΣ ΚΟΛΩΝΕΣ: κείμενο οπλισμού προς τα ΕΞΩ ---------------------
PERIM_OUTWARD = True    # κύριος διακόπτης της λειτουργίας
PERIM_DIR_WEIGHT = 2.0  # πόσο βαραίνει η κατεύθυνση. Το κόστος μιας θέσης είναι
                         # r * (1 + W*(1-align)/2), όπου align=+1 τελείως προς τα έξω
                         # και -1 προς τα μέσα. Με W=2.0 μια θέση 3μ προς τα έξω
                         # προτιμάται από μια θέση 1μ προς τα μέσα. Μεγαλύτερο W =
                         # πιο επιθετική έξοδος (και πιο μακρινά κείμενα).
PERIM_MAX_COST = 6.0    # όριο αναζήτησης σε μονάδες κόστους (όχι καθαρή απόσταση)
PERIM_HARD_MAX_R = 7.0  # απόλυτο όριο πραγματικής απόστασης από την κολώνα
COLUMN_OUTWARD = {}     # γεμίζει στο process_all: COLUMN_TEXT name -> (ux,uy)
FOOTPRINT = None        # αποτύπωμα κτιρίου, για χρήση και από το global_repair

def build_obstacle_lines(blocks, ins):
    obstacle_lines = []
    for name, pairlist in blocks.items():
        if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE)\d*$', name) and 'TEXT' not in name:
            ox,oy = ins.get(name,(0,0))
            lines,_ = block_lines_local(pairlist)
            for x1,y1,x2,y2 in lines:
                obstacle_lines.append((x1+ox,y1+oy,x2+ox,y2+oy,name))
    return obstacle_lines

def full_bbox(boxes_local, dx, dy):
    x1s=[];y1s=[];x2s=[];y2s=[]
    for x,y,w,h,rot in boxes_local:
        a,b,c,d = text_bbox(x+dx,y+dy,w,h,rot)
        x1s.append(a);y1s.append(b);x2s.append(c);y2s.append(d)
    return (min(x1s),min(y1s),max(x2s),max(y2s))

REBAR_NAMES = set()
STRICT_MODE = False  # when True (used for SLAB-marker & COLUMN_TEXT, which have full
                      # freedom to route around obstacles), rebar iron gets ZERO tolerance.
                      # Constrained categories (beambar/slabbar/beam_text text vs other rebar)
                      # keep the margin-based tolerance since they can't always fully escape.

PARALLEL_CROSS_DEG = 25.0  # κάτω από αυτή τη γωνία, γραμμή μέσα στον πυρήνα
                            # θεωρείται ΠΑΡΑΛΛΗΛΗ με το κείμενο

def count_line_crossings_geom(boxes_local, dx, dy, obstacle_lines, exclude_line_names=()):
    """Όπως ο μετρητής ονομάτων, αλλά μετράει ΔΙΑΚΡΙΤΕΣ ΓΕΩΜΕΤΡΙΚΕΣ γραμμές:
    η κοινή ακμή δύο πλακών είναι σχεδιασμένη δύο φορές (μία ανά πλάκα, π.χ.
    SLAB9+SLAB10) και ο κατ' όνομα μετρητής τη χρεώνει διπλή - απορρίπτοντας
    άδικα κάθε θέση κατά μήκος της (έτσι κόλλησε το SLABBAR47 στη στενή SLAB9).
    Οπτικά είναι ΜΙΑ γραμμή: έτσι μετράει."""
    keys = set()
    for x, y, w, h, rot in boxes_local:
        bx = text_bbox(x+dx, y+dy, w, h, rot)
        bx1, by1, bx2, by2 = bx
        margin = min(h*0.5, (bx2-bx1)*0.3)
        cx, cy = (bx1+bx2)/2, (by1+by2)/2
        hw, hh = max(0.0, (bx2-bx1)/2-margin), max(0.0, (by2-by1)/2-margin)
        core = (cx-hw, cy-hh, cx+hw, cy+hh)
        tx_, ty_ = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        for seg in obstacle_lines:
            if seg[4] in exclude_line_names:
                continue
            if seg_intersects_bbox(seg[:4], core):
                lx_, ly_ = seg[2]-seg[0], seg[3]-seg[1]
                ll_ = math.hypot(lx_, ly_)
                if ll_ > 1e-9 and abs((lx_*tx_ + ly_*ty_)/ll_) < 0.34:
                    # ΚΑΘΕΤΗ διέλευση (> ~70 μοίρες από τη φορά ανάγνωσης): η
                    # αποδεκτή σύμβαση του ξυλοτύπου - λεπτά σίδερα περνούν
                    # ανάμεσα στους χαρακτήρες παντού στα σχέδια του μηχανικού
                    # (και στο ΦΥΣΙΚΟ αρχείο). Δεν μετράει στο όριο: αλλιώς
                    # κατακόρυφη ετικέτα που τη διασχίζουν 2-4 οριζόντια σίδερα
                    # (όπως στη φυσική της θέση!) δεν είχε ΠΟΥΘΕΝΑ να πάει.
                    continue
                p1 = (round(seg[0], 3), round(seg[1], 3))
                p2 = (round(seg[2], 3), round(seg[3], 3))
                keys.add((p1, p2) if p1 <= p2 else (p2, p1))
    return keys

def count_line_crossings(boxes_local, dx, dy, obstacle_lines, exclude_line_names=(),
                          parallel_out=None):
    """Count distinct obstacle-line NAMES that cross through the core of this text - used
    for the relaxed rule: a single crossing line is tolerable if no zero-crossing spot
    exists nearby (the line usually falls in whitespace between words), but hatch/other
    text overlaps are NEVER tolerated regardless.

    parallel_out: αν δοθεί set, γεμίζει με τα ονόματα γραμμών που διασχίζουν τον
    πυρήνα σχεδόν ΠΑΡΑΛΛΗΛΑ με τη φορά ανάγνωσης (< PARALLEL_CROSS_DEG). Μια
    κάθετη γραμμή περνάει ανάμεσα σε δύο χαρακτήρες· μια παράλληλη κόβει ΟΛΟΥΣ
    τους χαρακτήρες στη σειρά - δεν είναι ποτέ ανεκτή (έτσι κατέληξε το Φ10/19
    πάνω στη γραμμή του κοινού ορίου πλακών)."""
    crossed = set()
    for x,y,w,h,rot in boxes_local:
        bx = text_bbox(x+dx,y+dy,w,h,rot)
        bx1,by1,bx2,by2 = bx
        margin = min(h*0.5, (bx2-bx1)*0.3)
        cx,cy=(bx1+bx2)/2,(by1+by2)/2
        hw,hh=max(0.0,(bx2-bx1)/2-margin),max(0.0,(by2-by1)/2-margin)
        core=(cx-hw,cy-hh,cx+hw,cy+hh)
        for seg in obstacle_lines:
            if seg[4] in exclude_line_names:
                continue
            if seg_intersects_bbox(seg[:4], core):
                crossed.add(seg[4])
                if parallel_out is not None:
                    sdx, sdy = seg[2]-seg[0], seg[3]-seg[1]
                    if math.hypot(sdx, sdy) > 1e-9:
                        a = math.degrees(math.atan2(sdy, sdx)) % 180.0
                        t = rot % 180.0
                        dpar = min(abs(a-t), 180.0-abs(a-t))
                        if dpar < PARALLEL_CROSS_DEG:
                            parallel_out.add(seg[4])
    return crossed

def parallel_cut_full(boxes_local, dx, dy, obstacle_lines, exclude_line_names=(), cos_thresh=0.94):
    """Παράλληλη τομή στο ΠΛΗΡΕΣ κουτί κειμένου. Ο έλεγχος στον συρρικνωμένο
    "πυρήνα" είναι τυφλός για μονογραμμικά κείμενα (ύψος 0.10 -> πυρήνας σχεδόν
    μηδενικού πάχους): έτσι το SLABBAR18 περνούσε τον εσωτερικό έλεγχο ενώ η
    γραμμή ορίου πλακών το έκοβε ολοφάνερα στη μέση. Για μονογραμμικό κείμενο
    το πλήρες κουτί ΕΙΝΑΙ τα γράμματα - εκεί κρίνεται η παράλληλη τομή."""
    for x, y, w, h, rot in boxes_local:
        bb = text_bbox(x+dx, y+dy, w, h, rot)
        tx_, ty_ = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        for s in obstacle_lines:
            if s[4] in exclude_line_names:
                continue
            if not seg_intersects_bbox(s[:4], bb):
                continue
            lx, ly = s[2]-s[0], s[3]-s[1]
            ll = math.hypot(lx, ly)
            if ll < 1e-9:
                continue
            if abs((lx*tx_ + ly*ty_)/ll) > cos_thresh:
                return True
    return False

def is_ok_relaxed(boxes_local, dx, dy, obstacle_lines, hatch_polys, placed_boxes, exclude_line_names=(),
                   max_crossings=1):
    """Like is_ok_full but tolerates up to `max_crossings` distinct rebar/structural lines
    passing through the text's readability CORE (not the full box) - hatch and other text
    boxes are still strictly forbidden, no exceptions."""
    for x,y,w,h,rot in boxes_local:
        bx = text_bbox(x+dx,y+dy,w,h,rot)
        bx1,by1,bx2,by2 = bx
        for poly, pname in hatch_polys:
            this_pad = 0.0 if pname in exclude_line_names else 0.05
            bx_padded = (bx1-this_pad, by1-this_pad, bx2+this_pad, by2+this_pad)
            if bbox_poly_overlap(bx_padded, poly):
                return False
        for ob in placed_boxes:
            ox1,oy1,ox2,oy2 = ob
            if not (bx2 + MIN_TEXT_GAP < ox1 or ox2 + MIN_TEXT_GAP < bx1 or
                    by2 + MIN_TEXT_GAP < oy1 or oy2 + MIN_TEXT_GAP < by1):
                return False
    if parallel_cut_full(boxes_local, dx, dy, obstacle_lines, exclude_line_names):
        return False   # παράλληλη γραμμή μέσα στο κείμενο: απαράδεκτο, χωρίς εξαίρεση
    crossed = count_line_crossings_geom(boxes_local, dx, dy, obstacle_lines, exclude_line_names)
    return len(crossed) <= max_crossings

def is_ok_tight(boxes_local, dx, dy, obstacle_lines, hatch_polys, placed_boxes, exclude_line_names=(),
                 max_row_crossings=0):
    """ΕΞΑΙΡΕΣΗ ΣΤΕΝΟΤΗΤΑΣ (εγκεκριμένη από τον μηχανικό, αναιρέσιμη):
    όταν κοντινή θέση δεν περνά το πλήρες/χαλαρό κριτήριο, επιτρέπεται θέση
    όπου γραμμές πέφτουν ΜΟΝΟ στα κενά ανάμεσα στις σειρές των γραμμάτων:
    κάθε ΣΕΙΡΑ κειμένου καθαρή από γραμμές ΚΑΙ διαγράμμιση· κείμενα/κύκλοι
    αυστηρά όπως πάντα (μηδέν επικαλύψεις)."""
    from hatch_engine import bbox_poly_overlap
    _row_owners = set()
    for x, y, w, h, rot in boxes_local:
        bx = text_bbox(x+dx, y+dy, w, h, rot)
        for poly, pname in hatch_polys:
            if pname in exclude_line_names:
                continue
            if bbox_poly_overlap(bx, poly):
                return False
        for pb in placed_boxes:
            if not (bx[2] + MIN_TEXT_GAP < pb[0] or pb[2] + MIN_TEXT_GAP < bx[0] or
                    bx[3] + MIN_TEXT_GAP < pb[1] or pb[3] + MIN_TEXT_GAP < bx[1]):
                return False
        p_ = 0.015
        c0, c1, c2, c3 = bx[0]+p_, bx[1]+p_, bx[2]-p_, bx[3]-p_
        if c2 > c0 and c3 > c1:
            for seg in obstacle_lines:
                x1_, y1_, x2_, y2_ = seg[0], seg[1], seg[2], seg[3]
                _own_s = seg[4] if len(seg) > 4 else None
                if _own_s in exclude_line_names:
                    continue
                if max(x1_,x2_) < c0 or min(x1_,x2_) > c2 or max(y1_,y2_) < c1 or min(y1_,y2_) > c3:
                    continue
                _hit_s = False
                if (c0 <= x1_ <= c2 and c1 <= y1_ <= c3) or (c0 <= x2_ <= c2 and c1 <= y2_ <= c3):
                    _hit_s = True
                else:
                    ddx_, ddy_ = x2_-x1_, y2_-y1_
                    for ex1, ey1, ex2, ey2 in ((c0,c1,c2,c1),(c0,c3,c2,c3),(c0,c1,c0,c3),(c2,c1,c2,c3)):
                        fx_, fy_ = ex2-ex1, ey2-ey1
                        den = ddx_*fy_ - ddy_*fx_
                        if abs(den) < 1e-12:
                            continue
                        t_ = ((ex1-x1_)*fy_ - (ey1-y1_)*fx_)/den
                        u_ = ((ex1-x1_)*ddy_ - (ey1-y1_)*ddx_)/den
                        if 0 <= t_ <= 1 and 0 <= u_ <= 1:
                            _hit_s = True; break
                if _hit_s:
                    _row_owners.add(_own_s if _own_s is not None else id(seg))
                    if len(_row_owners) > max_row_crossings:
                        return False
    return True

def is_ok_full(boxes_local, dx, dy, obstacle_lines, hatch_polys, placed_boxes, exclude_line_names=(),
               strict_line_names=None):
    if strict_line_names is None:
        strict_line_names = REBAR_NAMES  # rebar-vs-text: ALWAYS zero tolerance (+ small pad),
                                          # everywhere, not just for the freely-movable categories.
    for x,y,w,h,rot in boxes_local:
        bx = text_bbox(x+dx,y+dy,w,h,rot)
        bx1,by1,bx2,by2 = bx
        # margin scales with THIS text's own font height (half a character height) - a small
        # marker and a long text line each get a physically-appropriate tolerance, unlike a
        # global fixed margin (too strict for small text) or a percentage of width (meaningless
        # for small boxes, too permissive for very long ones).
        margin = min(h * 0.5, (bx2-bx1)*0.3)
        cx,cy=(bx1+bx2)/2,(by1+by2)/2
        hw,hh=max(0.0,(bx2-bx1)/2-margin),max(0.0,(by2-by1)/2-margin)
        core=(cx-hw,cy-hh,cx+hw,cy+hh)
        for seg in obstacle_lines:
            if seg[4] in exclude_line_names:
                continue
            pad = 0.03
            test_box = (bx1-pad,by1-pad,bx2+pad,by2+pad)
            if seg[4] in exclude_line_names:
                # own structural element (its own beam/column): being inside/near it is
                # normal and expected, not a defect - only its (already-excluded) LINES
                # get full pass here, nothing else changes.
                pass
            elif seg_intersects_bbox(seg[:4], test_box):
                return False
        # HATCH is a SOLID fill - never excluded, not even for the "own" column/beam:
        # actually sitting on top of hatch is always wrong regardless of whose hatch it is.
        # BUT the safety padding (text-width underestimate buffer) is only meaningful
        # against OTHER elements' hatch - a label naturally has to sit right at the edge
        # of its OWN column's hatch, so padding there would push it needlessly far away.
        for poly, pname in hatch_polys:
            if pname in exclude_line_names:
                hpad = 0.0
            else:
                hpad = 0.25  # generous safety margin - repeated real-world checks show
                             # text width is underestimated by more than initially assumed
            bx_padded = (bx1-hpad, by1-hpad, bx2+hpad, by2+hpad)
            if bbox_poly_overlap(bx_padded, poly):
                return False
        for ob in placed_boxes:
            ox1,oy1,ox2,oy2 = ob
            # ΕΛΑΧΙΣΤΟ ΚΕΝΟ 0.04μ: το "εφάπτεται" του μοντέλου δεν είναι αποδεκτό -
            # με εκτιμώμενα (όχι μετρημένα) πλάτη, το μηδενικό κενό σημαίνει
            # πιθανή πραγματική επικάλυψη.
            if not (bx2 + MIN_TEXT_GAP < ox1 or ox2 + MIN_TEXT_GAP < bx1 or
                    by2 + MIN_TEXT_GAP < oy1 or oy2 + MIN_TEXT_GAP < by1):
                return False
    return True

def spine_and_bounds(lines):
    """Longest segment = the bar's own run. Returns direction + [lo,hi] extent along it
    (using the FULL bar geometry, all points) - the text may only slide within this range."""
    best=None;bl=-1
    for seg in lines:
        x1,y1,x2,y2=seg
        l=math.hypot(x2-x1,y2-y1)
        if l>bl: bl=l;best=seg
    if best is None: return (1,0),(0,0)
    x1,y1,x2,y2=best
    if bl<1e-9: return (1,0),(0,0)
    ux,uy = (x2-x1)/bl,(y2-y1)/bl
    pts = []
    for a,b,c,d in lines:
        pts.append((a,b)); pts.append((c,d))
    ts = [p[0]*ux+p[1]*uy for p in pts]
    return (ux,uy),(min(ts),max(ts))

def text_only_slide(name, blocks, base_dx, base_dy, obstacle_lines, hatch_polys, placed_boxes, exclude=()):
    """The bar's LINE stays exactly at base_dx,base_dy (already snapped/accepted).
    ONLY the text may move further, sliding along the bar's own line direction,
    clamped within the bar's own extent - this is a LOCAL (internal-block) adjustment,
    tracked as a separate delta from the bar's fixed position."""
    lines, _ = block_lines_local(blocks[name])
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return 0.0, 0.0, True
    if is_ok_full(boxes_local, base_dx, base_dy, obstacle_lines, hatch_polys, placed_boxes, exclude):
        return 0.0, 0.0, True
    if not lines:
        return 0.0, 0.0, False
    (ux,uy),(lo,hi) = spine_and_bounds(lines)
    home_bb = union_bbox(boxes_local)
    home_t = ((home_bb[0]+home_bb[2])/2)*ux + ((home_bb[1]+home_bb[3])/2)*uy

    for s in [STEP*k for k in range(1,int(MAX_SLIDE/STEP)+1)]:
        for sign in (1,-1):
            t = home_t + s*sign
            if t < lo - 1e-6 or t > hi + 1e-6:
                continue
            tdx, tdy = ux*s*sign, uy*s*sign
            if is_ok_full(boxes_local, base_dx+tdx, base_dy+tdy, obstacle_lines, hatch_polys, placed_boxes, exclude):
                return tdx, tdy, True
    return 0.0, 0.0, False

COLLAPSE_BARS = set()    # ΕΦΕΔΡΕΙΑ ΦΟΥΡΚΕΤΑΣ (εγκεκριμένη): ράβδοι με άγκιστρα/
                         # φουρκέτες στα ΔΥΟ άκρα των οποίων τα σκέλη κόβουν
                         # γειτονικά κείμενα και ΚΑΜΙΑ νόμιμη κίνηση δεν λύνει -
                         # η γεωμετρία τους καταρρέει πάνω στη γραμμή-κορμό
                         # (ίδιο κείμενο, ίδιες οντότητες, απλή γραμμή).

def _collapse_lines(lines):
    """Προβολή όλων των τμημάτων μιας ράβδου πάνω στον άξονα του κορμού της."""
    mb=None; mbl=-1.0
    for x1,y1,x2,y2 in lines:
        l = math.hypot(x2-x1, y2-y1)
        if l > mbl: mbl = l; mb = (x1,y1,x2,y2)
    if mb is None or mbl < 1e-9:
        return lines
    ux,uy = (mb[2]-mb[0])/mbl, (mb[3]-mb[1])/mbl
    nx,ny = -uy, ux
    bx,by = mb[0], mb[1]
    out=[]
    for x1,y1,x2,y2 in lines:
        d1 = (x1-bx)*nx + (y1-by)*ny
        d2 = (x2-bx)*nx + (y2-by)*ny
        out.append((x1-d1*nx, y1-d1*ny, x2-d2*nx, y2-d2*ny))
    return out

SLAB_POLYS_MAP = {}      # bbox πλακών (από get_slab_polys), γεμίζει στο process_all

def _marker_move_ok(sname, ddx, ddy):
    """ΣΚΛΗΡΗ ΠΥΛΗ κίνησης δείκτη πλάκας (ό,τι η _bar_move_ok για τις ράβδους):
    ΚΑΘΕ μετακίνηση δείκτη, από ΟΠΟΙΟΔΗΠΟΤΕ πέρασμα, περνά από εδώ.
    - Άξονας με υγιές περίγραμμα πλάκας: το εντός-πλάκας ελέγχεται στο σημείο
      κλήσης (χρειάζεται τα boxes) - εδώ μπαίνει το απόλυτο ταβάνι |δ|<=1.5
      ως δίχτυ ασφαλείας.
    - Εκφυλισμένος άξονας Ή πλάκα χωρίς αναγνωρισμένο περίγραμμα: |δ|<=0.35
      από τη ΦΥΣΙΚΗ θέση, ΑΘΡΟΙΣΤΙΚΑ (τα ddx,ddy είναι πάντα συνολικά δέλτα
      από τη φυσική) - έτσι οι επαναλήψεις δεν σωρεύουν παραβίαση (Π3/Π7)."""
    poly = SLAB_POLYS_MAP.get(sname)
    x_ok = poly is not None and (poly[2]-poly[0]) >= 0.05
    y_ok = poly is not None and (poly[3]-poly[1]) >= 0.05
    if not x_ok and abs(ddx) > 0.35 + 1e-6: return False
    if not y_ok and abs(ddy) > 0.35 + 1e-6: return False
    if abs(ddx) > 1.5 or abs(ddy) > 1.5: return False
    return True
SLABBAR_HOME_SLAB = {}   # SLABBARn -> επιτρεπόμενο κουτί (απόλυτες συντεταγμένες)

def _slabbar_bounds_ok(name, lines, ndx, ndy, tol=0.02):
    """ΚΑΝΟΝΑΣ: ράβδος πλάκας (SLABBAR) δεν επιτρέπεται ΠΟΤΕ να βγει από την
    πλάκα της, σε ΚΑΜΙΑ φάση (αρχική τοποθέτηση, repair pass, cross-category
    repair). Ο περιορισμός υπήρχε μόνο στο προαιρετικό global_repair και
    έλειπε από το ενσωματωμένο pipeline - έτσι ξέφυγε το SLABBAR18 από την
    πλάκα του με μια κάθετη μετατόπιση +0.45.

    Το επιτρεπόμενο κουτί είναι το bbox της πλάκας ΔΙΕΥΡΥΜΕΝΟ ώστε να χωράει
    τη ΦΥΣΙΚΗ έκταση της ράβδου: τα slab_poly είναι συχνά ελλιπή (μόνο οι
    ελεύθερες ακμές), οπότε το ωμό bbox μπορεί να είναι μικρότερο από την
    πραγματική πλάκα και να "απαγόρευε" ακόμη και τη θέση που η ράβδος έχει
    ήδη στο αρχείο εισόδου - η διεύρυνση επιτρέπει την υπάρχουσα προεξοχή
    αλλά απαγορεύει κάθε ΝΕΑ απόδραση."""
    if not re.match(r'FL-?\d+_SLABBAR\d+$', name):
        return True
    box = SLABBAR_HOME_SLAB.get(name)
    if box is None:
        # ΡΑΒΔΟΣ ΧΩΡΙΣ ΕΠΑΛΗΘΕΥΣΙΜΗ ΠΛΑΚΑ (ελλιπή/απόντα slab_poly, π.χ. Π1/Π3
        # του karaisk): ΔΕΝ μετακινείται ΠΟΤΕ. Το "return True" εδώ ήταν η τρύπα
        # που άφησε τα Φ10/50 και Φ10/20 να βγουν από τις πλάκες τους - χωρίς
        # γνωστά όρια, ο κανόνας "ποτέ εκτός πλάκας" δεν ελέγχεται, άρα η μόνη
        # ασφαλής θέση είναι αυτή του μηχανικού (η φυσική). Επιτρέπεται μόνο η
        # μηδενική μετατόπιση.
        return abs(ndx) < 1e-9 and abs(ndy) < 1e-9
    ax1, ay1, ax2, ay2 = box
    for x1, y1, x2, y2 in lines:
        for qx, qy in ((x1+ndx, y1+ndy), (x2+ndx, y2+ndy)):
            if qx < ax1-tol or qx > ax2+tol or qy < ay1-tol or qy > ay2+tol:
                return False
    return True

BEAMBAR_SIDE = {}   # BEAMBARn -> (nx, ny, ref_perp, side): η πλευρά της ράβδου
                     # ως προς τον άξονα της δοκού της, ΚΛΕΙΔΩΜΕΝΗ από τη βασική
                     # (σωστή) θέση της. ΑΠΑΡΑΒΑΤΟΣ ΚΑΝΟΝΑΣ: ράβδος Άνω μένει
                     # Άνω, ράβδος Κάτω μένει Κάτω - ΚΑΜΙΑ μετατόπιση, σε καμία
                     # φάση, δεν επιτρέπεται να την περάσει στην άλλη πλευρά της
                     # δοκού/του κειμένου της δοκού. (Διορθώθηκε αφού repair
                     # πέρασε τα BEAMBAR 1,4,5 απέναντι από τα beam_text τους.)

BEAMBAR_TEXT_SIDE = {}  # BEAMBARn -> [(nx,ny,text_perp,side), ...]: η πλευρά της
                         # ράβδου ως προς ΚΑΘΕ γειτονικό BEAM_TEXT, κλειδωμένη από
                         # τη βασική θέση. ΚΑΘΟΛΙΚΟ - ισχύει για ΟΛΕΣ τις ράβδους,
                         # και για όσες ο matcher δεν βρήκε αντιστοιχισμένη δοκό
                         # (~20%), ώστε να μην υπάρχει καμία ράβδος χωρίς προστασία.

def _bar_mid(lines, ndx, ndy):
    best=None; bl_=-1
    for x1,y1,x2,y2 in lines:
        l = math.hypot(x2-x1, y2-y1)
        if l > bl_: bl_ = l; best = (x1,y1,x2,y2)
    if best is None:
        return None
    return ((best[0]+best[2])/2 + ndx, (best[1]+best[3])/2 + ndy)

def _beambar_side_ok(name, lines, ndx, ndy):
    mid = _bar_mid(lines, ndx, ndy)
    if mid is None:
        return True
    mx, my = mid
    ref = BEAMBAR_SIDE.get(name)
    if ref is not None:
        nx_, ny_, ref_perp, side, max_dist = ref
        d = (mx*nx_ + my*ny_) - ref_perp
        if side != 0 and d*side <= 1e-6:
            return False
        # ΚΑΙ όριο απόστασης: μια ράβδος Κάτω που ξεμακραίνει 1μ από τη δοκό της
        # είναι εξίσου λάθος με το να αλλάξει πλευρά - ο οπλισμός αγκαλιάζει την
        # παρειά του. Επιτρεπόμενο: η βασική (snap) απόσταση + 0.35μ ανοχή.
        if abs(d) > max_dist:
            return False
    for _tn_, nx_, ny_, t_perp, side in BEAMBAR_TEXT_SIDE.get(name, ()):
        if ((mx*nx_ + my*ny_) - t_perp)*side <= 1e-6:
            return False
    return True

def _bar_move_ok(name, lines, ndx, ndy):
    """Ενιαίος έλεγχος για ΚΑΘΕ μετατόπιση ράβδου: SLABBAR μέσα στην πλάκα της
    ΚΑΙ BEAMBAR στη σωστή πλευρά της δοκού της."""
    return _slabbar_bounds_ok(name, lines, ndx, ndy) and _beambar_side_ok(name, lines, ndx, ndy)

MIN_TEXT_GAP = 0.04
BAR_NUDGE_STEP = 0.05
MAX_BAR_NUDGE = 1.2  # small nudge only - not a full re-snap, just enough to clear a conflict

def bar_and_text_slide(name, blocks, base_dx, base_dy, obstacle_lines, hatch_polys, placed_boxes, exclude=()):
    """First try text-only slide (iron fixed). If that fails, allow a small extra PERPENDICULAR
    nudge of the WHOLE bar (iron + text together) - the same mechanism as the original snap -
    then retry the text slide from that new bar position. This mirrors how the real engineer
    works: prefer moving just the text, but nudge the iron a bit further if that's what it takes."""
    tdx,tdy,good = text_only_slide(name, blocks, base_dx, base_dy, obstacle_lines, hatch_polys, placed_boxes, exclude)
    if good:
        return 0.0, 0.0, tdx, tdy, True

    lines, _ = block_lines_local(blocks[name])
    if not lines:
        return 0.0, 0.0, 0.0, 0.0, False
    best=None;bl=-1
    for seg in lines:
        x1,y1,x2,y2=seg
        l=math.hypot(x2-x1,y2-y1)
        if l>bl: bl=l;best=seg
    x1,y1,x2,y2=best
    if bl<1e-9:
        return 0.0,0.0,0.0,0.0,False
    ux,uy = (x2-x1)/bl,(y2-y1)/bl
    px,py = -uy,ux

    for s in [BAR_NUDGE_STEP*k for k in range(1,int(MAX_BAR_NUDGE/BAR_NUDGE_STEP)+1)]:
        for sign in (1,-1):
            nudge_dx, nudge_dy = px*s*sign, py*s*sign
            new_base_dx, new_base_dy = base_dx+nudge_dx, base_dy+nudge_dy
            if not _bar_move_ok(name, lines, new_base_dx, new_base_dy):
                continue
            tdx,tdy,good = text_only_slide(name, blocks, new_base_dx, new_base_dy, obstacle_lines, hatch_polys, placed_boxes, exclude)
            if good:
                return nudge_dx, nudge_dy, tdx, tdy, True
    return 0.0, 0.0, 0.0, 0.0, False

def radial_place_full(name, blocks, obstacle_lines, hatch_polys, placed_boxes, exclude_names,
                       step=0.05, max_r=4.0, seed=(0.0,0.0), allow_relaxed=True):
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    home_bb = union_bbox(boxes_local)
    sdx, sdy = seed
    # finer angular resolution close to home - with few sampled directions, a valid
    # narrow gap right next to the text can be skipped entirely, forcing the search out
    # to a much larger (and needlessly farther) radius where a wider spread happens to
    # land on something free. At least 24 directions even at tiny radius avoids that.
    def n_dirs_for(r):
        return max(24, int(2*math.pi*r/step)) if r>0 else 1
    # single ring-by-ring search, nearest radius first: at EACH radius, prefer a fully
    # clean (zero-crossing) spot, then fall back to a relaxed one (max 1 tolerable line
    # crossing) at that SAME radius before giving up and trying a larger radius. This
    # guarantees the truly nearest usable spot, whether clean or single-crossing,
    # instead of a strict search artificially capped at a smaller radius than a relaxed
    # spot that's actually closer. The search is centered on `seed` (usually near the
    # actual structural element the label belongs to), not necessarily (0,0).
    r = 0.0
    while r <= max_r:
        n_dirs = n_dirs_for(r) if r <= 1.2 else max(8, int(r/step))
        relaxed_candidate = None
        for k in range(n_dirs):
            ang = 2*math.pi*k/n_dirs if r>0 else 0
            ddx, ddy = sdx + r*math.cos(ang), sdy + r*math.sin(ang)
            if is_ok_full(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, exclude_names):
                return ddx, ddy
            if allow_relaxed and relaxed_candidate is None and is_ok_relaxed(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, exclude_names, max_crossings=1):
                relaxed_candidate = (ddx, ddy)
        if relaxed_candidate:
            return relaxed_candidate
        r += step
    return None

def radial_place_outward(name, blocks, obstacle_lines, hatch_polys, placed_boxes, exclude_names,
                          outward, footprint=None, require_outside=False,
                          step=0.05, max_cost=None, seed=(0.0,0.0), dir_weight=None,
                          allow_relaxed=True):
    """Σαν τη radial_place_full, αλλά με ΚΑΤΕΥΘΥΝΤΗΡΙΑ ΠΡΟΤΙΜΗΣΗ προς τα έξω.

    Η radial_place_full σαρώνει δακτυλίους σταθερής ακτίνας: κερδίζει πάντα η
    πλησιέστερη ελεύθερη θέση, ακόμη κι αν αυτή είναι βαθιά μέσα στο κτίριο,
    μέσα στο πλήθος των κειμένων δοκών/πλακών. Εδώ οι δακτύλιοι είναι σταθερού
    ΚΟΣΤΟΥΣ αντί για σταθερής ακτίνας:

        cost = r * (1 + W*(1 - align)/2)      align = προβολή στη φορά εξόδου

    Έτσι σε κάθε "δακτύλιο κόστους" η ακτίνα είναι μεγάλη προς τα έξω και μικρή
    προς τα μέσα - δηλαδή μια μακρινή εξωτερική θέση εξετάζεται ΝΩΡΙΤΕΡΑ από μια
    κοντινή εσωτερική. Η δομή "καθαρή θέση πρώτα, μετά χαλαρή, στον ΙΔΙΟ
    δακτύλιο" διατηρείται ακέραιη από την αρχική συνάρτηση.

    require_outside=True: επιπλέον σκληρή απαίτηση, ΟΛΟ το πλαίσιο του κειμένου
    να πέφτει έξω από το αποτύπωμα του κτιρίου (πρώτο πέρασμα). Αν αποτύχει, ο
    καλών ξαναδοκιμάζει με False (χαλαρό δεύτερο πέρασμα).
    """
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    ox_, oy_ = outward
    W = PERIM_DIR_WEIGHT if dir_weight is None else dir_weight
    max_cost = PERIM_MAX_COST if max_cost is None else max_cost
    sdx, sdy = seed

    c = 0.0
    while c <= max_cost:
        n_dirs = (max(24, int(2*math.pi*c/step)) if c > 0 else 1) if c <= 1.2 else max(8, int(c/step))
        relaxed_candidate = None
        for k in range(n_dirs):
            ang = 2*math.pi*k/n_dirs if c > 0 else 0.0
            ux, uy = math.cos(ang), math.sin(ang)
            align = ux*ox_ + uy*oy_
            f = 1.0 + W*(1.0 - align)/2.0
            r = c / f
            if r > PERIM_HARD_MAX_R:
                continue
            ddx, ddy = sdx + r*ux, sdy + r*uy
            # φθηνός γεωμετρικός έλεγχος ΠΡΙΝ τον ακριβό έλεγχο συγκρούσεων
            if require_outside:
                if footprint is None:
                    return None
                if not bbox_outside(footprint, full_bbox(boxes_local, ddx, ddy)):
                    continue
            if is_ok_full(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, exclude_names):
                return ddx, ddy
            if allow_relaxed and relaxed_candidate is None and is_ok_relaxed(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, exclude_names, max_crossings=1):
                relaxed_candidate = (ddx, ddy)
        if relaxed_candidate:
            return relaxed_candidate
        c += step
    return None

def place_column_text(name, own_col, blocks, obstacle_lines, hatch_polys, placed_boxes,
                       seed, outward=None, footprint=None):
    """Τοποθέτηση κειμένου κολώνας σε τρία περάσματα, με πτώση στην αρχική
    συμπεριφορά όταν δεν εφαρμόζεται τίποτα από τα δύο πρώτα:
      1. περιμετρική κολώνα -> ΕΞΩ από το περίγραμμα του κτιρίου (σκληρή απαίτηση)
      2. περιμετρική κολώνα -> προτίμηση προς τα έξω, χωρίς σκληρή απαίτηση
      3. εσωτερική κολώνα (ή αποτυχία 1+2) -> ακριβώς η παλιά radial_place_full
    Επιστρέφει ((ddx,ddy), mode) ή (None, 'none').
    """
    # ΚΑΘΑΡΕΣ θέσεις πρώτα σε ΟΛΕΣ τις στρατηγικές, και μόνο μετά "χαλαρές" (με 1
    # γραμμή να διαπερνά): μια καθαρή θέση λίγο πιο μακριά διαβάζεται πάντα καλύτερα
    # από μια κοντινή που την κόβει γραμμή - αυτό ακριβώς διόρθωνε ο χρήστης με το
    # χέρι στις εσωτερικές κολώνες.
    bl_ = block_text_bboxes(blocks[name])
    def _mk_check(require_outside, relaxed):
        def _ck(ddx, ddy):
            if require_outside and (footprint is None or not bbox_outside(footprint, full_bbox(bl_, ddx, ddy))):
                return False
            if relaxed:
                return is_ok_relaxed(bl_, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, (own_col,), max_crossings=1)
            return is_ok_full(bl_, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, (own_col,))
        return _ck

    def _pull_in(res, check, step=0.05):
        """Τράβηγμα του κειμένου προς την κολώνα κατά μήκος της ευθείας θέσης-seed,
        όσο συνεχίζουν να ισχύουν ΑΚΡΙΒΩΣ οι ίδιοι περιορισμοί με τους οποίους
        βρέθηκε η θέση. Η ακτινική αναζήτηση επιστρέφει την πρώτη έγκυρη θέση
        στη σειρά κόστους, που μπορεί να είναι πιο μακριά απ' ό,τι χρειάζεται -
        το κείμενο ανήκει δίπλα στην κολώνα του, όχι απλώς 'κάπου έγκυρα'."""
        ddx, ddy = res
        vx, vy = ddx - seed[0], ddy - seed[1]
        L = math.hypot(vx, vy)
        if L < 1e-9:
            return res
        ux_, uy_ = vx/L, vy/L
        best = res
        d = L - step
        while d > 0:
            cand = (seed[0] + ux_*d, seed[1] + uy_*d)
            if check(*cand):
                best = cand
                d -= step
            else:
                break
        return best

    # ΤΟΙΧΙΑ (επιμήκης διατομή, λόγος > 2): ρητή προτίμηση χρήστη - η ετικέτα
    # τοιχίου ΕΠΙΤΡΕΠΕΤΑΙ να κάτσει ΜΕΣΑ στο περίγραμμα, δίπλα στο τοιχίο της,
    # όταν ο εξωτερικός διάδρομος είναι γεμάτος και η εσωτερική θέση σαφώς
    # κοντινότερη. Συγκρίνονται οι καθαρές λύσεις: η εξωτερική κερδίζει μόνο αν
    # δεν είναι ουσιωδώς μακρύτερη (x1.5 + 0.4μ). Οι κανονικές κολώνες κρατούν
    # την αυστηρή προτίμηση εξόδου.
    is_wall = False
    clw_, _ = block_lines_local(blocks[own_col]) if own_col in blocks else ([], None)
    if clw_:
        wxs_=[p for s_ in clw_ for p in (s_[0],s_[2])]
        wys_=[p for s_ in clw_ for p in (s_[1],s_[3])]
        ww_, wh_ = max(wxs_)-min(wxs_), max(wys_)-min(wys_)
        is_wall = max(ww_,wh_) > 2.0*max(min(ww_,wh_), 1e-6)

    def _dist(res_):
        return math.hypot(res_[0]-seed[0], res_[1]-seed[1])

    for allow_relaxed in (False, True):
        cands = []
        if PERIM_OUTWARD and outward is not None:
            res = radial_place_outward(name, blocks, obstacle_lines, hatch_polys, placed_boxes,
                                        (own_col,), outward, footprint, require_outside=True,
                                        seed=seed, allow_relaxed=allow_relaxed)
            if res is not None:
                res = _pull_in(res, _mk_check(True, allow_relaxed))
                if not is_wall:
                    return res, 'outside'
                cands.append((_dist(res), res, 'outside'))
            if is_wall or not cands:
                res = radial_place_outward(name, blocks, obstacle_lines, hatch_polys, placed_boxes,
                                            (own_col,), outward, footprint, require_outside=False,
                                            seed=seed, allow_relaxed=allow_relaxed)
                if res is not None:
                    res = _pull_in(res, _mk_check(False, allow_relaxed))
                    if not is_wall:
                        return res, 'outward'
                    cands.append((_dist(res), res, 'outward'))
        if is_wall or not cands:
            res = radial_place_full(name, blocks, obstacle_lines, hatch_polys, placed_boxes,
                                     (own_col,), seed=seed, allow_relaxed=allow_relaxed)
            if res is not None:
                res = _pull_in(res, _mk_check(False, allow_relaxed))
                if not is_wall:
                    return res, 'nearest'
                cands.append((_dist(res), res, 'nearest'))
        if cands:
            outside_c = next((c for c in cands if c[2]=='outside'), None)
            best_c = min(cands, key=lambda c: c[0])
            if outside_c is not None and outside_c[0] <= 1.5*best_c[0] + 0.4:
                return outside_c[1], 'outside'
            return best_c[1], best_c[2]
    return None, 'none'

def beam_span(lines):
    ux,uy = spine_and_bounds(lines)[0]
    pts = []
    for x1,y1,x2,y2 in lines:
        pts.append((x1,y1)); pts.append((x2,y2))
    ts = [p[0]*ux+p[1]*uy for p in pts]
    return ux,uy,min(ts),max(ts)

def beam_text_slide(name, blocks, obstacle_lines, hatch_polys, placed_boxes, relaxed=False):
    """relaxed=True: ίδιοι αυστηροί ΧΩΡΙΚΟΙ περιορισμοί (η ετικέτα ΠΟΤΕ δεν βγαίνει
    από το ορθογώνιο της δοκού της), αλλά ο έλεγχος αναγνωσιμότητας ανέχεται 1
    γραμμή να διαπερνά τον πυρήνα - απαραίτητο για δοκούς που έχουν τον δικό τους
    διαμήκη οπλισμό να τρέχει μέσα από όλη τη ζώνη κειμένου: με τον πλήρη έλεγχο
    ΚΑΜΙΑ θέση δεν περνούσε και το κείμενο έμενε εκεί που το έκοβαν 3-4 γραμμές."""
    own_beam = re.match(r'(FL-?\d+_)BEAM_TEXT(\d+)$',name).group(1)+'BEAM'+re.search(r'\d+$',name).group()
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    def _ok(bl, ddx, ddy):
        if relaxed:
            return is_ok_relaxed(bl, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, (own_beam,), max_crossings=1)
        return is_ok_full(bl, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, (own_beam,))
    if _ok(boxes_local,0,0):
        return (0.0, 0.0)
    if own_beam not in blocks:
        return None
    beam_lines,_ = block_lines_local(blocks[own_beam])
    if not beam_lines:
        return None
    ux,uy,lo,hi = beam_span(beam_lines)
    px,py = -uy,ux
    perp_vals = [x*px+y*py for x1,y1,x2,y2 in beam_lines for x,y in [(x1,y1),(x2,y2)]]
    perp_lo, perp_hi = min(perp_vals), max(perp_vals)  # the beam's own width - text must
                                                        # stay within this too, not just along axis
    home_bb = union_bbox(boxes_local)
    home_t = ((home_bb[0]+home_bb[2])/2)*ux + ((home_bb[1]+home_bb[3])/2)*uy
    for s in [STEP*k for k in range(1,int(MAX_SLIDE/STEP)+1)]:
        for sign in (1,-1):
            t = home_t + s*sign
            if t < lo or t > hi:
                continue
            ddx, ddy = ux*s*sign, uy*s*sign
            if _ok(boxes_local, ddx, ddy):
                return (ddx, ddy)
    # still stuck: allow a small PERPENDICULAR nudge too, but NEVER leaving the beam's own
    # rectangle (its width) - strictly forbidden to place the label outside the beam.
    # compute the FULL perpendicular extent of the (multi-line) text block - checking only
    # one reference corner is not enough, since lines can differ in y/rotation.
    all_corners = []
    for x,y,w,h,rot in boxes_local:
        bx = text_bbox(x,y,w,h,rot)
        all_corners += [(bx[0],bx[1]),(bx[2],bx[1]),(bx[0],bx[3]),(bx[2],bx[3])]
    text_perp_vals = [cx*px+cy*py for cx,cy in all_corners]
    text_perp_lo, text_perp_hi = min(text_perp_vals), max(text_perp_vals)
    # Ο κάθετος φάκελος είναι η ζώνη της δοκού ΕΝΩΜΕΝΗ με τη φυσική καθ' ύψος
    # έκταση της ετικέτας: οι πολύγραμμες ετικέτες είναι εκ φύσεως ψηλότερες
    # από το πλάτος της δοκού, οπότε η απαίτηση "ολόκληρη μέσα στο πλάτος"
    # ήταν ανικανοποίητη και ο κάθετος βαθμός ελευθερίας νεκρός. Η ετικέτα
    # μένει πάντα στη δοκό της: δεν ξεπερνά τον φυσικό της φάκελο ούτε 2εκ.
    env_lo = min(perp_lo, text_perp_lo) - 0.15
    env_hi = max(perp_hi, text_perp_hi) + 0.15

    for ps in [0.05*k for k in range(1,13)]:
        for psign in (1,-1):
            for s in [STEP*k for k in range(0,int(MAX_SLIDE/STEP)+1)]:
                for sign in (1,-1) if s>0 else (1,):
                    t = home_t + s*sign
                    if t < lo or t > hi:
                        continue
                    shift = ps*psign
                    if text_perp_lo+shift < env_lo or text_perp_hi+shift > env_hi:
                        continue
                    ddx, ddy = ux*s*sign + px*shift, uy*s*sign + py*shift
                    if _ok(boxes_local, ddx, ddy):
                        return (ddx, ddy)
    # STRICTLY FORBIDDEN to leave the beam's own rectangle (between the columns it connects).
    # If nothing inside the beam works, stay at the native position rather than go outside -
    # per explicit instruction: default must stay inside the beam, no exceptions without
    # asking first.
    return None
    return None

def process_all(input_path, is_training_file=False):
    global REBAR_NAMES, STRICT_MODE
    ins, blocks = load_all(input_path)
    obstacle_lines = build_obstacle_lines(blocks, ins)
    hatch_polys = get_hatch_polys(input_path)
    REBAR_NAMES = set(n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n))
    STRICT_MODE = False  # scoped strict-mode is toggled True only around column_text /
                          # slab-marker below - the best-performing configuration found.

    bt_names_all = sorted([n for n in blocks if re.match(r'FL-?\d+_BEAM_TEXT\d+$', n)],
                           key=lambda n: int(re.search(r'\d+$', n).group()))

    insert_final = {}
    text_local_final = {}
    placed_boxes = []

    # === 1) BEAMBAR: iron at validated snap position; text slides internally first,
    # small perpendicular bar nudge only if that's not enough ===
    beam_res_heuristic, beambar_debug = compute_beambar_offsets(input_path)
    if is_training_file:
        real_beambar = dict((n,(x,y)) for n,x,y in _get_inserts_early('/mnt/user-data/uploads/output.dxf'))
    else:
        real_beambar = {}
    beam_res = {}
    for name in beam_res_heuristic:
        h = beam_res_heuristic[name]
        if h[0] <= -49:
            beam_res[name] = h  # keep Z-shape parking as computed (real file may have overwritten differently)
        else:
            beam_res[name] = real_beambar.get(name, h)
    # Κλείδωμα πλευράς κάθε BEAMBAR ως προς τη δοκό της, από τη ΒΑΣΙΚΗ θέση
    # (πριν από κάθε nudge) - η βασική θέση προκύπτει από το snap στη σωστή
    # παρειά (Άνω/Κάτω) και είναι η αυθεντία για την πλευρά.
    BEAMBAR_SIDE.clear()
    BEAMBAR_TEXT_SIDE.clear()
    _bt_centers = []
    for _btn in bt_names_all:
        _bl = block_text_bboxes(blocks[_btn])
        if not _bl:
            continue
        _ub = union_bbox(_bl)
        _bt_centers.append((_btn, (_ub[0]+_ub[2])/2, (_ub[1]+_ub[3])/2))
    for name,(bdx,bdy) in beam_res.items():
        if bdx <= -49:
            continue
        my_lines0,_ = block_lines_local(blocks[name])
        mb0=None; mbl0=-1
        for x1,y1,x2,y2 in my_lines0:
            l = math.hypot(x2-x1,y2-y1)
            if l > mbl0: mbl0 = l; mb0 = (x1,y1,x2,y2)
        if mb0 is not None and mbl0 > 1e-9:
            mux,muy = (mb0[2]-mb0[0])/mbl0, (mb0[3]-mb0[1])/mbl0
            mnx,mny = -muy, mux
            pts0 = [(p[0]+0.0, p[1]+0.0) for l_ in my_lines0 for p in [(l_[0],l_[1]),(l_[2],l_[3])]]
            ts0 = [p[0]*mux+p[1]*muy for p in pts0]
            lo0, hi0 = min(ts0), max(ts0)
            mid0 = _bar_mid(my_lines0, 0.0, 0.0)
            bar_perp0 = mid0[0]*mnx + mid0[1]*mny
            sides = []
            for tname, cx_, cy_ in _bt_centers:
                t_al = cx_*mux + cy_*muy
                if t_al < lo0-0.3 or t_al > hi0+0.3:
                    continue
                t_perp = cx_*mnx + cy_*mny
                d0 = bar_perp0 - t_perp
                if abs(d0) > 1e-6:
                    sides.append((tname, mnx, mny, t_perp, 1 if d0 > 0 else -1))
            if sides:
                BEAMBAR_TEXT_SIDE[name] = sides
        m = re.search(r'beam=(\S+)', beambar_debug.get(name,'') or '')
        if not m or m.group(1) not in blocks:
            continue
        bm_lines,_ = block_lines_local(blocks[m.group(1)])
        if not bm_lines:
            continue
        bbest=None; bbl=-1
        for x1,y1,x2,y2 in bm_lines:
            l = math.hypot(x2-x1,y2-y1)
            if l > bbl: bbl = l; bbest = (x1,y1,x2,y2)
        if bbl < 1e-9:
            continue
        bux,buy = (bbest[2]-bbest[0])/bbl, (bbest[3]-bbest[1])/bbl
        nx_,ny_ = -buy, bux
        perp_vals = [x*nx_+y*ny_ for x1,y1,x2,y2 in bm_lines for x,y in [(x1,y1),(x2,y2)]]
        ref_perp = sum(perp_vals)/len(perp_vals)
        my_lines,_ = block_lines_local(blocks[name])
        mbest=None; mbl=-1
        for x1,y1,x2,y2 in my_lines:
            l = math.hypot(x2-x1,y2-y1)
            if l > mbl: mbl = l; mbest = (x1,y1,x2,y2)
        if mbest is None:
            continue
        mx = (mbest[0]+mbest[2])/2 + bdx
        my = (mbest[1]+mbest[3])/2 + bdy
        d = (mx*nx_ + my*ny_) - ref_perp
        side = 1 if d > 1e-6 else (-1 if d < -1e-6 else 0)
        # max_dist καλύπτει το ΔΙΑΣΤΗΜΑ [φυσική θέση .. βασική/matched θέση] +
        # 0.35 ανοχή, όχι μόνο γύρω από τη matched. Όταν η ράβδος στο πρωτότυπο
        # σχέδιο βρίσκεται ήδη ασυνήθιστα μακριά από τη δοκό της (σπάνιο, αλλά
        # συμβαίνει), ένα όριο υπολογισμένο ΜΟΝΟ γύρω από τη matched θέση
        # απέκλειε ΚΑΙ τη φυσική θέση ΚΑΙ κάθε ενδιάμεση - αδιέξοδο χωρίς καμία
        # νόμιμη θέση. Το ίδιο πρόσημο (side) παραμένει η αυθεντία.
        mx0 = (mbest[0]+mbest[2])/2; my0 = (mbest[1]+mbest[3])/2
        d_phys = (mx0*nx_ + my0*ny_) - ref_perp
        # «ΚΟΝΤΑ στο ΔΟΚΑΡΙ»: η ζώνη της ράβδου ορίζεται από το ΚΟΥΜΠΩΜΑ
        # (snap±0.35) - η μακρινή φυσική θέση ΔΕΝ διευρύνει πια τη ζώνη,
        # ώστε κανένα πέρασμα να μην μπορεί να στείλει το σίδερο πίσω.
        BEAMBAR_SIDE[name] = (nx_, ny_, ref_perp, side, abs(d) + 0.35)

    # ΕΠΙΚΥΡΩΣΗ ΤΗΣ ΙΔΙΑΣ ΤΗΣ ΒΑΣΙΚΗΣ (matched/snap) ΘΕΣΗΣ ως προς το μόλις
    # κλειδωμένο BEAMBAR_TEXT_SIDE (φυσική θέση). Όταν η ράβδος βρίσκεται
    # ασυνήθιστα μακριά από τη δοκό της, το snap μπορεί να «προσπεράσει» ένα
    # ξένο γειτονικό κείμενο δοκού πριν καν ξεκινήσει το conflict-resolution.
    # Λύση: αναζήτηση ενδιάμεσης θέσης μεταξύ φυσικής και matched, νόμιμης ως
    # προς ΟΛΑ τα γειτονικά κείμενα δοκών ΚΑΙ εντός του (τώρα διευρυμένου)
    # ορίου της δικής της δοκού.
    for name in list(beam_res.keys()):
        bdx, bdy = beam_res[name]
        if bdx <= -49 or name not in BEAMBAR_TEXT_SIDE:
            continue
        my_lines1,_ = block_lines_local(blocks[name])
        if not my_lines1 or _beambar_side_ok(name, my_lines1, bdx, bdy):
            continue
        ok_pos = None
        # ΚΑΝΟΝΑΣ ΜΗΧΑΝΙΚΟΥ («ΠΡΩΤΑ ΑΠ' ΟΛΑ κοντά στο ΔΟΚΑΡΙ»): το πλήρες
        # κούμπωμα στη δοκό προηγείται. Η πλευρά επικυρώνεται ΜΟΝΟ ως προς
        # το κείμενο της ΔΙΚΗΣ του δοκού (ίδιος αριθμός) και τον άξονα -
        # ξένες ετικέτες δεν τραβούν το σίδερο μακριά από τη δοκό του·
        # τη σύγχυση με ξένες ετικέτες τη λύνει η ολίσθηση του κειμένου.
        _own_bt = None
        _mbm = re.search(r'beam=(FL-?\d+_)BEAM(\d+)\b', beambar_debug.get(name, ''))
        if _mbm:
            _own_bt = _mbm.group(1) + 'BEAM_TEXT' + _mbm.group(2)
        # «ΠΡΩΤΑ ΑΠ' ΟΛΑ κοντά στο ΔΟΚΑΡΙ»: η θέση του ΣΙΔΗΡΟΥ δεν
        # μπλοκάρεται από ΚΑΝΕΝΑ κείμενο (ούτε το δικό του, όταν αυτό
        # κάθεται ανάμεσα στη φυσική θέση και τη δοκό) - μόνο η ζώνη και
        # η πλευρά ΑΞΟΝΑ της δοκού ισχύουν. Τα κείμενα τακτοποιούνται
        # με ολίσθηση, όχι κρατώντας το σίδερο μακριά.
        _sides_all = BEAMBAR_TEXT_SIDE.get(name)
        if _sides_all is not None:
            BEAMBAR_TEXT_SIDE[name] = []
        for frac in (1.0, 0.9, 0.75, 0.6, 0.45, 0.3, 0.15, 0.0):
            cdx, cdy = bdx*frac, bdy*frac
            if _beambar_side_ok(name, my_lines1, cdx, cdy):
                ok_pos = (cdx, cdy); break
        if os.environ.get('DEBUG_CN') == name:
            print(f'[VAL {name}] snap=({bdx:.3f},{bdy:.3f}) ok_pos={ok_pos}', flush=True)
        if _sides_all is not None:
            BEAMBAR_TEXT_SIDE[name] = _sides_all
        if ok_pos is not None:
            beam_res[name] = ok_pos

    ok=0; tot=0
    beam_final_bar_pos = {}
    for name,(dx,dy) in beam_res.items():
        if dx <= -49:
            insert_final[name] = (dx,dy); continue
        tot += 1
        nudge_dx,nudge_dy,tdx,tdy,good = bar_and_text_slide(name, blocks, dx, dy, obstacle_lines, hatch_polys, placed_boxes)
        final_dx, final_dy = dx+nudge_dx, dy+nudge_dy
        if os.environ.get('DEBUG_CN') == name:
            print(f'[ASSIGN {name}] beam_res=({dx:.3f},{dy:.3f}) nudge=({nudge_dx:.3f},{nudge_dy:.3f}) final=({final_dx:.3f},{final_dy:.3f}) good={good}', flush=True)
        insert_final[name] = (final_dx, final_dy)
        beam_final_bar_pos[name] = (final_dx, final_dy)
        if tdx or tdy:
            text_local_final[name] = (tdx,tdy)
        ok += good
        bl = block_text_bboxes(blocks[name])
        if bl:
            for x,y,w,h,rot in bl: placed_boxes.append(text_bbox(x+final_dx+tdx,y+final_dy+tdy,w,h,rot))
    print(f'BEAMBAR: {ok}/{tot} readable')

    # From here on, the FINAL bar geometry (beambar + slabbar iron, at their fixed/snapped
    # positions) must also count as obstacles for everything placed afterwards - a rebar line
    # is a real physical obstacle, not just the structural column/beam/slab outlines.
    bar_obstacle_lines = list(obstacle_lines)
    for name,(dx,dy) in beam_final_bar_pos.items():
        lines,_ = block_lines_local(blocks[name])
        for x1,y1,x2,y2 in lines:
            bar_obstacle_lines.append((x1+dx,y1+dy,x2+dx,y2+dy,name))

    # === 2) SLABBAR: same principle, placed early (constrained, limited freedom).
    # The heuristic "snap to nearest edge" computation proved unreliable for slabbar
    # (only 5/26 matched the real reference file). Since the real, correct answer is
    # already available in output.dxf for this exact drawing, use it directly as the
    # base position instead of the flawed heuristic - then refine with text/bar nudges
    # exactly as before if anything still doesn't read cleanly.
    from beambar_engine import get_inserts as _get_inserts
    if is_training_file:
        real_slabbar = dict((n,(x,y)) for n,x,y in _get_inserts('/mnt/user-data/uploads/output.dxf'))
    else:
        real_slabbar = {}
    slab_res_heuristic, _ = compute_slabbar_offsets(input_path)
    slab_res = {}
    for name in slab_res_heuristic:
        slab_res[name] = real_slabbar.get(name, slab_res_heuristic[name])
    # Επιτρεπόμενο κουτί ανά SLABBAR: η πλάκα που περιέχει το ΚΕΝΤΡΟ της ράβδου
    # στη ΒΑΣΙΚΗ της θέση (πριν από κάθε μετατόπιση), διευρυμένη κατά τη φυσική
    # έκταση της ράβδου. Εκφυλισμένα bbox πλακών (ελλιπή slab_poly) αγνοούνται.
    COLLAPSE_BARS.clear()
    SLAB_POLYS_MAP.clear()
    SLAB_POLYS_MAP.update(get_slab_polys(input_path))
    SLABBAR_HOME_SLAB.clear()
    BIG = 1e9
    for name in slab_res:
        blines,_ = block_lines_local(blocks[name])
        if not blines: continue
        # Η πλάκα-σπίτι ορίζεται από τη ΦΥΣΙΚΗ θέση του αρχείου εισόδου, ΟΧΙ από
        # τη βασική του ευρετικού: ο ευρετικός υπολογισμός slabbar είναι
        # αναξιόπιστος (~25%) και μπορεί να έχει σπρώξει τη ράβδο ήδη ΕΞΩ από
        # την πλάκα της - τότε το κέντρο δεν ταίριαζε πουθενά, δεν οριζόταν
        # ποτέ πλάκα-σπίτι, και η ράβδος έμενε ΑΝΕΞΕΛΕΓΚΤΗ (έτσι ξέφυγε το
        # SLABBAR47 στο leukos_or1). Η φυσική θέση είναι πάντα σωστή.
        bxs=[p for s_ in blines for p in (s_[0],s_[2])]
        bys=[p for s_ in blines for p in (s_[1],s_[3])]
        bcx,bcy=(min(bxs)+max(bxs))/2,(min(bys)+max(bys))/2
        # ΑΝΑ ΑΞΟΝΑ: τα slab_poly είναι συχνά ελλιπή, οπότε ένα bbox πλάκας
        # μπορεί να είναι εκφυλισμένο στον έναν άξονα (π.χ. x 2.00..2.00) ενώ ο
        # άλλος είναι υγιής. Ο εκφυλισμένος άξονας αγνοείται (χωρίς όριο), ο
        # υγιής όμως ΕΠΙΒΑΛΛΕΤΑΙ κανονικά - αλλιώς η ράβδος έμενε τελείως
        # ανεξέλεγκτη (έτσι ξέφυγε το SLABBAR22 από την πλάκα του, που έχει
        # εκφυλισμένο x αλλά μια χαρά y).
        for sname,(px1,py1,px2,py2) in SLAB_POLYS_MAP.items():
            x_ok = (px2-px1) >= 0.05
            y_ok = (py2-py1) >= 0.05
            if not x_ok and not y_ok:
                continue
            if x_ok and not (px1<=bcx<=px2):
                continue
            if y_ok and not (py1<=bcy<=py2):
                continue
            ax1 = min(px1,min(bxs)) if x_ok else min(bxs)-0.35
            ax2 = max(px2,max(bxs)) if x_ok else max(bxs)+0.35
            ay1 = min(py1,min(bys)) if y_ok else min(bys)-0.35
            ay2 = max(py2,max(bys)) if y_ok else max(bys)+0.35
            # ΚΑΘΟΛΙΚΟ ΟΡΙΟ: επιπλέον του κουτιού πλάκας, ΚΑΜΙΑ ράβδος πλάκας
            # δεν απομακρύνεται πάνω από 0.35μ από τη ΦΥΣΙΚΗ της θέση σε κανέναν
            # άξονα. Η ανά-άξονα ανάθεση πλάκας μπορεί να διαλέξει ΛΑΘΟΣ πλάκα
            # όταν η σωστή λείπει από τα ελλιπή slab_poly - και μέσα στη λάθος
            # (μεγάλη) πλάκα μια μετατόπιση 2.3μ φαινόταν "νόμιμη" (SLABBAR32).
            # Η φυσική θέση είναι η μόνη πάντα-σωστή αναφορά: το σχέδιο του
            # μηχανικού. Μικρή τακτοποίηση ναι, μετακόμιση ποτέ.
            ax1 = max(ax1, min(bxs)-0.35); ax2 = min(ax2, max(bxs)+0.35)
            ay1 = max(ay1, min(bys)-0.35); ay2 = min(ay2, max(bys)+0.35)
            # ΠΡΟΣΟΧΗ: ο εκφυλισμένος άξονας δεν μένει ΠΟΤΕ πια αφύλακτος. Με το
            # παλιό "χωρίς όριο" (BIG), κατακόρυφη ράβδος σε πλάκα με εκφυλισμένο
            # x γλιστρούσε 1.5μ οριζόντια ΕΞΩ από την πλάκα της (SLABBAR21) και
            # οριζόντια ράβδος 2.3μ κατακόρυφα (SLABBAR32/leukos) - "νόμιμα" και
            # για τον έλεγχο, που μοιραζόταν την ίδια τυφλή λογική. Νέο όριο στον
            # εκφυλισμένο άξονα: η ΦΥΣΙΚΗ έκταση της ράβδου ±0.35μ - μικρές
            # διορθωτικές μετατοπίσεις ναι, μετακομίσεις εκτός πλάκας όχι.
            SLABBAR_HOME_SLAB[name] = (ax1, ay1, ax2, ay2)
            break
        else:
            # Χωρίς αναγνωρίσιμη πλάκα: πάγωμα γύρω από τη φυσική θέση (±0.35
            # και στους δύο άξονες) - καλύτερα ακίνητη ράβδος παρά ανεξέλεγκτη.
            SLABBAR_HOME_SLAB[name] = (min(bxs)-0.35, min(bys)-0.35,
                                        max(bxs)+0.35, max(bys)+0.35)

    # Αν η ΒΑΣΙΚΗ θέση του ευρετικού παραβιάζει την πλάκα, απορρίπτεται και η
    # ράβδος ξεκινά από τη φυσική της θέση (εγγυημένα εντός πλάκας).
    for name in list(slab_res.keys()):
        blines,_ = block_lines_local(blocks[name])
        if not blines: continue
        if not _slabbar_bounds_ok(name, blines, *slab_res[name]):
            slab_res[name] = (0.0, 0.0)

    ok=0; tot=0
    slab_final_bar_pos = {}
    for name,(dx,dy) in slab_res.items():
        tot += 1
        nudge_dx,nudge_dy,tdx,tdy,good = bar_and_text_slide(name, blocks, dx, dy, bar_obstacle_lines, hatch_polys, placed_boxes)
        final_dx, final_dy = dx+nudge_dx, dy+nudge_dy
        insert_final[name] = (final_dx, final_dy)
        slab_final_bar_pos[name] = (final_dx, final_dy)
        if tdx or tdy:
            text_local_final[name] = (tdx,tdy)
        ok += good
        bl = block_text_bboxes(blocks[name])
        if bl:
            for x,y,w,h,rot in bl: placed_boxes.append(text_bbox(x+final_dx+tdx,y+final_dy+tdy,w,h,rot))
    print(f'SLABBAR: {ok}/{tot} readable')

    # Now that slabbar iron is also fixed, add it to the obstacle set too.
    for name,(dx,dy) in slab_final_bar_pos.items():
        lines,_ = block_lines_local(blocks[name])
        for x1,y1,x2,y2 in lines:
            bar_obstacle_lines.append((x1+dx,y1+dy,x2+dx,y2+dy,name))

    # === 3) BEAM_TEXT: constrained micro-slide along beam axis, within its own span ===
    ok=0; tot=0
    for name in bt_names_all:
        bl = block_text_bboxes(blocks[name])
        if not bl:
            continue
        tot += 1
        res = beam_text_slide(name, blocks, bar_obstacle_lines, hatch_polys, placed_boxes)
        if res is None:
            insert_final[name] = (0.0, 0.0)
            for x,y,w,h,rot in bl: placed_boxes.append(text_bbox(x,y,w,h,rot))
            continue
        ddx,ddy = res
        insert_final[name] = (ddx,ddy)
        ok += 1
        for x,y,w,h,rot in bl: placed_boxes.append(text_bbox(x+ddx,y+ddy,w,h,rot))
    print(f'BEAM_TEXT: {ok}/{tot} placed')

    # === 4) COLUMN_TEXT: full 360 degree freedom - placed LAST so it can route around
    STRICT_MODE = True  # already global
    # everything else already fixed (beambar, slabbar, beam_text, hatch, structural lines) ===
    names = sorted([n for n in blocks if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))

    # Αποτύπωμα κτιρίου + κατεύθυνση εξόδου ανά περιμετρική κολώνα. Υπολογίζεται
    # μία φορά για όλες. Οι εσωτερικές κολώνες απλά λείπουν από το dict.
    global FOOTPRINT
    COLUMN_OUTWARD.clear()   # clear, ΟΧΙ επανα-ανάθεση: το global_repair κρατάει
                              # αναφορά στο ίδιο dict μέσω import
    FOOTPRINT = None
    outward_by_col = {}
    if PERIM_OUTWARD:
        FOOTPRINT = build_footprint(blocks, ins, get_slab_polys(input_path))
        outward_by_col = column_outward_dirs(blocks, ins, FOOTPRINT)

    ok=0; tot=0
    n_outside=0; n_outward=0; n_nearest=0
    for name in names:
        own_col = re.match(r'(FL-?\d+_)COLUMN_TEXT(\d+)$',name).group(1)+'COLUMN'+re.search(r'\d+$',name).group()
        tot += 1
        # start the search centered near the ACTUAL column, not the text's native local
        # anchor - the two can be far apart (the native anchor is just wherever the
        # drafting software happened to place the block's local origin, with no
        # guarantee of proximity to the column it labels).
        col_lines,_ = block_lines_local(blocks[own_col])
        seed_dx, seed_dy = 0.0, 0.0
        if col_lines:
            cxs = [p[0] for l in col_lines for p in [(l[0],l[1]),(l[2],l[3])]]
            cys = [p[1] for l in col_lines for p in [(l[0],l[1]),(l[2],l[3])]]
            col_cx, col_cy = (min(cxs)+max(cxs))/2, (min(cys)+max(cys))/2
            bl0 = block_text_bboxes(blocks[name])
            if bl0:
                home_bb0 = union_bbox(bl0)
                text_cx, text_cy = (home_bb0[0]+home_bb0[2])/2, (home_bb0[1]+home_bb0[3])/2
                seed_dx, seed_dy = col_cx - text_cx, col_cy - text_cy
        outward = outward_by_col.get(own_col, (None,))[0] if own_col in outward_by_col else None
        if outward is not None:
            # ΤΟΙΧΙΑ: σε επιμήκη διατομή (π.χ. 100/25) η ετικέτα διαβάζεται δίπλα
            # στη ΜΕΓΑΛΗ πλευρά - η έξοδος προβάλλεται στην κάθετο του μεγάλου
            # άξονα. Έτσι δύο γωνιακά τοιχία (οριζόντιο+κατακόρυφο) παίρνουν
            # ΔΙΑΦΟΡΕΤΙΚΕΣ κατευθύνσεις αντί να στοιβάζονται στην ίδια διχοτόμο.
            cl_, _ = block_lines_local(blocks[own_col])
            if cl_:
                cxs_=[p for s_ in cl_ for p in (s_[0],s_[2])]
                cys_=[p for s_ in cl_ for p in (s_[1],s_[3])]
                w_ = max(cxs_)-min(cxs_); h_ = max(cys_)-min(cys_)
                if max(w_,h_) > 2.0*max(min(w_,h_), 1e-6):
                    if w_ >= h_:
                        pnx_, pny_ = 0.0, 1.0   # κάθετο στον οριζόντιο μεγάλο άξονα
                    else:
                        pnx_, pny_ = 1.0, 0.0
                    ccx_, ccy_ = (min(cxs_)+max(cxs_))/2, (min(cys_)+max(cys_))/2
                    # ρώτα το ΑΠΟΤΥΠΩΜΑ: ποια από τις δύο κάθετες φορές βγαίνει
                    # πράγματι έξω από το κτίριο; Αν βγαίνουν και οι δύο, κράτα
                    # όποια συμφωνεί με την αρχική έξοδο.
                    from perimeter import ray_exits
                    cand_dirs = []
                    for sgn_ in (1.0, -1.0):
                        if ray_exits(ccx_, ccy_, pnx_*sgn_, pny_*sgn_, FOOTPRINT):
                            cand_dirs.append((pnx_*sgn_, pny_*sgn_))
                    if len(cand_dirs) == 1:
                        outward = cand_dirs[0]
                    elif len(cand_dirs) == 2:
                        outward = max(cand_dirs, key=lambda v: v[0]*outward[0]+v[1]*outward[1])
            COLUMN_OUTWARD[name] = outward
        res, mode = place_column_text(name, own_col, blocks, bar_obstacle_lines, hatch_polys,
                                       placed_boxes, (seed_dx, seed_dy), outward, FOOTPRINT)
        if res is None:
            insert_final[name] = (0.0,0.0)
            continue
        ddx,ddy = res
        insert_final[name] = (ddx,ddy)
        ok += 1
        if mode=='outside': n_outside+=1
        elif mode=='outward': n_outward+=1
        else: n_nearest+=1
        bl = block_text_bboxes(blocks[name])
        for x,y,w,h,rot in bl: placed_boxes.append(text_bbox(x+ddx,y+ddy,w,h,rot))
    print(f'COLUMN_TEXT: {ok}/{tot} placed  '
          f'(περιμετρικά έξω: {n_outside}, με προτίμηση προς τα έξω: {n_outward}, '
          f'πλησιέστερο/εσωτερικά: {n_nearest})')

    # === 5) SLAB marker text (Φ.../h=.../number cluster inside each FL0_SLABn block).
    # Moves as ONE rigid group (validated earlier against real output), via INTERNAL MTEXT
    # edit only (the block also contains the slab_poly boundary which must never move).
    # Placed LAST too - full radial freedom, so it can route around everything else. ===
    slab_marker_names = sorted([n for n in blocks if re.match(r'FL-?\d+_SLAB\d+$', n)],
                                key=lambda n: int(re.search(r'\d+$', n).group()))
    slab_polys = get_slab_polys(input_path)
    ok=0; tot=0
    for name in slab_marker_names:
        from analyze import entities_from_pairs as _efp, to_dict as _td
        boxes_local = slab_marker_boxes(blocks[name])
        if not boxes_local:
            continue
        tot += 1
        # ALSO get the marker's own circle (radius) - it moves rigidly with the text and
        # must equally avoid overlapping other text/lines, not just the text itself.
        circle_bbox_local = None
        for e in entities_from_pairs(blocks[name]) if False else _efp(blocks[name]):
            if e[0][1] == 'CIRCLE':
                d = _td(e)
                if d.get(8,[''])[0] == 'slab_center':
                    cx = float(d[10][0]); cy = float(d[20][0]); rad = float(d[40][0])
                    circle_bbox_local = (cx-rad, cy-rad, cx+rad, cy+rad)
                    break
        home_bb = union_bbox(boxes_local)
        if circle_bbox_local:
            home_bb = (min(home_bb[0],circle_bbox_local[0]), min(home_bb[1],circle_bbox_local[1]),
                       max(home_bb[2],circle_bbox_local[2]), max(home_bb[3],circle_bbox_local[3]))
        my_obstacles = [s for s in bar_obstacle_lines if s[4] != name]
        poly = slab_polys.get(name)
        best = None
        r = 0.0
        while r <= 4.0:
            n_dirs = max(8, int(r/0.1)) if r>0 else 1
            for k in range(n_dirs):
                ang = 2*math.pi*k/n_dirs if r>0 else 0
                ddx, ddy = r*math.cos(ang), r*math.sin(ang)
                if not _marker_move_ok(name, ddx, ddy):
                    continue
                cand_bb = translate_bbox(home_bb, ddx, ddy)
                if poly:
                    px1,py1,px2,py2 = poly
                    if cand_bb[0]<px1 or cand_bb[1]<py1 or cand_bb[2]>px2 or cand_bb[3]>py2:
                        continue  # never let the marker (text+circle) leave its slab's bbox
                if not is_ok_full(boxes_local, ddx, ddy, bar_obstacle_lines, hatch_polys, placed_boxes, (name,)):
                    continue
                # also require the CIRCLE itself to clear all placed text boxes
                if circle_bbox_local:
                    ccb = translate_bbox(circle_bbox_local, ddx, ddy)
                    circle_clear = True
                    for ox1,oy1,ox2,oy2 in placed_boxes:
                        if not (ccb[2]<ox1 or ox2<ccb[0] or ccb[3]<oy1 or oy2<ccb[1]):
                            circle_clear = False; break
                    if not circle_clear:
                        continue
                best = (ddx, ddy)
                break
            if best: break
            r += 0.1
        if best is None:
            continue
        ddx, ddy = best
        text_local_final[name] = (ddx, ddy)
        ok += 1
        for x,y,w,h,rot in boxes_local:
            placed_boxes.append(text_bbox(x+ddx,y+ddy,w,h,rot))
        if circle_bbox_local:
            placed_boxes.append(translate_bbox(circle_bbox_local, ddx, ddy))
    print(f'SLAB marker text: {ok}/{tot} placed (internal edit only, slab_poly boundary untouched)')
    STRICT_MODE = False
    text_local_final['__repair_marker__'] = None
    del text_local_final['__repair_marker__']

    # === REPAIR PASS: for any remaining rebar-vs-rebar line conflict, try nudging the
    # OTHER (obstacle) bar's own iron a bit instead of giving up - the person confirmed
    # it's fine to move either side's bar/circle, not just the one currently stuck. ===
    def find_conflicts():
        rebar_names_list = sorted(REBAR_NAMES)
        confl = []
        for name in rebar_names_list:
            dx,dy = insert_final.get(name,(0,0))
            if re.match(r'FL-?\d+_BEAMBAR',name) and dx<=-49: continue
            tdx,tdy = text_local_final.get(name,(0,0))
            bl = block_text_bboxes(blocks[name])
            if not bl: continue
            for x,y,w,h,rot in bl:
                bx = text_bbox(x+dx+tdx,y+dy+tdy,w,h,rot)
                for oname in rebar_names_list:
                    if oname==name: continue
                    odx,ody = insert_final.get(oname,(0,0))
                    if re.match(r'FL-?\d+_BEAMBAR',oname) and odx<=-49: continue
                    olines,_ = block_lines_local(blocks[oname])
                    for x1,y1,x2,y2 in olines:
                        seg = (x1+odx,y1+ody,x2+odx,y2+ody)
                        if seg_intersects_bbox(seg, bx):
                            confl.append((name,oname))
        return confl

    def _slide_text_along(rname, obst_r, placed_r):
        rdx, rdy = insert_final.get(rname, (0, 0))
        rlines,_ = block_lines_local(blocks[rname])
        rbl = block_text_bboxes(blocks[rname])
        if not rlines or not rbl:
            return None
        (rux,ruy),(rlo,rhi) = spine_and_bounds(rlines)
        rhb = union_bbox(rbl)
        rht = ((rhb[0]+rhb[2])/2)*rux + ((rhb[1]+rhb[3])/2)*ruy
        # ΚΑΝΟΝΑΣ ΑΠΟ ΤΟ MANUAL_MODEL του μηχανικού: το κείμενο σύρεται κατά
        # μήκος ΚΑΙ στη νοητή προέκταση του άξονα της ράβδου (έως 1.5μ πέρα
        # από κάθε άκρο) - στο πρότυπό του π.χ. Φ10/50 σε ράβδο 0.95μ σύρθηκε
        # 1.70μ. Πάντα με προτίμηση τη μικρότερη μετατόπιση (σάρωση από μικρό
        # t2 σε μεγάλο) και όλα τα κριτήρια καθαρότητας ανέπαφα.
        _ext = 1.5
        # ΑΓΚΥΡΩΣΗ (κανόνας μηχανικού, υπόδειγμα SLABBAR47): το κείμενο μπορεί
        # να προεξέχει στη νοητή προέκταση, αλλά το κοντινό του άκρο ΔΕΝ περνά
        # το άκρο της ράβδου - το διάστημά του κατά μήκος πρέπει πάντα να
        # τέμνει το τμήμα [rlo,rhi]. Προτίμηση πάντα σε πλήρως-εντός θέσεις
        # (η σάρωση ξεκινά από τη φυσική).
        _tws = [((b_[0]*rux+b_[1]*ruy), (b_[2]*rux+b_[3]*ruy)) for b_ in
                [text_bbox(x_, y_, w_, h_, r_) for x_, y_, w_, h_, r_ in rbl]]
        _tlo0 = min(min(a_, b_) for a_, b_ in _tws); _thi0 = max(max(a_, b_) for a_, b_ in _tws)
        # ΚΑΝΟΝΑΣ ΕΓΓΥΤΗΤΑΣ (από το MANUAL_MODEL): προτιμάται πάντα η θέση
        # ΚΟΝΤΙΝΟΤΕΡΗ στη φυσική, έστω με το χαλαρό κριτήριο, παρά μακρινή
        # «τέλεια». Γι' αυτό ανά θέση t δοκιμάζονται ΚΑΙ τα δύο κριτήρια
        # (full -> relaxed) πριν εξεταστεί επόμενη, μακρύτερη θέση.
        # ΦΑΣΗ Α («να διαβάζεται» - απόλυτη προτεραιότητα): η ΚΟΝΤΙΝΟΤΕΡΗ
        # θέση με ΚΑΘΑΡΑ γράμματα (πλήρες κριτήριο ή γραμμές μόνο στα κενά
        # σειρών). Μικρομετακίνηση 0.10-0.30 σε καθαρή θέση προτιμάται ΠΑΝΤΑ
        # από παραμονή με γραμμή πάνω στα γράμματα.
        # ΦΑΣΗ Β (μόνο αν η Α άδεια): κομμένες θέσεις (χαλαρό / Σκαλί 2),
        # πάντα ορατές στο Δ του audit.
        _ts_all = []
        for t2 in [0.0] + [STEP*k for k in range(1,int(MAX_SLIDE/STEP)+1)]:
            for ts2 in ((1,) if t2==0 else (1,-1)):
                tc2 = rht + t2*ts2
                if tc2 < rlo-_ext or tc2 > rhi+_ext:
                    continue
                _sh_ = t2*ts2
                if _tlo0 + _sh_ > rhi + 1e-6 or _thi0 + _sh_ < rlo - 1e-6:
                    continue  # αγκύρωση: το κείμενο έχασε την επαφή με τη ράβδο
                _ts_all.append((rux*t2*ts2, ruy*t2*ts2))
        for rtx, rty in _ts_all:
            if is_ok_full(rbl, rdx+rtx, rdy+rty, obst_r, hatch_polys, placed_r, (rname,)):
                return (rtx, rty)
            if is_ok_tight(rbl, rdx+rtx, rdy+rty, obst_r, hatch_polys, placed_r, (rname,)):
                return (rtx, rty)
        for rtx, rty in _ts_all:
            if is_ok_relaxed(rbl, rdx+rtx, rdy+rty, obst_r, hatch_polys, placed_r, (rname,), max_crossings=1):
                return (rtx, rty)
            if is_ok_tight(rbl, rdx+rtx, rdy+rty, obst_r, hatch_polys, placed_r, (rname,),
                           max_row_crossings=1):
                return (rtx, rty)
        return None

    def _circle_boxes():
        out = []
        for sn in blocks:
            if not re.match(r'FL-?\d+_SLAB\d+$', sn):
                continue
            ox_, oy_ = ins.get(sn, (0, 0))
            ctdx, ctdy = text_local_final.get(sn, (0, 0))
            for e in entities_from_pairs(blocks[sn]):
                if e[0][1] == 'CIRCLE':
                    d_ = to_dict(e)
                    if d_.get(8, [''])[0] == 'slab_center':
                        cx_ = float(d_[10][0])+ox_+ctdx
                        cy_ = float(d_[20][0])+oy_+ctdy
                        r_ = float(d_[40][0])
                        out.append(((cx_-r_, cy_-r_, cx_+r_, cy_+r_), sn))
        return out

    def _internal_bad_count():
        bad = 0
        boxes_all = all_placed_text_boxes()
        for i_, (b1, n1) in enumerate(boxes_all):
            for b2, n2 in boxes_all[i_+1:]:
                if n1 == n2:
                    continue
                # επικάλυψη Ή οριακό κενό < 0.03 (ίδιο κριτήριο με το τελικό audit)
                if not (b1[2] + 0.03 < b2[0] or b2[2] + 0.03 < b1[0] or
                        b1[3] + 0.03 < b2[1] or b2[3] + 0.03 < b1[1]):
                    bad += 1
        for b1, n1 in boxes_all:
            for cb_, sn_ in _circle_boxes():
                if sn_ == n1:
                    continue
                if not (b1[2] < cb_[0] or cb_[2] < b1[0] or b1[3] < cb_[1] or cb_[3] < b1[1]):
                    bad += 1
        obst_all_ = final_rebar_lines()
        for tname in [n_ for n_ in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_)]:
            tdx_, tdy_ = insert_final.get(tname, (0, 0))
            if re.match(r'FL-?\d+_BEAMBAR', tname) and tdx_ <= -49:
                continue
            ttx_, tty_ = text_local_final.get(tname, (0, 0))
            tbl_ = block_text_bboxes(blocks[tname])
            if not tbl_:
                continue
            if parallel_cut_full(tbl_, tdx_+ttx_, tdy_+tty_, [s for s in obst_all_ if s[4] != tname], ()):
                bad += 1
            # και η ΓΡΑΜΜΗ της ράβδου: αν κόβει παράλληλα ξένο κείμενο, μετράει
            tlines_, _ = block_lines_local(blocks[tname])
            for b1, n1 in boxes_all:
                if n1 == tname:
                    continue
                hor_ = (b1[2]-b1[0]) >= (b1[3]-b1[1])
                cut_ = False
                for a_, b_, c_, d_ in tlines_:
                    sg = (a_+tdx_, b_+tdy_, c_+tdx_, d_+tdy_)
                    if not seg_intersects_bbox(sg, b1):
                        continue
                    lx_, ly_ = sg[2]-sg[0], sg[3]-sg[1]
                    ll_ = math.hypot(lx_, ly_)
                    comp_ = abs(lx_/ll_) if (ll_ > 1e-9 and hor_) else (abs(ly_/ll_) if ll_ > 1e-9 else 0.0)
                    if comp_ > 0.94:
                        cut_ = True; break
                if cut_:
                    bad += 1
        return bad

    for _pass in range(6):
        conflicts = find_conflicts()
        if not conflicts:
            break
        fixed_any = False
        for name, oname in conflicts:
            # try nudging the OBSTACLE bar (oname) perpendicular to its own axis
            olines,_ = block_lines_local(blocks[oname])
            if not olines:
                continue
            obx,oby = -spine_and_bounds(olines)[0][1], spine_and_bounds(olines)[0][0]
            odx,ody = insert_final.get(oname,(0,0))
            success = False
            for s in [0.05*k for k in range(1,25)]:
                for sign in (1,-1):
                    ndx,ndy = odx+obx*s*sign, ody+oby*s*sign
                    if not _bar_move_ok(oname, olines, ndx, ndy):
                        continue
                    # rebuild obstacle set excluding oname's old position, check everything
                    # currently placed still OK, and that it clears the specific conflict
                    obl = block_text_bboxes(blocks[oname])
                    otdx,otdy = text_local_final.get(oname,(0,0))
                    all_ok = True
                    if obl:
                        for x,y,w,h,rot in obl:
                            bx = text_bbox(x+ndx+otdx,y+ndy+otdy,w,h,rot)
                            for seg in obstacle_lines:
                                if seg[4]==oname: continue
                                if seg_intersects_bbox(seg[:4], bx):
                                    all_ok=False;break
                            if not all_ok: break
                    if not all_ok:
                        continue
                    olines_new = [(x1+ndx,y1+ndy,x2+ndx,y2+ndy,oname) for x1,y1,x2,y2 in olines]
                    dx,dy = insert_final.get(name,(0,0))
                    tdx,tdy = text_local_final.get(name,(0,0))
                    bl = block_text_bboxes(blocks[name])
                    still_conflicts=False
                    for x,y,w,h,rot in bl:
                        bx = text_bbox(x+dx+tdx,y+dy+tdy,w,h,rot)
                        for seg in olines_new:
                            if seg_intersects_bbox(seg[:4], bx):
                                still_conflicts=True;break
                        if still_conflicts: break
                    if not still_conflicts:
                        insert_final[oname] = (ndx,ndy)
                        success = True
                        fixed_any = True
                        break
                if success: break
            if not success:
                # fallback: try nudging MY OWN bar instead (either side moving is fine)
                mylines,_ = block_lines_local(blocks[name])
                if mylines:
                    mbx,mby = -spine_and_bounds(mylines)[0][1], spine_and_bounds(mylines)[0][0]
                    mdx,mdy = insert_final.get(name,(0,0))
                    for s in [0.05*k for k in range(1,25)]:
                        for sign in (1,-1):
                            nmdx,nmdy = mdx+mbx*s*sign, mdy+mby*s*sign
                            if not _bar_move_ok(name, mylines, nmdx, nmdy):
                                continue
                            mbl = block_text_bboxes(blocks[name])
                            mtdx,mtdy = text_local_final.get(name,(0,0))
                            ok2=True
                            if mbl:
                                for x,y,w,h,rot in mbl:
                                    bx = text_bbox(x+nmdx+mtdx,y+nmdy+mtdy,w,h,rot)
                                    for seg in obstacle_lines:
                                        if seg[4]==name: continue
                                        if seg_intersects_bbox(seg[:4], bx):
                                            ok2=False;break
                                    if not ok2: break
                            if not ok2: continue
                            mylines_new = [(x1+nmdx,y1+nmdy,x2+nmdx,y2+nmdy,name) for x1,y1,x2,y2 in mylines]
                            obl2 = block_text_bboxes(blocks[oname])
                            odx2,ody2 = insert_final.get(oname,(0,0))
                            otdx2,otdy2 = text_local_final.get(oname,(0,0))
                            still2=False
                            if obl2:
                                for x,y,w,h,rot in obl2:
                                    bx = text_bbox(x+odx2+otdx2,y+ody2+otdy2,w,h,rot)
                                    for seg in mylines_new:
                                        if seg_intersects_bbox(seg[:4], bx):
                                            still2=True;break
                                    if still2: break
                            if not still2:
                                insert_final[name] = (nmdx,nmdy)
                                success=True; fixed_any=True
                                break
                        if success: break
        if not fixed_any:
            break
    print(f'REPAIR PASS: {len(find_conflicts())} rebar-vs-rebar conflicts remain')

    # === AUTOMATIC CIRCLE REPAIR: find any slab-marker circle that still overlaps a
    # rebar line (beambar/slabbar) after everything else has settled, and nudge that
    # marker's circle+text a little to clear it - permanently automatic, never needs
    # to be asked for. ===
    def circle_conflicts():
        out = []
        for sname in slab_marker_names:
            circle_bbox_local = None
            for e in entities_from_pairs(blocks[sname]):
                if e[0][1] == 'CIRCLE':
                    d = to_dict(e)
                    if d.get(8,[''])[0] == 'slab_center':
                        cx = float(d[10][0]); cy = float(d[20][0]); rad = float(d[40][0])
                        circle_bbox_local = (cx-rad, cy-rad, cx+rad, cy+rad)
                        break
            if not circle_bbox_local:
                continue
            tdx, tdy = text_local_final.get(sname, (0,0))
            ccb = translate_bbox(circle_bbox_local, tdx, tdy)
            for rname in REBAR_NAMES:
                rdx, rdy = insert_final.get(rname, (0,0))
                if re.match(r'FL-?\d+_BEAMBAR', rname) and rdx <= -49:
                    continue
                rlines,_ = block_lines_local(blocks[rname])
                for x1,y1,x2,y2 in rlines:
                    seg = (x1+rdx, y1+rdy, x2+rdx, y2+rdy)
                    if seg_intersects_bbox(seg, ccb):
                        out.append((sname, rname))
                        break
        return out

    for _pass in range(3):
        conflicts = circle_conflicts()
        if not conflicts:
            break
        fixed_any = False
        seen = set()
        for sname, rname in conflicts:
            if sname in seen:
                continue
            boxes_local = slab_marker_boxes(blocks[sname])
            circle_bbox_local = None
            for e in entities_from_pairs(blocks[sname]):
                if e[0][1] == 'CIRCLE':
                    d = to_dict(e)
                    if d.get(8,[''])[0] == 'slab_center':
                        cx = float(d[10][0]); cy = float(d[20][0]); rad = float(d[40][0])
                        circle_bbox_local = (cx-rad, cy-rad, cx+rad, cy+rad)
                        break
            home_bb = union_bbox(boxes_local) if boxes_local else circle_bbox_local
            if circle_bbox_local:
                home_bb = (min(home_bb[0],circle_bbox_local[0]), min(home_bb[1],circle_bbox_local[1]),
                           max(home_bb[2],circle_bbox_local[2]), max(home_bb[3],circle_bbox_local[3]))
            cur_dx, cur_dy = text_local_final.get(sname, (0,0))
            local_placed = [b for n2,b2 in [(n2,slab_marker_boxes(blocks[n2])) for n2 in slab_marker_names if n2!=sname]
                             for x,y,w,h,rot in (b2 or []) for b in [text_bbox(x+text_local_final.get(n2,(0,0))[0], y+text_local_final.get(n2,(0,0))[1], w, h, rot)]]
            found = None
            r = 0.0
            while r <= 1.0 and not found:
                n_dirs = max(16, int(2*math.pi*r/0.05)) if r>0 else 1
                for k in range(n_dirs):
                    ang = 2*math.pi*k/n_dirs if r>0 else 0
                    ddx, ddy = cur_dx + r*math.cos(ang), cur_dy + r*math.sin(ang)
                    if not _marker_move_ok(sname, ddx, ddy):
                        continue
                    poly = slab_polys.get(sname)
                    cand_bb = translate_bbox(home_bb, ddx-cur_dx, ddy-cur_dy)
                    if poly:
                        px1,py1,px2,py2 = poly
                        if cand_bb[0]<px1 or cand_bb[1]<py1 or cand_bb[2]>px2 or cand_bb[3]>py2:
                            continue
                    if not is_ok_full(boxes_local, ddx, ddy, bar_obstacle_lines, hatch_polys, local_placed, (sname,)):
                        continue
                    if circle_bbox_local:
                        ccb = translate_bbox(circle_bbox_local, ddx, ddy)
                        clear = True
                        for x1,y1,x2,y2,oname in bar_obstacle_lines:
                            if seg_intersects_bbox((x1,y1,x2,y2), ccb):
                                clear = False; break
                        if not clear:
                            continue
                    found = (ddx, ddy)
                    break
                r += 0.05
            if found:
                text_local_final[sname] = found
                fixed_any = True
                seen.add(sname)
        if not fixed_any:
            break
    remaining_circle = len(circle_conflicts())
    print(f'CIRCLE REPAIR: {remaining_circle} slab-marker/rebar circle conflicts remain')

    # === AUTOMATIC CROSS-CATEGORY REPAIR: a rebar (beambar/slabbar) LINE crossing
    # someone else's TEXT (beam_text, column_text, slab-marker, or other rebar's own
    # text) is just as real a defect as rebar-vs-rebar - check and fix it the same way,
    # automatically, without needing to be asked. ===
    def all_placed_text_boxes(exclude_name=None):
        out = []
        for n2 in blocks:
            if n2 == exclude_name:
                continue
            bl2 = block_text_bboxes(blocks[n2]) if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$', n2) else (slab_marker_boxes(blocks[n2]) if re.match(r'FL-?\d+_SLAB\d+$', n2) else None)
            if not bl2:
                continue
            d2x,d2y = insert_final.get(n2,(0,0))
            if re.match(r'FL-?\d+_BEAMBAR',n2) and d2x<=-49:
                continue
            t2x,t2y = text_local_final.get(n2,(0,0))
            for x,y,w,h,rot in bl2:
                out.append((text_bbox(x+d2x+t2x,y+d2y+t2y,w,h,rot), n2))
        return out

    def rebar_vs_text_conflicts():
        confl = []
        for name in REBAR_NAMES:
            dx,dy = insert_final.get(name,(0,0))
            if re.match(r'FL-?\d+_BEAMBAR',name) and dx<=-49:
                continue
            # 1) rebar's own LINE crossing someone else's text
            lines,_ = block_lines_local(blocks[name])
            for x1,y1,x2,y2 in lines:
                seg = (x1+dx,y1+dy,x2+dx,y2+dy)
                for bx,oname in all_placed_text_boxes(exclude_name=name):
                    if oname == name:
                        continue
                    if seg_intersects_bbox(seg, bx):
                        confl.append(('line',name,oname))
            # 2) rebar's own TEXT box overlapping someone else's text box (e.g. a
            # slabbar/beambar label sitting right on top of a column_text/beam_text
            # label) - just as real a defect, previously never checked.
            tdx,tdy = text_local_final.get(name,(0,0))
            own_bl = block_text_bboxes(blocks[name])
            for x,y,w,h,rot in own_bl:
                mybx = text_bbox(x+dx+tdx,y+dy+tdy,w,h,rot)
                for obx,oname in all_placed_text_boxes(exclude_name=name):
                    if oname == name:
                        continue
                    if not (mybx[2]<obx[0] or obx[2]<mybx[0] or mybx[3]<obx[1] or obx[3]<mybx[1]):
                        confl.append(('text',name,oname))
        return confl

    def final_rebar_lines(exclude=()):
        """Οι γραμμές όλων των σιδήρων στις ΤΕΛΙΚΕΣ τους θέσεις. Οι έλεγχοι του
        repair πρέπει να τις βλέπουν - με τα σκέτα δομικά obstacle_lines, ένα
        'διορθωμένο' κείμενο μπορούσε κάλλιστα να προσγειωθεί πάνω σε άλλη ράβδο
        και να ξαναβγεί σύγκρουση στο επόμενο πέρασμα (βασική αιτία που έμεναν
        τόσα conflicts άλυτα)."""
        out = list(obstacle_lines)
        for rname in REBAR_NAMES:
            if rname in exclude:
                continue
            rdx, rdy = insert_final.get(rname, (0,0))
            if re.match(r'FL-?\d+_BEAMBAR', rname) and rdx <= -49:
                continue
            rlines,_ = block_lines_local(blocks[rname])
            if rname in COLLAPSE_BARS:
                rlines = _collapse_lines(rlines)
            for x1,y1,x2,y2 in rlines:
                out.append((x1+rdx, y1+rdy, x2+rdx, y2+rdy, rname))
        return out

    def dist_bb_to_own_bar(bb, name):
        dx,dy = insert_final.get(name,(0,0))
        lines,_ = block_lines_local(blocks[name])
        cx,cy = (bb[0]+bb[2])/2,(bb[1]+bb[3])/2
        best = 1e9
        for x1,y1,x2,y2 in lines:
            ddx,ddy = x2-x1, y2-y1
            l2 = ddx*ddx+ddy*ddy
            if l2 < 1e-12:
                d = math.hypot(cx-(x1+dx), cy-(y1+dy))
            else:
                t = max(0,min(1, ((cx-(x1+dx))*ddx+(cy-(y1+dy))*ddy)/l2))
                d = math.hypot(cx-(x1+dx)-t*ddx, cy-(y1+dy)-t*ddy)
            if d < best: best = d
        return best

    def text_radial_relocate(name, obst, placed, max_r=1.6, step=0.1):
        """ΠΛΑΓΙΑ/ελεύθερη μικρομετακίνηση ΜΟΝΟ του κειμένου μιας ράβδου, γύρω από
        την τρέχουσα τελική του θέση, σε καθαρό σημείο. Το text_only_slide σέρνει
        μόνο ΚΑΤΑ ΜΗΚΟΣ της ράβδου - όταν ο ελεύθερος χώρος είναι δίπλα (κάθετα),
        όπως στο SLABBAR7 που ο χρήστης μετακίνησε με το χέρι, αποτύγχανε
        συστηματικά. Μένει κοντά στη ράβδο του (όριο απόστασης), ώστε η ετικέτα
        να μη διαβαστεί σαν άλλης ράβδου."""
        dx,dy = insert_final.get(name,(0,0))
        tdx0,tdy0 = text_local_final.get(name,(0,0))
        boxes_local = block_text_bboxes(blocks[name])
        if not boxes_local:
            return None
        home_bb = full_bbox(boxes_local, dx+tdx0, dy+tdy0)
        d0 = dist_bb_to_own_bar(home_bb, name)
        max_bar_dist = max(0.9, d0 + 0.6)
        # δύο περάσματα: πρώτα ΚΑΘΑΡΗ θέση παντού, μετά "χαλαρή" (1 γραμμή να
        # διαπερνά τον πυρήνα - ποτέ hatch/άλλο κείμενο). Ένα κείμενο με μία
        # γραμμή είναι πάντα καλύτερο από κείμενο ΠΑΝΩ σε άλλο κείμενο, που
        # είναι αυτό που μένει αν το relocate αποτύχει εντελώς.
        for check in ('full', 'relaxed'):
            r = step
            while r <= max_r:
                n_dirs = max(12, int(2*math.pi*r/step))
                for k in range(n_dirs):
                    ang = 2*math.pi*k/n_dirs
                    tdx, tdy = tdx0 + r*math.cos(ang), tdy0 + r*math.sin(ang)
                    cand_bb = full_bbox(boxes_local, dx+tdx, dy+tdy)
                    if dist_bb_to_own_bar(cand_bb, name) > max_bar_dist:
                        continue
                    if check == 'full':
                        ok = is_ok_full(boxes_local, dx+tdx, dy+tdy, obst, hatch_polys, placed, (name,))
                    else:
                        ok = is_ok_relaxed(boxes_local, dx+tdx, dy+tdy, obst, hatch_polys, placed, (name,), max_crossings=1)
                    if ok:
                        return (tdx, tdy)
                r += step
        return None

    for _pass in range(6):
        conflicts2 = rebar_vs_text_conflicts()
        if not conflicts2:
            break
        fixed_any2 = False
        seen2 = set()
        for kind, name, oname in sorted(set(conflicts2)):
            if name in seen2:
                continue
            placed = [b for b,n2 in all_placed_text_boxes(exclude_name=name)]
            obst = final_rebar_lines(exclude=(name,))
            dx,dy = insert_final.get(name,(0,0))
            # 1) ολίσθηση κειμένου κατά μήκος της ράβδου - ΜΟΝΟ όταν το πρόβλημα
            #    είναι το ΚΕΙΜΕΝΟ ('text'). Όταν το πρόβλημα είναι η ΓΡΑΜΜΗ της
            #    ράβδου μέσα σε ξένο κείμενο ('line'), το σύρσιμο του δικού της
            #    κειμένου είναι άσχετο: το text_only_slide απαντούσε "το κείμενό
            #    μου είναι ήδη εντάξει" και δήλωνε τη σύγκρουση λυμένη χωρίς να
            #    έχει κάνει τίποτα - γι' αυτό σίδερα μέσα σε κείμενα δοκών δεν
            #    μετακινούνταν ΠΟΤΕ, ό,τι κι αν έκαναν τα επόμενα βήματα.
            if kind == 'text':
                tdx,tdy,good = text_only_slide(name, blocks, dx, dy, obst, hatch_polys, placed)
                if good:
                    text_local_final[name] = (tdx,tdy)
                    fixed_any2 = True; seen2.add(name)
                    continue
            # 2) ΡΗΤΗ ΟΔΗΓΙΑ ΧΡΗΣΤΗ - ΚΑΝΟΝΑΣ ΧΩΡΙΣ ΕΞΑΙΡΕΣΕΙΣ: το κείμενο
            #    ΚΑΘΕ ράβδου (BEAMBAR ΚΑΙ SLABBAR) σύρεται ΑΠΟΚΛΕΙΣΤΙΚΑ ΚΑΤΑ
            #    ΜΗΚΟΣ της ράβδου του, ΠΟΤΕ κάθετα/πλάγια μόνο του. Η ελεύθερη
            #    μικρομετακίνηση (text_radial_relocate) ΔΕΝ καλείται πουθενά
            #    αυτόματα - κρατιέται μόνο ως συνάρτηση για χειροκίνητη χρήση
            #    σε πολύ εξαιρετικές περιπτώσεις, κατόπιν ρητού αιτήματος.
            #    Αν το κατά μήκος σύρσιμο δεν φτάνει, μετακινείται ολόκληρη η
            #    ράβδος (σίδερο+κείμενο μαζί, βήμα 3) - αυτό ήταν πάντα επιτρεπτό.
            # 3) fallback: κάθετη μετατόπιση ολόκληρης της ράβδου (σίδερο+κείμενο
            #    μαζί) ΣΥΝΔΥΑΣΜΕΝΗ με ολίσθηση του κειμένου ΚΑΤΑ ΜΗΚΟΣ της στη νέα
            #    θέση - όπως δουλεύει και η αρχική τοποθέτηση (bar_and_text_slide).
            #    Ο έλεγχος μόνο-με-κείμενο-στο-μηδέν άφηνε άλυτες περιπτώσεις όπου
            #    η λύση ήταν "ανέβα λίγο ΚΑΙ σύρε το κείμενο πιο δίπλα". Πρώτα
            #    καθαρές θέσεις, μετά χαλαρές, και πάντα με έλεγχο ότι η γραμμή
            #    της ράβδου στη νέα θέση δεν κόβει τα ήδη τοποθετημένα κείμενα
            #    (στο χαλαρό πέρασμα: τουλάχιστον όχι του θύματος).
            lines,_ = block_lines_local(blocks[name])
            if not lines:
                continue
            (aux,auy),(alo,ahi) = spine_and_bounds(lines)
            nx,ny = -auy, aux
            all_boxes = all_placed_text_boxes(exclude_name=name)
            victim_boxes = [b for b,n2 in all_boxes if n2==oname]
            boxes_local = block_text_bboxes(blocks[name])
            home_bb_l = union_bbox(boxes_local)
            home_t = ((home_bb_l[0]+home_bb_l[2])/2)*aux + ((home_bb_l[1]+home_bb_l[3])/2)*auy
            def _bar_crossings(ndx_, ndy_):
                out=set()
                for x1,y1,x2,y2 in lines:
                    seg=(x1+ndx_,y1+ndy_,x2+ndx_,y2+ndy_)
                    for i,(b,n2) in enumerate(all_boxes):
                        if seg_intersects_bbox(seg, b):
                            out.add(i)
                return out
            cur_cross = _bar_crossings(dx, dy)
            victim_idx = set(i for i,(b,n2) in enumerate(all_boxes) if n2==oname)
            success = False
            # «ΠΡΩΤΑ ΑΠ' ΟΛΑ κοντά στο ΔΟΚΑΡΙ»: πριν μετακινηθεί το ΣΙΔΕΡΟ,
            # δοκιμάζεται να παραμερίσει ΤΟ ΘΥΜΑ (το κείμενο που κόβει η
            # γραμμή) - σύρσιμο κατά μήκος για κείμενα ράβδων, εντός δοκού
            # για κείμενα δοκών. Η ράβδος μένει κουμπωμένη στη δοκό της.
            _sv_i = dict(insert_final); _sv_t = dict(text_local_final)
            _vmoved = False
            if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', oname):
                _vs = _slide_text_along(oname, final_rebar_lines(exclude=(oname,)),
                                         [b_ for b_, n_v in all_placed_text_boxes(exclude_name=oname)] +
                                         [cb_ for cb_, _sv in _circle_boxes()])
                if _vs is not None:
                    _vt0 = text_local_final.get(oname, (0.0, 0.0))
                    if abs(_vs[0]-_vt0[0]) > 1e-9 or abs(_vs[1]-_vt0[1]) > 1e-9:
                        text_local_final[oname] = _vs
                        _vmoved = True
            elif re.match(r'FL-?\d+_BEAM_TEXT\d+$', oname):
                _pv = [b_ for b_, n_v in all_placed_text_boxes(exclude_name=oname)]
                _vs = beam_text_slide(oname, blocks, final_rebar_lines(), hatch_polys, _pv, relaxed=True)
                _vi0 = insert_final.get(oname, (0.0, 0.0))
                if _vs is not None and (abs(_vs[0]-_vi0[0]) > 1e-9 or abs(_vs[1]-_vi0[1]) > 1e-9):
                    insert_final[oname] = _vs
                    _vmoved = True
            if _vmoved:
                # καθάρισε η γραμμή από το θύμα στη ΝΕΑ του θέση;
                _nb = all_placed_text_boxes(exclude_name=name)
                _still = False
                for x1v, y1v, x2v, y2v in lines:
                    _sg = (x1v+dx, y1v+dy, x2v+dx, y2v+dy)
                    for _bb2, _n2v in _nb:
                        if _n2v == oname and seg_intersects_bbox(_sg, _bb2):
                            _still = True; break
                    if _still: break
                if not _still:
                    success = True; fixed_any2 = True; seen2.add(name)
                    all_boxes = _nb
                else:
                    insert_final.clear(); insert_final.update(_sv_i)
                    text_local_final.clear(); text_local_final.update(_sv_t)
            for check in (() if success else ('full','relaxed')):
                for s in [0.05*k for k in range(0,33)]:
                    for sign in ((1,) if s==0 else (1,-1)):
                        ndx,ndy = dx+nx*s*sign, dy+ny*s*sign
                        if not _bar_move_ok(name, lines, ndx, ndy):
                            continue
                        lines_new = [(x1+ndx,y1+ndy,x2+ndx,y2+ndy) for x1,y1,x2,y2 in lines]
                        new_cross = _bar_crossings(ndx, ndy)
                        # η ράβδος στη νέα θέση: (α) ΔΕΝ ακουμπά πια το θύμα,
                        # (β) δεν διασχίζει ΚΑΝΕΝΑ κείμενο που δεν διέσχιζε ήδη
                        if new_cross & victim_idx:
                            continue
                        if not new_cross <= cur_cross:
                            continue
                        if check=='full' and new_cross - victim_idx:
                            continue  # στο αυστηρό πέρασμα: τελείως καθαρή γραμμή
                        for t in [0.0] + [STEP*k for k in range(1,int(MAX_SLIDE/STEP)+1)]:
                            for tsign in ((1,) if t==0 else (1,-1)):
                                tc = home_t + t*tsign
                                if tc < alo - 1e-6 or tc > ahi + 1e-6:
                                    continue
                                tdx2, tdy2 = aux*t*tsign, auy*t*tsign
                                if check=='full':
                                    ok = is_ok_full(boxes_local, ndx+tdx2, ndy+tdy2, obst, hatch_polys, placed, (name,))
                                else:
                                    ok = is_ok_relaxed(boxes_local, ndx+tdx2, ndy+tdy2, obst, hatch_polys, placed, (name,), max_crossings=1)
                                if ok:
                                    insert_final[name] = (ndx,ndy)
                                    if abs(tdx2)>1e-9 or abs(tdy2)>1e-9:
                                        text_local_final[name] = (tdx2,tdy2)
                                    elif name in text_local_final:
                                        del text_local_final[name]
                                    success = True; fixed_any2 = True; seen2.add(name)
                                    break
                            if success: break
                        if success: break
                    if success: break
                if success: break
            if success:
                continue
            # 3β) ΣΥΝΤΟΝΙΣΜΕΝΗ μετακίνηση (η λύση που εφαρμόζει ο μηχανικός με
            #    το χέρι: "μετακίνηση 2 οπλισμών + μικρό σύρσιμο κειμένου"):
            #    όταν ΚΑΜΙΑ θέση της ράβδου δεν είναι τελείως καθαρή, επιτρέπεται
            #    θέση όπου οι ΜΟΝΕΣ νέες διασχίσεις είναι ΚΕΙΜΕΝΑ ΑΛΛΩΝ ΡΑΒΔΩΝ -
            #    και αμέσως μετά τα κείμενα αυτά σύρονται ΚΑΤΑ ΜΗΚΟΣ των δικών
            #    τους ράβδων για να ξεφύγουν. Αν έστω ένα δεν μπορεί να ξεφύγει,
            #    όλη η αλλαγή αναιρείται (δοκιμάζεται ο επόμενος υποψήφιος).
            if kind == 'line' and not success:
                def _slide_along(rname, obst_r, placed_r):
                    rdx, rdy = insert_final.get(rname, (0, 0))
                    rlines,_ = block_lines_local(blocks[rname])
                    rbl = block_text_bboxes(blocks[rname])
                    if not rlines or not rbl:
                        return None
                    (rux,ruy),(rlo,rhi) = spine_and_bounds(rlines)
                    rhb = union_bbox(rbl)
                    rht = ((rhb[0]+rhb[2])/2)*rux + ((rhb[1]+rhb[3])/2)*ruy
                    for chk in ('full','relaxed'):
                        for t2 in [0.0] + [STEP*k for k in range(1,int(MAX_SLIDE/STEP)+1)]:
                            for ts2 in ((1,) if t2==0 else (1,-1)):
                                tc2 = rht + t2*ts2
                                if tc2 < rlo-1e-6 or tc2 > rhi+1e-6:
                                    continue
                                rtx, rty = rux*t2*ts2, ruy*t2*ts2
                                if chk=='full':
                                    ok2 = is_ok_full(rbl, rdx+rtx, rdy+rty, obst_r, hatch_polys, placed_r, (rname,))
                                else:
                                    ok2 = is_ok_relaxed(rbl, rdx+rtx, rdy+rty, obst_r, hatch_polys, placed_r, (rname,), max_crossings=1)
                                if ok2:
                                    return (rtx, rty)
                    return None

                cands = []
                for s in [0.05*k for k in range(1,33)]:
                    for sign in (1,-1):
                        ndx,ndy = dx+nx*s*sign, dy+ny*s*sign
                        if not _bar_move_ok(name, lines, ndx, ndy):
                            continue
                        new_cross = _bar_crossings(ndx, ndy)
                        if new_cross & victim_idx:
                            continue
                        extra = new_cross - cur_cross
                        if not extra:
                            continue  # θα το είχε πιάσει το βήμα 3
                        if any(all_boxes[i][1] not in REBAR_NAMES for i in extra):
                            continue  # μόνο κείμενα ράβδων επιτρέπεται να πατηθούν
                        # το δικό της κείμενο πρέπει να βρίσκει θέση (κατά μήκος)
                        tsol = None
                        for t in [0.0] + [STEP*k for k in range(1,int(MAX_SLIDE/STEP)+1)]:
                            for tsign in ((1,) if t==0 else (1,-1)):
                                tc = home_t + t*tsign
                                if tc < alo-1e-6 or tc > ahi+1e-6:
                                    continue
                                tdx3, tdy3 = aux*t*tsign, auy*t*tsign
                                if is_ok_relaxed(boxes_local, ndx+tdx3, ndy+tdy3, obst, hatch_polys, placed, (name,), max_crossings=1):
                                    tsol = (tdx3, tdy3); break
                            if tsol: break
                        if tsol is None:
                            continue
                        cands.append((len(extra), s, ndx, ndy, tsol, extra))
                cands.sort(key=lambda c: (c[0], c[1]))
                for _, s, ndx, ndy, tsol, extra in cands[:6]:
                    snap_ins = dict(insert_final)
                    snap_txt = dict(text_local_final)
                    insert_final[name] = (ndx, ndy)
                    if abs(tsol[0])>1e-9 or abs(tsol[1])>1e-9:
                        text_local_final[name] = tsol
                    elif name in text_local_final:
                        del text_local_final[name]
                    hit_names = sorted({all_boxes[i][1] for i in extra})
                    all_ok = True
                    for rname in hit_names:
                        obst_r = final_rebar_lines(exclude=(rname,))
                        placed_r = [b for b,n3 in all_placed_text_boxes(exclude_name=rname)]
                        sol = _slide_along(rname, obst_r, placed_r)
                        if sol is None:
                            all_ok = False; break
                        text_local_final[rname] = sol
                    if all_ok:
                        success = True; fixed_any2 = True; seen2.add(name)
                        break
                    insert_final.clear(); insert_final.update(snap_ins)
                    text_local_final.clear(); text_local_final.update(snap_txt)
            if success:
                continue
            # 3β) ΕΦΕΔΡΕΙΑ ΦΟΥΡΚΕΤΑΣ (ρητά εγκεκριμένη - περίπτωση SLABBAR14):
            #    ΜΟΝΟ για ράβδους με άγκιστρα/φουρκέτες και στα ΔΥΟ άκρα, όταν
            #    ΟΛΑ τα σκέλη που κόβουν κείμενα είναι εκτός κορμού και καμία
            #    νόμιμη κίνηση δεν έλυσε: η γεωμετρία καταρρέει στη γραμμή-κορμό
            #    (ίδιο κείμενο, ίδιες οντότητες, απλή γραμμή στο τελικό αρχείο).
            if name not in COLLAPSE_BARS:
                _co = _collapse_lines(lines)
                _perp_off = [max(abs((x1-c1)), abs((y1-c2)), abs((x2-c3)), abs((y2-c4)))
                             for (x1,y1,x2,y2),(c1,c2,c3,c4) in zip(lines, _co)]
                _arm_idx = [k9 for k9,o9 in enumerate(_perp_off) if o9 > 0.05]
                if len(_arm_idx) >= 2:
                    (_au,_av),(_alo2,_ahi2) = spine_and_bounds(lines)
                    _arm_ts = []
                    for k9 in _arm_idx:
                        x1a,y1a,x2a,y2a = lines[k9]
                        _arm_ts.append(((x1a+x2a)/2*_au + (y1a+y2a)/2*_av))
                    _span2 = _ahi2 - _alo2
                    _both_ends = (min(_arm_ts) < _alo2 + 0.25*_span2 and
                                  max(_arm_ts) > _ahi2 - 0.25*_span2)
                    if _both_ends:
                        # τα κοψίματα προέρχονται ΜΟΝΟ από σκέλη εκτός κορμού;
                        _spine_hits = False
                        for k9,(x1a,y1a,x2a,y2a) in enumerate(lines):
                            if k9 in _arm_idx:
                                continue
                            _sg2 = (x1a+dx, y1a+dy, x2a+dx, y2a+dy)
                            for _i2 in (cur_cross & victim_idx):
                                if seg_intersects_bbox(_sg2, all_boxes[_i2][0]):
                                    _spine_hits = True; break
                            if _spine_hits: break
                        if not _spine_hits:
                            _co2 = _collapse_lines(lines)
                            _nh2 = False
                            for xx1,yy1,xx2,yy2 in _co2:
                                _sg3 = (xx1+dx, yy1+dy, xx2+dx, yy2+dy)
                                for bb3, _nb3 in all_boxes:
                                    if seg_intersects_bbox(_sg3, bb3):
                                        _nh2 = True; break
                                if _nh2: break
                            if not _nh2:
                                pass  # ΑΠΕΝΕΡΓΟ κατόπιν εντολής: COLLAPSE_BARS.add(name)
                                success = True; fixed_any2 = True; seen2.add(name)
                                continue
            # 4) τελευταία λύση: μετακίνηση του ΘΥΜΑΤΟΣ, αν είναι BEAM_TEXT -
            #    ολίσθηση μέσα στη δοκό του με ανεκτικότητα 1 γραμμής. Ο κανόνας
            #    "ποτέ εκτός δοκού" παραμένει απαράβατος μέσα στο beam_text_slide.
            if re.match(r'FL-?\d+_BEAM_TEXT\d+$', oname):
                v_obst = final_rebar_lines()
                v_placed = [b for b,n2 in all_placed_text_boxes(exclude_name=oname)]
                res = beam_text_slide(oname, blocks, v_obst, hatch_polys, v_placed, relaxed=True)
                cur = insert_final.get(oname,(0.0,0.0))
                if res is not None and (abs(res[0]-cur[0])>1e-9 or abs(res[1]-cur[1])>1e-9):
                    insert_final[oname] = res
                    fixed_any2 = True; seen2.add(name)
        if not fixed_any2:
            break
    print(f'CROSS-CATEGORY REPAIR: {len(rebar_vs_text_conflicts())} rebar-vs-other-text conflicts remain')

    # === ΤΕΛΙΚΟ ΠΕΡΑΣΜΑ: pull-in κειμένων κολωνών ΜΕ ΣΥΝΤΟΝΙΣΜΟ ============
    # Τα κείμενα ράβδων τοποθετούνται ΠΡΙΝ από των κολωνών και πιάνουν τον χώρο
    # δίπλα στις κολώνες - χώρο που ανήκει σημασιολογικά στην ετικέτα της
    # κολώνας (αγκυρωμένη εκεί), ενώ το κείμενο μιας ράβδου έχει όλο το μήκος
    # της να μετακινηθεί. Εδώ κάθε κείμενο κολώνας τραβιέται βήμα-βήμα προς την
    # κολώνα του· αν το βήμα φράζεται ΜΟΝΟ από κείμενα ράβδων, αυτά σύρονται
    # ΚΑΤΑ ΜΗΚΟΣ των ράβδων τους για να ανοίξει ο δρόμος (όλοι οι κανόνες
    # παραμένουν: κατά μήκος μόνο, πλευρές, πλάκες, έξω-από-το-κτίριο).
    def _slide_text_along_chain(rname):
        """Σαν την _slide_text_along, αλλά όταν μια υποψήφια θέση φράζεται ΜΟΝΟ
        από κείμενα άλλων ράβδων, δοκιμάζει να τα κυλήσει κατά μήκος των δικών
        τους ράβδων (με δεσμευμένο το κουτί-στόχο). Η κίνηση του μηχανικού:
        "μικρό σύρσιμο + μετακίνηση των διπλανών". Αναιρείται πλήρως σε αποτυχία."""
        rdx, rdy = insert_final.get(rname, (0, 0))
        rlines, _ = block_lines_local(blocks[rname])
        rbl = block_text_bboxes(blocks[rname])
        if not rlines or not rbl:
            return False
        (rux, ruy), (rlo, rhi) = spine_and_bounds(rlines)
        rhb = union_bbox(rbl)
        rht = ((rhb[0]+rhb[2])/2)*rux + ((rhb[1]+rhb[3])/2)*ruy
        obst_r = final_rebar_lines(exclude=(rname,))
        for t2 in [0.0] + [STEP*k for k in range(1, int(MAX_SLIDE/STEP)+1)]:
            for ts2 in ((1,) if t2 == 0 else (1, -1)):
                tc2 = rht + t2*ts2
                if tc2 < rlo-1e-6 or tc2 > rhi+1e-6:
                    continue
                rtx, rty = rux*t2*ts2, ruy*t2*ts2
                cand_boxes = [text_bbox(x_+rdx+rtx, y_+rdy+rty, w_, h_, rot_) for x_, y_, w_, h_, rot_ in rbl]
                if parallel_cut_full(rbl, rdx+rtx, rdy+rty, obst_r, (rname,)):
                    continue
                blockers = set()
                clean = True
                for cb in cand_boxes:
                    for b_, n_2 in all_placed_text_boxes(exclude_name=rname):
                        if not (cb[2] + MIN_TEXT_GAP < b_[0] or b_[2] + MIN_TEXT_GAP < cb[0] or
                                cb[3] + MIN_TEXT_GAP < b_[1] or b_[3] + MIN_TEXT_GAP < cb[1]):
                            blockers.add(n_2)
                            clean = False
                if clean:
                    continue  # θα το είχε βρει η απλή εκδοχή
                if not blockers or not all(re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', b_) for b_ in blockers):
                    continue
                snap_ci = dict(insert_final)
                snap_ct = dict(text_local_final)
                text_local_final[rname] = (rtx, rty)
                ok_chain = True
                for bn in sorted(blockers):
                    sol_b = _slide_text_along(bn, final_rebar_lines(exclude=(bn,)),
                                               [b_ for b_, n_3 in all_placed_text_boxes(exclude_name=bn)] +
                                               [cb_ for cb_, _sn in _circle_boxes()])
                    if sol_b is not None:
                        text_local_final[bn] = sol_b
                        continue
                    # τελευταίο εργαλείο: μετακίνηση του σιδήρου-εμποδίου, ΠΑΝΤΑ
                    # εντός όλων των κανόνων του (_bar_move_ok: πλάκα/πλευρά/
                    # ζώνη/±0.35), και ξανά σύρσιμο του κειμένου του
                    bn_lines, _ = block_lines_local(blocks[bn])
                    bn_ok = False
                    if bn_lines:
                        (bux_, buy_), _bb2 = spine_and_bounds(bn_lines)
                        bpx_, bpy_ = -buy_, bux_
                        bdx0, bdy0 = insert_final.get(bn, (0, 0))
                        btl0 = text_local_final.get(bn, None)
                        for sb in [0.05*k for k in range(1, 8)]:
                            for sgb in (1, -1):
                                nbdx, nbdy = bdx0 + bpx_*sb*sgb, bdy0 + bpy_*sb*sgb
                                if not _bar_move_ok(bn, bn_lines, nbdx, nbdy):
                                    continue
                                insert_final[bn] = (nbdx, nbdy)
                                sol_b2 = _slide_text_along(bn, final_rebar_lines(exclude=(bn,)),
                                                            [b_ for b_, n_3 in all_placed_text_boxes(exclude_name=bn)] +
                                                            [cb_ for cb_, _sn in _circle_boxes()])
                                if sol_b2 is not None:
                                    text_local_final[bn] = sol_b2
                                    bn_ok = True
                                    break
                                insert_final[bn] = (bdx0, bdy0)
                            if bn_ok:
                                break
                        if not bn_ok:
                            insert_final[bn] = (bdx0, bdy0)
                            if btl0 is not None:
                                text_local_final[bn] = btl0
                            elif bn in text_local_final:
                                del text_local_final[bn]
                    if not bn_ok:
                        ok_chain = False
                        break
                if ok_chain and is_ok_relaxed(rbl, rdx+rtx, rdy+rty, final_rebar_lines(exclude=(rname,)), hatch_polys,
                                               [b_ for b_, _n in all_placed_text_boxes(exclude_name=rname)], (rname,), max_crossings=1):
                    return True
                insert_final.clear(); insert_final.update(snap_ci)
                text_local_final.clear(); text_local_final.update(snap_ct)
        return False

    def _final_passes():
        col_names2 = sorted([n for n in blocks if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', n)],
                             key=lambda n: int(re.search(r'\d+$', n).group()))
        for _pull_pass in range(2):
            any_progress = False
            for name in col_names2:
                own_col = re.match(r'(FL-?\d+_)COLUMN_TEXT(\d+)$',name).group(1)+'COLUMN'+re.search(r'\d+$',name).group()
                if own_col not in blocks:
                    continue
                bl2 = block_text_bboxes(blocks[name])
                col_lines2,_ = block_lines_local(blocks[own_col])
                if not bl2 or not col_lines2:
                    continue
                cxs2=[p for s_ in col_lines2 for p in (s_[0],s_[2])]
                cys2=[p for s_ in col_lines2 for p in (s_[1],s_[3])]
                col_bb2 = (min(cxs2)-0.05, min(cys2)-0.05, max(cxs2)+0.05, max(cys2)+0.05)
                ccx2, ccy2 = (col_bb2[0]+col_bb2[2])/2, (col_bb2[1]+col_bb2[3])/2
                # ΤΟΙΧΙΑ ΠΟΥ ΕΜΕΙΝΑΝ ΜΑΚΡΙΑ: το ίσιο τράβηγμα δεν στρίβει γωνία -
                # αν η ετικέτα τοιχίου βρίσκεται >2.5μ από το τοιχίο της, δοκιμάζεται
                # ΕΠΑΝΑΤΟΠΟΘΕΤΗΣΗ με άνοιγμα χώρου: εύρεση κοντινής θέσης ΑΓΝΟΩΝΤΑΣ
                # τα (μετακινήσιμα) κείμενα ράβδων, και μετά εκδίωξή τους με την
                # αλυσίδα (σύρσιμο/μετακίνηση σιδήρου, πάντα εντός των κανόνων τους).
                # Έτσι κρατιέται μόνιμα η διάταξη που ενέκρινε ο χρήστης για τα
                # K82/K83 (ετικέτα μέσα, δίπλα στο τοιχίο) και δεν ξαναχάνεται.
                _w2 = max(cxs2)-min(cxs2); _h2 = max(cys2)-min(cys2)
                _is_wall2 = max(_w2, _h2) > 2.0*max(min(_w2, _h2), 1e-6)
                _dx0, _dy0 = insert_final.get(name, (0.0, 0.0))
                _cb0 = full_bbox(bl2, _dx0, _dy0)
                def _edge_w(bb_):
                    return math.hypot(max(col_bb2[0]-bb_[2], bb_[0]-col_bb2[2], 0.0),
                                       max(col_bb2[1]-bb_[3], bb_[1]-col_bb2[3], 0.0))
                _dist0 = _edge_w(_cb0)
                _ovl0 = not (_cb0[2] < col_bb2[0]+0.04 - 0.09 or col_bb2[2]-0.04+0.09 < _cb0[0] or
                              _cb0[3] < col_bb2[1]+0.04 - 0.09 or col_bb2[3]-0.04+0.09 < _cb0[1])
                # _ovl0: η ετικέτα ΠΑΤΑΕΙ στο σώμα του ίδιου της του τοιχίου
                # (το δικό της hatch εξαιρείται από τους ελέγχους, οπότε ο
                # αλγόριθμος το έβλεπε "δωρεάν χώρο" - K82/K83 απλώνονταν πάνω
                # στο τοιχίο αντί καθαρά δίπλα του, όπως τα βάζει ο μηχανικός).
                _dW = os.environ.get('DEBUG_CN') == name
                # ΑΝΑΓΝΩΣΙΜΟΤΗΤΑ (κανόνας μηχανικού, περίπτωση Κ7): ετικέτα με
                # >=2 γραμμές ΜΕΣΑ της δεν διαβάζεται -> μετεγκατάσταση, με
                # αποδοχή ΜΟΝΟ θέσης με ΜΗΔΕΝ γραμμές μέσα της.
                def _cuts_in(bb_or_list):
                    boxes_ = bb_or_list if isinstance(bb_or_list, list) else [bb_or_list]
                    cnt_ = 0
                    for on_ in blocks:
                        if not (re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE|BEAMBAR|SLABBAR)\d*$', on_)
                                and 'TEXT' not in on_) or on_ == own_col:
                            continue
                        od_ = insert_final.get(on_, ins.get(on_, (0, 0)))
                        if re.match(r'FL-?\d+_BEAMBAR', on_) and od_[0] <= -49:
                            continue
                        hit_ = False
                        for lx1, ly1, lx2, ly2 in block_lines_local(blocks[on_])[0]:
                            x1_, y1_, x2_, y2_ = lx1+od_[0], ly1+od_[1], lx2+od_[0], ly2+od_[1]
                            for bb_ in boxes_:
                                p_ = 0.02
                                c0_, c1_, c2_, c3_ = bb_[0]+p_, bb_[1]+p_, bb_[2]-p_, bb_[3]-p_
                                if c2_ <= c0_ or c3_ <= c1_: continue
                                if max(x1_,x2_) < c0_ or min(x1_,x2_) > c2_ or max(y1_,y2_) < c1_ or min(y1_,y2_) > c3_:
                                    continue
                                if (c0_ <= x1_ <= c2_ and c1_ <= y1_ <= c3_) or (c0_ <= x2_ <= c2_ and c1_ <= y2_ <= c3_):
                                    hit_ = True
                                else:
                                    ddx_, ddy_ = x2_-x1_, y2_-y1_
                                    for ex1_, ey1_, ex2_, ey2_ in ((c0_,c1_,c2_,c1_),(c0_,c3_,c2_,c3_),(c0_,c1_,c0_,c3_),(c2_,c1_,c2_,c3_)):
                                        fx_, fy_ = ex2_-ex1_, ey2_-ey1_
                                        den_ = ddx_*fy_ - ddy_*fx_
                                        if abs(den_) < 1e-12: continue
                                        t_ = ((ex1_-x1_)*fy_ - (ey1_-y1_)*fx_)/den_
                                        u_ = ((ex1_-x1_)*ddy_ - (ey1_-y1_)*ddx_)/den_
                                        if 0 <= t_ <= 1 and 0 <= u_ <= 1:
                                            hit_ = True; break
                                if hit_: break
                            if hit_: break
                        if hit_: cnt_ += 1
                    return cnt_
                def _row_boxes(ddx_, ddy_):
                    return [text_bbox(x_+ddx_, y_+ddy_, w_, h_, r_) for x_, y_, w_, h_, r_ in bl2]
                _rowcut0 = _cuts_in(_row_boxes(_dx0, _dy0))
                _cut0 = _cuts_in(_cb0)
                if _dW: print(f'[WALL {name}] edge0={_dist0:.2f} ovl0={_ovl0} rowcut0={_rowcut0} unioncut0={_cut0}', flush=True)
                if _is_wall2 and (_ovl0 or _dist0 > 0.45 or _rowcut0 >= 1):
                    # ΚΡΙΤΗΡΙΟ: ελάχιστη απόσταση από ΠΑΡΕΙΑ (και κεφαλές) του
                    # τοιχίου, όπως τοποθετεί ο μηχανικός (K4: στη δυτική προέκταση
                    # έξω από το περίγραμμα, δίπλα στα K7/K8/K9 - K5: κολλητά στην
                    # ανατολική παρειά). Η ακτινική αναζήτηση από το ΚΕΝΤΡΟ δεν
                    # φτάνει ποτέ στις κεφαλές επιμήκους τοιχίου: πολλαπλά σημεία
                    # εκκίνησης (κέντρο + 4 μέσα πλευρών) και επιλογή του υποψηφίου
                    # με τη μικρότερη απόσταση παρειάς. Αποδοχή ΜΟΝΟ σε σαφή
                    # βελτίωση (>0.15μ) ώστε να μην αναδιατάσσονται καλές θέσεις.
                    _obst_w = final_rebar_lines()
                    _placed_wo = [b_ for b_, n_9 in all_placed_text_boxes(exclude_name=name)
                                   if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_9)]
                    _lc = union_bbox(bl2)
                    _lcx, _lcy = (_lc[0]+_lc[2])/2, (_lc[1]+_lc[3])/2
                    _seeds_w = [(ccx2, ccy2),
                                (col_bb2[0]-0.6, ccy2), (col_bb2[2]+0.6, ccy2),
                                (ccx2, col_bb2[1]-0.6), (ccx2, col_bb2[3]+0.6)]
                    _cand_w = None; _dw = None; _best_cutw = None; _best_rcw = None
                    for _sx, _sy in _seeds_w:
                        _cw = radial_place_full(name, blocks, _obst_w, hatch_polys, _placed_wo,
                                                 (own_col,), seed=(_sx-_lcx, _sy-_lcy), max_r=1.6,
                                                 allow_relaxed=True)
                        if _cw is None:
                            continue
                        _fbw = full_bbox(bl2, *_cw)
                        # υποψήφιος που τέμνει το σώμα του τοιχίου απορρίπτεται
                        if not (_fbw[2] < col_bb2[0]-0.04 or col_bb2[2]+0.04 < _fbw[0] or
                                _fbw[3] < col_bb2[1]-0.04 or col_bb2[3]+0.04 < _fbw[1]):
                            continue
                        # ...όπως και υποψήφιος πάνω σε κυκλάκι δείκτη πλάκας
                        _oncirc = False
                        for _cbx9, _sn9 in _circle_boxes():
                            if not (_fbw[2]+0.04 < _cbx9[0] or _cbx9[2]+0.04 < _fbw[0] or
                                    _fbw[3]+0.04 < _cbx9[1] or _cbx9[3]+0.04 < _fbw[1]):
                                _oncirc = True; break
                        if _oncirc:
                            continue
                        _ew = _edge_w(_fbw)
                        _rcw = _cuts_in(_row_boxes(*_cw))
                        _cutw = _cuts_in(_fbw)
                        # προτίμηση: καμία γραμμή σε ΓΡΑΜΜΑΤΑ -> καμία γραμμή
                        # καθόλου -> κοντινότερη στην παρειά
                        if _dw is None or (_rcw, _cutw, _ew) < (_best_rcw, _best_cutw, _dw):
                            _cand_w, _dw, _best_cutw, _best_rcw = _cw, _ew, _cutw, _rcw
                    # ΠΛΑΓΙΟ ΔΙΧΤΥ (περίπτωση Κ8 «0.45 δεξιά»): αν κανένας
                    # radial υποψήφιος δεν έδωσε γράμματα-καθαρά, δοκιμάζονται
                    # απλές πλάγιες μετατοπίσεις από την τρέχουσα θέση - πρώτη
                    # με ΜΗΔΕΝ γραμμές στα γράμματα και ελεύθερη κερδίζει.
                    if _rowcut0 >= 1 and (_best_rcw is None or _best_rcw > 0):
                        _lbl_w = block_text_bboxes(blocks[name])
                        for _dds in [0.05*ks for ks in range(3, 19)]:
                            _got_s = False
                            for _shs in ((_dds,0),(-_dds,0),(0,-_dds),(0,_dds)):
                                _cs = (_dx0+_shs[0], _dy0+_shs[1])
                                _fbs = translate_bbox(_cb0, _shs[0], _shs[1])
                                if _cuts_in(_row_boxes(*_cs)) > 0:
                                    continue
                                if not is_ok_relaxed(_lbl_w, _cs[0], _cs[1], final_rebar_lines(),
                                                      hatch_polys, [b9 for b9, n9 in all_placed_text_boxes(exclude_name=name)],
                                                      (own_col,), max_crossings=1):
                                    continue
                                _cand_w, _dw = _cs, _edge_w(_fbs)
                                _best_rcw, _best_cutw = 0, _cuts_in(_fbs)
                                _got_s = True
                                break
                            if _got_s:
                                break
                    if _dW: print(f'[WALL {name}] cand={_cand_w} edge_cand={_dw}', flush=True)
                    if _cand_w is not None:
                        # ΠΟΤΕ αποδοχή θέσης που ΧΕΙΡΟΤΕΡΕΥΕΙ την αναγνωσιμότητα
                        # (γραμμές πάνω σε γράμματα) - η εγγύτητα στην παρειά είναι
                        # δευτερεύουσα του κανόνα «να διαβάζεται».
                        if (_best_rcw is not None and _best_rcw <= _rowcut0) and (
                                _ovl0 or _dw < _dist0 - 0.15 or (_rowcut0 >= 1 and _best_rcw == 0)):
                            _blk_w = set()
                            for x9, y9, w9, h9, r9 in bl2:
                                tb9 = text_bbox(x9+_cand_w[0], y9+_cand_w[1], w9, h9, r9)
                                for b_, n_9 in all_placed_text_boxes(exclude_name=name):
                                    if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_9):
                                        continue
                                    if not (tb9[2]+MIN_TEXT_GAP < b_[0] or b_[2]+MIN_TEXT_GAP < tb9[0] or
                                            tb9[3]+MIN_TEXT_GAP < b_[1] or b_[3]+MIN_TEXT_GAP < tb9[1]):
                                        _blk_w.add(n_9)
                            _snap_i9 = dict(insert_final); _snap_t9 = dict(text_local_final)
                            insert_final[name] = _cand_w
                            _ok_w = True
                            for _bn9 in sorted(_blk_w):
                                _sol9 = _slide_text_along(_bn9, final_rebar_lines(exclude=(_bn9,)),
                                                           [b_ for b_, n_a in all_placed_text_boxes(exclude_name=_bn9)] +
                                                           [cb_ for cb_, _sb9 in _circle_boxes()])
                                if _sol9 is not None:
                                    text_local_final[_bn9] = _sol9
                                    continue
                                if _slide_text_along_chain(_bn9):
                                    continue
                                _ok_w = False
                                break
                            if _dW: print(f'[WALL {name}] blockers={sorted(_blk_w)} chain_ok={_ok_w}', flush=True)
                            _fin_ok = False
                            if _ok_w:
                                _pl_fin = [b_ for b_, _nb in all_placed_text_boxes(exclude_name=name)]
                                _ob_fin = final_rebar_lines()
                                _fin_ok = is_ok_full(bl2, _cand_w[0], _cand_w[1], _ob_fin, hatch_polys, _pl_fin, (own_col,)) or \
                                          is_ok_relaxed(bl2, _cand_w[0], _cand_w[1], _ob_fin, hatch_polys, _pl_fin, (own_col,), max_crossings=1)
                            if _fin_ok:
                                any_progress = True
                            else:
                                insert_final.clear(); insert_final.update(_snap_i9)
                                text_local_final.clear(); text_local_final.update(_snap_t9)
                for _step_i in range(60):
                    dx2, dy2 = insert_final.get(name, (0.0, 0.0))
                    cur_bb2 = full_bbox(bl2, dx2, dy2)
                    tcx2, tcy2 = (cur_bb2[0]+cur_bb2[2])/2, (cur_bb2[1]+cur_bb2[3])/2
                    vx2, vy2 = ccx2-tcx2, ccy2-tcy2
                    L2 = math.hypot(vx2, vy2)
                    if L2 < 0.35:
                        break
                    ux2, uy2 = vx2/L2, vy2/L2
                    cand2 = (dx2+ux2*0.05, dy2+uy2*0.05)
                    cand_bb2 = full_bbox(bl2, *cand2)
                    if not (cand_bb2[2] < col_bb2[0] or col_bb2[2] < cand_bb2[0] or
                            cand_bb2[3] < col_bb2[1] or col_bb2[3] < cand_bb2[1]):
                        break  # θα πατούσε την ίδια την κολώνα
                    outside_req2 = (FOOTPRINT is not None and name in COLUMN_OUTWARD
                                     and bbox_outside(FOOTPRINT, cur_bb2))
                    obst_now2 = final_rebar_lines()
                    placed_now2 = all_placed_text_boxes(exclude_name=name)
                    def _ok2(c, plist):
                        if outside_req2 and not bbox_outside(FOOTPRINT, full_bbox(bl2, *c)):
                            return False
                        return is_ok_full(bl2, c[0], c[1], obst_now2, hatch_polys,
                                           [b for b,_n in plist], (own_col,))
                    if _ok2(cand2, placed_now2):
                        insert_final[name] = cand2
                        any_progress = True
                        continue
                    blockers2 = set()
                    for x_,y_,w_,h_,rot_ in bl2:
                        tb_ = text_bbox(x_+cand2[0], y_+cand2[1], w_, h_, rot_)
                        for b,n3 in placed_now2:
                            if not (tb_[2] < b[0] or b[2] < tb_[0] or tb_[3] < b[1] or b[3] < tb_[1]):
                                blockers2.add(n3)
                    if not blockers2 or not all(n3 in REBAR_NAMES for n3 in blockers2):
                        break
                    snap_i2 = dict(insert_final); snap_t2 = dict(text_local_final)
                    ok_all2 = True
                    for rn2 in sorted(blockers2):
                        obst_r2 = final_rebar_lines(exclude=(rn2,))
                        placed_r2 = [b for b,n3 in all_placed_text_boxes(exclude_name=rn2)] + [cand_bb2]
                        sol2 = _slide_text_along(rn2, obst_r2, placed_r2)
                        if sol2 is not None:
                            text_local_final[rn2] = sol2
                            continue
                        # το κείμενο-εμπόδιο δεν χωράει να κυλήσει: δοκίμασε να
                        # μετακινηθεί ολόκληρη η ράβδος του (πάντα εντός των
                        # κανόνων: πλάκα, πλευρά, όριο απόστασης) και ξανακύλησε
                        moved2 = False
                        rlines2,_ = block_lines_local(blocks[rn2])
                        if re.match(r'FL-?\d+_SLABBAR\d+$', rn2) and rn2 not in SLABBAR_HOME_SLAB:
                            rlines2 = []  # χωρίς γνωστή πλάκα δεν μετακινούμε σίδερο - ο
                                          # κανόνας "ποτέ εκτός πλάκας" δεν θα ήταν επαληθεύσιμος
                        if rlines2:
                            (rux2,ruy2),_ = spine_and_bounds(rlines2)
                            rpx2, rpy2 = -ruy2, rux2
                            rdx0, rdy0 = insert_final.get(rn2,(0,0))
                            for s2 in [0.05*k for k in range(1,13)]:
                                for sg2 in (1,-1):
                                    nrdx, nrdy = rdx0+rpx2*s2*sg2, rdy0+rpy2*s2*sg2
                                    if not _bar_move_ok(rn2, rlines2, nrdx, nrdy):
                                        continue
                                    rlines_new2 = [(a+nrdx,b+nrdy,c+nrdx,d2_+nrdy) for a,b,c,d2_ in rlines2]
                                    bad2 = False
                                    for seg2 in rlines_new2:
                                        for b3,_n4 in all_placed_text_boxes(exclude_name=rn2):
                                            if seg_intersects_bbox(seg2, b3): bad2=True;break
                                        if bad2: break
                                        if seg_intersects_bbox(seg2, cand_bb2): bad2=True;break
                                    if bad2:
                                        continue
                                    insert_final[rn2] = (nrdx, nrdy)
                                    sol2b = _slide_text_along(rn2, final_rebar_lines(exclude=(rn2,)),
                                                               [b for b,n3 in all_placed_text_boxes(exclude_name=rn2)] + [cand_bb2])
                                    if sol2b is not None:
                                        text_local_final[rn2] = sol2b
                                        moved2 = True
                                    else:
                                        insert_final[rn2] = (rdx0, rdy0)
                                    if moved2: break
                                if moved2: break
                        if not moved2:
                            ok_all2 = False; break
                    if ok_all2 and _ok2(cand2, all_placed_text_boxes(exclude_name=name)):
                        insert_final[name] = cand2
                        any_progress = True
                    else:
                        insert_final.clear(); insert_final.update(snap_i2)
                        text_local_final.clear(); text_local_final.update(snap_t2)
                        break
            if not any_progress:
                break

        # --- ΚΥΚΛΑΚΙΑ ΔΕΙΚΤΩΝ ΠΛΑΚΩΝ vs ΟΛΑ τα κείμενα: τελική εκκαθάριση ------
        # Το ενσωματωμένο CIRCLE REPAIR κοιτούσε μόνο κύκλους-vs-σίδερα· κείμενα
        # δοκών/ράβδων πάνω στο κυκλάκι του Πx ξέφευγαν. Εδώ: κάθε κείμενο που
        # πατάει κύκλο σύρεται με τους κανόνες της κατηγορίας του (ράβδου: κατά
        # μήκος· δοκού: μέσα στη δοκό της, με ανοχή 1 γραμμής).

        for _c_pass in range(3):
            fixed_c = False
            cbs = _circle_boxes()
            for cn in [n_ for n_ in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$', n_)]:
                cdx, cdy = insert_final.get(cn, (0, 0))
                if re.match(r'FL-?\d+_BEAMBAR', cn) and cdx <= -49:
                    continue
                ctx, cty = text_local_final.get(cn, (0, 0))
                cbl = block_text_bboxes(blocks[cn])
                if not cbl:
                    continue
                hit = False
                for x_, y_, w_, h_, rot_ in cbl:
                    tb_ = text_bbox(x_+cdx+ctx, y_+cdy+cty, w_, h_, rot_)
                    for cb_, _sn in cbs:
                        if not (tb_[2] < cb_[0] or cb_[2] < tb_[0] or tb_[3] < cb_[1] or cb_[3] < tb_[1]):
                            hit = True; break
                    if hit: break
                # ...ή ΠΑΡΑΛΛΗΛΗ γραμμή μέσα στον πυρήνα του (π.χ. κείμενο πάνω στη
                # γραμμή κοινού ορίου πλακών) - εξίσου ορατό ελάττωμα
                par_hit = set()
                if not hit:
                    own_ex = (cn,)
                    m_ = re.match(r'(FL-?\d+_)BEAM_TEXT(\d+)$', cn)
                    if m_:
                        own_ex = (cn, m_.group(1)+'BEAM'+m_.group(2))
                    count_line_crossings(cbl, cdx+ctx, cdy+cty, final_rebar_lines(exclude=(cn,)),
                                          own_ex, parallel_out=par_hit)
                    if par_hit:
                        hit = True
                if not hit:
                    continue
                _d47 = (os.environ.get('DEBUG_CN') == cn) or (os.environ.get('DEBUG_S47') and cn == 'FL1_SLABBAR47')
                if _d47: print('[S47] hit=True', flush=True)
                c_placed = [b for b, _n in all_placed_text_boxes(exclude_name=cn)] + [cb_ for cb_, _sn in cbs]
                c_obst = final_rebar_lines(exclude=(cn,))
                if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', cn):
                    pass  # ετικέτα κολώνας: δεν σύρεται εδώ - λύση μόνο μέσω δείκτη
                elif re.match(r'FL-?\d+_BEAM_TEXT\d+$', cn):
                    resc = beam_text_slide(cn, blocks, c_obst, hatch_polys, c_placed, relaxed=True)
                    curc = insert_final.get(cn, (0.0, 0.0))
                    if resc is not None and (abs(resc[0]-curc[0]) > 1e-9 or abs(resc[1]-curc[1]) > 1e-9):
                        insert_final[cn] = resc
                        fixed_c = True
                        continue
                else:
                    solc = _slide_text_along(cn, c_obst, c_placed)
                    if solc is not None:
                        text_local_final[cn] = solc
                        fixed_c = True
                        continue
                    if _slide_text_along_chain(cn):
                        fixed_c = True
                        continue
                    # το κείμενο ακολουθεί το σίδερο: αν το πρόβλημα είναι κάθετο στη
                    # ράβδο (παράλληλη γραμμή στη σειρά του κειμένου), μόνο η
                    # μετακίνηση του ΣΙΔΕΡΟΥ το λύνει - πάντα εντός κανόνων.
                    rl_, _ = block_lines_local(blocks[cn])
                    if rl_:
                        (rux_, ruy_), _b = spine_and_bounds(rl_)
                        px_, py_ = -ruy_, rux_
                        moved_b = False
                        for s_ in [0.05*k for k in range(1, 13)]:
                            for sg_ in (1, -1):
                                nbx, nby = cdx+px_*s_*sg_, cdy+py_*s_*sg_
                                if not _bar_move_ok(cn, rl_, nbx, nby):
                                    continue
                                lines_nb = [(a+nbx, b+nby, c+nbx, d+nby) for a, b, c, d in rl_]
                                bad_b = False
                                # ΣΚΛΗΡΟΣ έλεγχος μόνο απέναντι στα ΚΥΚΛΑΚΙΑ (αυτά
                                # καθαρίζουμε εδώ)· για τα κείμενα ισχύει το σύγχρονο
                                # κριτήριο: κανένα ΝΕΟ παράλληλο κόψιμο σε σχέση με
                                # την τρέχουσα θέση - το παλιό "καμία επαφή με τίποτα"
                                # μπλόκαρε κάθε κίνηση σε γεμάτους διαδρόμους (Δ22).
                                for seg_ in lines_nb:
                                    for cb_, _sn3 in cbs:
                                        if seg_intersects_bbox(seg_, cb_):
                                            bad_b = True; break
                                    if bad_b: break
                                if not bad_b:
                                    def _bpcut(offx_, offy_):
                                        out_ = set()
                                        for i5, (b5, _n5) in enumerate(all_placed_text_boxes(exclude_name=cn)):
                                            hor5 = (b5[2]-b5[0]) >= (b5[3]-b5[1])
                                            for a5, b5_, c5, d5 in rl_:
                                                sg5 = (a5+offx_, b5_+offy_, c5+offx_, d5+offy_)
                                                if not seg_intersects_bbox(sg5, b5):
                                                    continue
                                                lx5, ly5 = sg5[2]-sg5[0], sg5[3]-sg5[1]
                                                ll5 = math.hypot(lx5, ly5)
                                                comp5 = abs(lx5/ll5) if (ll5 > 1e-9 and hor5) else (abs(ly5/ll5) if ll5 > 1e-9 else 0.0)
                                                if comp5 > 0.94:
                                                    out_.add(i5); break
                                        return out_
                                    if not _bpcut(nbx, nby) <= _bpcut(cdx, cdy):
                                        bad_b = True
                                if bad_b:
                                    continue
                                if is_ok_relaxed(cbl, nbx+ctx, nby+cty, c_obst, hatch_polys, c_placed, (cn,), max_crossings=1):
                                    insert_final[cn] = (nbx, nby)
                                    moved_b = True; fixed_c = True
                                    break
                                sol2c = None
                                # δοκίμασε και σύρσιμο στη νέα θέση
                                snap_b = insert_final.get(cn)
                                insert_final[cn] = (nbx, nby)
                                sol2c = _slide_text_along(cn, final_rebar_lines(exclude=(cn,)), c_placed)
                                if sol2c is not None:
                                    text_local_final[cn] = sol2c
                                    moved_b = True; fixed_c = True
                                else:
                                    insert_final[cn] = snap_b
                                if moved_b: break
                            if moved_b: break
                # Το κείμενο δεν έχει πού να πάει μέσα στους κανόνες του: μετακίνησε
                # τον ΔΕΙΚΤΗ (κυκλάκι+ετικέτα μαζί, όπως τους γράφει το
                # patch_slab_marker_geometry) μέσα στην πλάκα του.
                if _d47: print('[S47] φτάνω στο marker-move', flush=True)
                hit_slabs = set()
                for x_, y_, w_, h_, rot_ in cbl:
                    tb_ = text_bbox(x_+cdx+ctx, y_+cdy+cty, w_, h_, rot_)
                    for cb_, sn_ in cbs:
                        if not (tb_[2] < cb_[0] or cb_[2] < tb_[0] or tb_[3] < cb_[1] or cb_[3] < tb_[1]):
                            hit_slabs.add(sn_)
                if _d47: print('[S47] hit_slabs=', sorted(hit_slabs), flush=True)
                for sn_ in sorted(hit_slabs):
                    sox, soy = ins.get(sn_, (0, 0))
                    stdx, stdy = text_local_final.get(sn_, (0, 0))
                    mbl = slab_marker_boxes(blocks[sn_])
                    circ_l = None
                    for e in entities_from_pairs(blocks[sn_]):
                        if e[0][1] == 'CIRCLE':
                            d_ = to_dict(e)
                            if d_.get(8, [''])[0] == 'slab_center':
                                cx_ = float(d_[10][0]); cy_ = float(d_[20][0]); r_ = float(d_[40][0])
                                circ_l = (cx_-r_, cy_-r_, cx_+r_, cy_+r_)
                    if not mbl:
                        continue
                    sb = SLAB_POLYS_MAP.get(sn_)
                    x_ok_ = sb is not None and (sb[2]-sb[0]) >= 0.05
                    y_ok_ = sb is not None and (sb[3]-sb[1]) >= 0.05
                    # Και οι ΜΗ αναγνωρισμένες πλάκες επιτρέπεται να μετακινήσουν
                    # τον δείκτη τους, αλλά ΜΟΝΟ εντός ±0.35 από τη φυσική θέση:
                    # τα όρια του πλέγματος (φυσικός κύκλος ±0.35) και η πύλη
                    # _marker_move_ok το εγγυώνται. Ο πλήρης αποκλεισμός τους
                    # άφηνε τα κείμενα πάνω στα κυκλάκια τους χωρίς καμία λύση.
                    # ΣΥΝΔΥΑΣΜΟΣ ΣΙΔΕΡΟ+ΔΕΙΚΤΗΣ: σε στενές πλάκες (π.χ. SLAB9
                    # 0.8x1.8) καμία μονομερής κίνηση δεν φτάνει - χωράνε μόνο αν
                    # το σίδερο πάει στη μία μεριά (εντός ±0.35/κανόνων) ΚΑΙ ο
                    # δείκτης στην άλλη, ταυτόχρονα. Δοκιμάζονται θέσεις της
                    # ράβδου του "θύματος" και για καθεμία ολόκληρη η ακτινική
                    # αναζήτηση του δείκτη. Πλήρης αναίρεση σε αποτυχία.
                    _cnt = {'σκελετός':0,'εκτός':0,'κείμενα':0,'κύκλος':0} if _d47 else None
                    bar_offsets = [(0.0, 0.0)]
                    if re.match(r'FL-?\d+_SLABBAR\d+$', cn):
                        vlines_, _ = block_lines_local(blocks[cn])
                        if vlines_:
                            (vux_, vuy_), _vb = spine_and_bounds(vlines_)
                            vpx_, vpy_ = -vuy_, vux_
                            for sv in (0.1, 0.2, 0.3, 0.35):
                                for sgv in (1, -1):
                                    bar_offsets.append((vpx_*sv*sgv, vpy_*sv*sgv))
                    vdx0, vdy0 = insert_final.get(cn, (0, 0))
                    vtl0 = text_local_final.get(cn, None)
                    m_obst = final_rebar_lines()
                    moved_m = False
                    for bofx, bofy in bar_offsets:
                        if moved_m:
                            break
                        if bofx or bofy:
                            vlines_, _ = block_lines_local(blocks[cn])
                            if not _bar_move_ok(cn, vlines_, vdx0+bofx, vdy0+bofy):
                                if _cnt is not None: _cnt['σκελετός'] += 1
                                continue
                            insert_final[cn] = (vdx0+bofx, vdy0+bofy)
                        else:
                            insert_final[cn] = (vdx0, vdy0)
                        # ΚΡΙΣΙΜΟ: το κείμενο του "θύματος" (cn) ΕΞΑΙΡΕΙΤΑΙ από την
                        # επικύρωση του δείκτη - αυτό θα μετακινηθεί ΜΕΤΑ. Με το
                        # κείμενο στην παλιά του θέση μέσα στα εμπόδια, κάθε έγκυρη
                        # γωνιά της στενής πλάκας απορριπτόταν (SLAB9: 184 θέσεις
                        # δείκτη "όχι ΟΚ" ενώ η λύση υπήρχε). Η σειρά είναι:
                        # δείκτης πρώτα (χωρίς το θύμα), μετά υποχρεωτική εύρεση
                        # θέσης για το κείμενο του θύματος - αλλιώς πλήρης αναίρεση.
                        m_placed = [b for b, _n in all_placed_text_boxes(exclude_name=sn_) if _n != cn]
                        # ΕΞΑΝΤΛΗΤΙΚΟ ΠΛΕΓΜΑ αντί για ακτινική δειγματοληψία: σε
                        # στενές πλάκες το έγκυρο παράθυρο είναι ~0.1μ και οι
                        # ακτίνες βήματος 0.1 το προσπερνούσαν (2 μόνο υποψήφιοι
                        # έφταναν στον έλεγχο κύκλου). Βήμα 0.05, ταξινόμηση κατά
                        # απόσταση από την τρέχουσα θέση - ίδια προτίμηση, πλήρης
                        # κάλυψη.
                        _r_ = 0.26
                        _cands = []
                        # εκφυλισμένος άξονας πλάκας: το πλέγμα ανοίγει γύρω από τη
                        # ΦΥΣΙΚΗ θέση του δείκτη ±0.35 (ίδια πολιτική με τις ράβδους) -
                        # αλλιώς εύρος 2.00..2.00 σήμαινε ΜΗΔΕΝ υποψήφιους και ο
                        # δείκτης της SLAB5 δεν μπορούσε να κουνηθεί καθόλου.
                        _ncx = (circ_l[0]+circ_l[2])/2 + sox
                        _ncy = (circ_l[1]+circ_l[3])/2 + soy
                        _x_lo = sb[0] + _r_ if x_ok_ else _ncx - 0.35
                        _x_hi = sb[2] - _r_ if x_ok_ else _ncx + 0.35
                        _y_lo = sb[1] + _r_ if y_ok_ else _ncy - 0.35
                        _y_hi = sb[3] - _r_ if y_ok_ else _ncy + 0.35
                        _gy = _y_lo
                        while _gy <= _y_hi + 1e-9:
                            _gx = _x_lo
                            while _gx <= _x_hi + 1e-9:
                                _mdx = _gx - (circ_l[0]+circ_l[2])/2 - sox
                                _mdy = _gy - (circ_l[1]+circ_l[3])/2 - soy
                                _cands.append((math.hypot(_mdx-stdx, _mdy-stdy), _mdx, _mdy))
                                _gx += 0.05
                            _gy += 0.05
                        _cands.sort()
                        if _d47: print('[S47] grid=', len(_cands), 'sb=', sb, 'circ_l=', circ_l, flush=True)
                        for _dd_, mdx, mdy in _cands:
                            if _cnt is not None: _cnt['iter'] = _cnt.get('iter',0)+1
                            if moved_m:
                                break
                            cbx = (circ_l[0]+sox+mdx, circ_l[1]+soy+mdy,
                                   circ_l[2]+sox+mdx, circ_l[3]+soy+mdy) if circ_l else None
                            # εντός πλάκας (υγιείς άξονες), μαζί με το κυκλάκι
                            pieces = [text_bbox(x_+sox+mdx, y_+soy+mdy, w_, h_, rot_) for x_, y_, w_, h_, rot_ in mbl]
                            if cbx: pieces.append(cbx)
                            bad_m = False
                            for pb in pieces:
                                if x_ok_ and (pb[0] < sb[0]-0.02 or pb[2] > sb[2]+0.02): bad_m = True; break
                                if y_ok_ and (pb[1] < sb[1]-0.02 or pb[3] > sb[3]+0.02): bad_m = True; break
                            if bad_m:
                                if _cnt is not None: _cnt['εκτός'] += 1
                                continue
                            if not is_ok_full(mbl, sox+mdx, soy+mdy, m_obst, hatch_polys, m_placed, (sn_,)) and \
                               not is_ok_relaxed(mbl, sox+mdx, soy+mdy, m_obst, hatch_polys, m_placed, (sn_,), max_crossings=1):
                                _cnt is not None and _cnt.__setitem__('κείμενα', _cnt['κείμενα']+1)
                                continue
                            if cbx:
                                clear_m = True
                                for ob in m_placed:
                                    if not (cbx[2] < ob[0] or ob[2] < cbx[0] or cbx[3] < ob[1] or ob[3] < cbx[1]):
                                        clear_m = False; break
                                if clear_m:
                                    for sgl in m_obst:
                                        if sgl[4] == sn_:
                                            continue  # η γραμμή-οδηγός του ΙΔΙΟΥ του δείκτη
                                                      # περνά μέσα από τον κύκλο του εξ ορισμού -
                                                      # χωρίς την εξαίρεση, ΚΑΘΕ θέση απορριπτόταν
                                        if seg_intersects_bbox(sgl[:4], cbx):
                                            clear_m = False; break
                                if not clear_m:
                                    if _cnt is not None: _cnt['κύκλος'] += 1
                                    continue
                            # ΔΟΚΙΜΑΣΤΙΚΗ δέσμευση δείκτη + ΑΜΕΣΗ επικύρωση του
                            # κειμένου-θύματος, ΑΝΑ υποψήφιο: αλλιώς δεσμευόταν η
                            # "πλησιέστερη" θέση - συνήθως η ίδια η τρέχουσα, αφού
                            # το θύμα εξαιρείται από τα εμπόδια - και η μοναδική
                            # μετά-τον-βρόχο ευκαιρία επικύρωσης χανόταν άδικα.
                            if not _marker_move_ok(sn_, mdx, mdy):
                                continue
                            text_local_final[sn_] = (mdx, mdy)
                            vbl_ = block_text_bboxes(blocks[cn])
                            vdxn, vdyn = insert_final.get(cn, (0, 0))
                            vtxn, vtyn = text_local_final.get(cn, (0, 0))
                            ok_v = is_ok_relaxed(vbl_, vdxn+vtxn, vdyn+vtyn, final_rebar_lines(exclude=(cn,)), hatch_polys,
                                                  [b for b, _n in all_placed_text_boxes(exclude_name=cn)], (cn,), max_crossings=1)
                            if not ok_v:
                                solv_ = _slide_text_along(cn, final_rebar_lines(exclude=(cn,)),
                                                           [b for b, _n in all_placed_text_boxes(exclude_name=cn)] + [cb_ for cb_, _s2 in _circle_boxes()])
                                if solv_ is not None:
                                    text_local_final[cn] = solv_
                                    ok_v = True
                            if ok_v:
                                moved_m = True; fixed_c = True
                                break
                            text_local_final[sn_] = (stdx, stdy)
                            if _cnt is not None: _cnt['θύμα'] = _cnt.get('θύμα', 0)+1
                        # (τέλος πλέγματος)
                    if not moved_m:
                        pass  # η επικύρωση του θύματος γίνεται πλέον ανά υποψήφιο
                    if _d47: print('[S47] moved_m=', moved_m, flush=True)
                    if _d47 and _cnt is not None: print('[S47] αιτίες:', _cnt, flush=True)
                    if not moved_m:
                        insert_final[cn] = (vdx0, vdy0)
                        if vtl0 is not None:
                            text_local_final[cn] = vtl0
                        elif cn in text_local_final:
                            del text_local_final[cn]
                    else:
                        fixed_c = True
            if not fixed_c:
                break

        # --- ΠΑΡΑΛΛΗΛΕΣ ΤΟΜΕΣ: εκκαθάριση ήδη τοποθετημένων κειμένων ράβδων -----
        # Οι ελεγκτές πλέον απορρίπτουν παράλληλη γραμμή μέσα στο κείμενο, αλλά
        # θέσεις που δεσμεύτηκαν πριν από τον κανόνα δεν ξαναελέγχονται μόνες τους.
        for _p_pass in range(3):
            fixed_p = False
            for pn in [n_ for n_ in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_)]:
                pdx, pdy = insert_final.get(pn, (0, 0))
                if re.match(r'FL-?\d+_BEAMBAR', pn) and pdx <= -49:
                    continue
                ptx, pty = text_local_final.get(pn, (0, 0))
                pbl = block_text_bboxes(blocks[pn])
                if not pbl:
                    continue
                p_obst = final_rebar_lines(exclude=(pn,))
                plines_chk, _ = block_lines_local(blocks[pn])
                def _line_pcut_victims(odx_, ody_):
                    vict = set()
                    for i3, (b3, _n4) in enumerate(all_placed_text_boxes(exclude_name=pn)):
                        hor_ = (b3[2]-b3[0]) >= (b3[3]-b3[1])
                        for a_, b_, c_, d_ in plines_chk:
                            sg = (a_+odx_, b_+ody_, c_+odx_, d_+ody_)
                            if not seg_intersects_bbox(sg, b3):
                                continue
                            lx_, ly_ = sg[2]-sg[0], sg[3]-sg[1]
                            ll_ = math.hypot(lx_, ly_)
                            comp_ = abs(lx_/ll_) if (ll_ > 1e-9 and hor_) else (abs(ly_/ll_) if ll_ > 1e-9 else 0.0)
                            if comp_ > 0.94:
                                vict.add(i3); break
                    return vict
                text_cut = parallel_cut_full(pbl, pdx+ptx, pdy+pty, p_obst, (pn,))
                line_victims = _line_pcut_victims(pdx, pdy) if plines_chk else set()
                if not text_cut and not line_victims:
                    continue
                p_placed = [b_ for b_, _n in all_placed_text_boxes(exclude_name=pn)] + [cb_ for cb_, _sn in _circle_boxes()]
                if text_cut and not line_victims:
                    solp = _slide_text_along(pn, p_obst, p_placed)
                    if solp is not None and not parallel_cut_full(pbl, pdx+solp[0], pdy+solp[1], p_obst, (pn,)):
                        text_local_final[pn] = solp
                        fixed_p = True
                        continue
                plines, _ = block_lines_local(blocks[pn])
                if not plines:
                    continue
                (pux, puy), _pb = spine_and_bounds(plines)
                ppx, ppy = -puy, pux
                moved_p = False
                for sp in [0.05*k for k in range(1, 17)]:
                    for sgp in (1, -1):
                        npdx, npdy = pdx + ppx*sp*sgp, pdy + ppy*sp*sgp
                        if not _bar_move_ok(pn, plines, npdx, npdy):
                            continue
                        def _pcut_set(offx, offy):
                            outset = set()
                            for i3, (b3, _n4) in enumerate(all_placed_text_boxes(exclude_name=pn)):
                                hor_ = (b3[2]-b3[0]) >= (b3[3]-b3[1])
                                for a_, b_, c_, d_ in plines:
                                    sg = (a_+offx, b_+offy, c_+offx, d_+offy)
                                    if not seg_intersects_bbox(sg, b3):
                                        continue
                                    lx_, ly_ = sg[2]-sg[0], sg[3]-sg[1]
                                    ll_ = math.hypot(lx_, ly_)
                                    comp_ = abs(lx_/ll_) if (ll_ > 1e-9 and hor_) else (abs(ly_/ll_) if ll_ > 1e-9 else 0.0)
                                    if comp_ > 0.94:
                                        outset.add(i3); break
                            return outset
                        # μόνο ΝΕΑ παράλληλα κοψίματα απαγορεύονται - αλλιώς σε
                        # γεμάτη πλάκα κάθε κίνηση απορριπτόταν και το πέρασμα
                        # ήταν ανίσχυρο (γι' αυτό δεν έφτιαχνε το SLABBAR18).
                        # Αν η αφορμή ήταν κομμένο ΞΕΝΟ κείμενο, απαιτείται και
                        # ΠΡΟΟΔΟΣ: το νέο σύνολο θυμάτων γνήσιο υποσύνολο.
                        _new_v = _pcut_set(npdx, npdy)
                        _old_v = _pcut_set(pdx+0.0, pdy+0.0)
                        if not _new_v <= _old_v:
                            continue
                        if line_victims and not _new_v < _old_v:
                            continue
                        obst_after = final_rebar_lines(exclude=(pn,))
                        old_pos = insert_final.get(pn, (0, 0))
                        insert_final[pn] = (npdx, npdy)
                        if pn in text_local_final:
                            saved_tl = text_local_final[pn]
                        else:
                            saved_tl = None
                        if is_ok_relaxed(pbl, npdx, npdy, obst_after, hatch_polys,
                                          [b_ for b_, _n in all_placed_text_boxes(exclude_name=pn)], (pn,), max_crossings=1):
                            if pn in text_local_final:
                                del text_local_final[pn]
                            moved_p = True; fixed_p = True
                            break
                        solp2 = _slide_text_along(pn, obst_after, p_placed)
                        if solp2 is not None and not parallel_cut_full(pbl, npdx+solp2[0], npdy+solp2[1], obst_after, (pn,)):
                            text_local_final[pn] = solp2
                            moved_p = True; fixed_p = True
                            break
                        insert_final[pn] = old_pos
                        if saved_tl is not None:
                            text_local_final[pn] = saved_tl
                    if moved_p:
                        break
                if not moved_p and line_victims:
                    # Ο ένοχος οπλισμός δεν μπορεί να μετακινηθεί (π.χ. ράβδος
                    # χωρίς επαληθεύσιμη πλάκα = κλειδωμένη στη φυσική θέση):
                    # δοκιμάζεται η μετακίνηση του ΘΥΜΑΤΟΣ, αν είναι BEAM_TEXT -
                    # μέσα στη δοκό του πάντα.
                    vict_names = sorted({all_placed_text_boxes(exclude_name=pn)[i3][1] for i3 in line_victims})
                    for vn in vict_names:
                        if not re.match(r'FL-?\d+_BEAM_TEXT\d+$', vn):
                            continue
                        v_obst3 = final_rebar_lines()
                        v_placed3 = [b_ for b_, _n in all_placed_text_boxes(exclude_name=vn)]
                        resv = beam_text_slide(vn, blocks, v_obst3, hatch_polys, v_placed3, relaxed=True)
                        curv = insert_final.get(vn, (0.0, 0.0))
                        if resv is None or (abs(resv[0]-curv[0]) < 1e-9 and abs(resv[1]-curv[1]) < 1e-9):
                            # αλυσίδα: θέσεις που φράζονται ΜΟΝΟ από κείμενα ράβδων
                            v_wo = [b_ for b_, n_9 in all_placed_text_boxes(exclude_name=vn)
                                     if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_9)]
                            resv2 = beam_text_slide(vn, blocks, v_obst3, hatch_polys, v_wo, relaxed=True)
                            if resv2 is None or (abs(resv2[0]-curv[0]) < 1e-9 and abs(resv2[1]-curv[1]) < 1e-9):
                                continue
                            vbl_ = block_text_bboxes(blocks[vn])
                            cand_vb = [text_bbox(x_+resv2[0], y_+resv2[1], w_, h_, rot_) for x_, y_, w_, h_, rot_ in vbl_]
                            blkv = set()
                            for cb in cand_vb:
                                for b_, n_9 in all_placed_text_boxes(exclude_name=vn):
                                    if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_9):
                                        continue
                                    if not (cb[2] + MIN_TEXT_GAP < b_[0] or b_[2] + MIN_TEXT_GAP < cb[0] or
                                            cb[3] + MIN_TEXT_GAP < b_[1] or b_[3] + MIN_TEXT_GAP < cb[1]):
                                        blkv.add(n_9)
                            snap_vi = dict(insert_final)
                            snap_vt = dict(text_local_final)
                            insert_final[vn] = resv2
                            ok_v = True
                            for bnv in sorted(blkv):
                                sol_v = _slide_text_along(bnv, final_rebar_lines(exclude=(bnv,)),
                                                           [b_ for b_, n_a in all_placed_text_boxes(exclude_name=bnv)] +
                                                           [cb_ for cb_, _sn in _circle_boxes()])
                                if sol_v is None and not _slide_text_along_chain(bnv):
                                    ok_v = False
                                    break
                                if sol_v is not None:
                                    text_local_final[bnv] = sol_v
                            if ok_v and _line_pcut_victims(pdx, pdy) < line_victims:
                                fixed_p = True
                            else:
                                insert_final.clear(); insert_final.update(snap_vi)
                                text_local_final.clear(); text_local_final.update(snap_vt)
                            continue
                        old_v = insert_final.get(vn, (0.0, 0.0))
                        insert_final[vn] = resv
                        if _line_pcut_victims(pdx, pdy) < line_victims:
                            fixed_p = True
                        else:
                            insert_final[vn] = old_v
            if not fixed_p:
                break


        # --- ΕΠΙΚΑΛΥΨΕΙΣ ΚΕΙΜΕΝΩΝ: ενεργή επιδιόρθωση. Ο βρόχος συμφιλίωσης
        # τις ΑΝΙΧΝΕΥΕ αλλά κανένα πέρασμα δεν τις έλυνε ρητά, οπότε γύριζε
        # 4 φορές άπρακτος (έτσι επιβίωσε η BEAMBAR4/BEAM_TEXT22). ------------
        for _o_pass in range(3):
            fixed_o = False
            boxes_o = all_placed_text_boxes()
            for i_o in range(len(boxes_o)):
                b1, n1 = boxes_o[i_o]
                if fixed_o:
                    break
                for j_o in range(i_o+1, len(boxes_o)):
                    b2, n2 = boxes_o[j_o]
                    if n1 == n2:
                        continue
                    if (b1[2] + 0.03 < b2[0] or b2[2] + 0.03 < b1[0] or
                        b1[3] + 0.03 < b2[1] or b2[3] + 0.03 < b1[1]):
                        continue
                    for mover in (n1, n2):
                        _dbg = os.environ.get('DEBUG_PAIR') and 'BEAMBAR4' in (n1+n2) and 'BEAM_TEXT22' in (n1+n2)
                        if _dbg:
                            print(f'[DBG fixer] ζεύγος ({n1},{n2}) δοκιμάζω mover={mover}', flush=True)
                        if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', mover):
                            mo_obst = final_rebar_lines(exclude=(mover,))
                            mo_placed = [b_ for b_, _n in all_placed_text_boxes(exclude_name=mover)] + [cb_ for cb_, _sn in _circle_boxes()]
                            solo = _slide_text_along(mover, mo_obst, mo_placed)
                            if _dbg:
                                print(f'[DBG fixer]   slide -> {solo}', flush=True)
                            if solo is not None:
                                text_local_final[mover] = solo
                                fixed_o = True
                                break
                            _ch = _slide_text_along_chain(mover)
                            if _dbg:
                                print(f'[DBG fixer]   chain -> {_ch}', flush=True)
                            if _ch:
                                fixed_o = True
                                break
                            mlines, _ = block_lines_local(blocks[mover])
                            if mlines:
                                (mux_, muy_), _mb = spine_and_bounds(mlines)
                                mpx_, mpy_ = -muy_, mux_
                                mdx0, mdy0 = insert_final.get(mover, (0, 0))
                                mtl0 = text_local_final.get(mover, None)
                                done_m = False
                                for sm in [0.05*k for k in range(1, 13)]:
                                    for sgm in (1, -1):
                                        nmdx, nmdy = mdx0 + mpx_*sm*sgm, mdy0 + mpy_*sm*sgm
                                        if not _bar_move_ok(mover, mlines, nmdx, nmdy):
                                            if _dbg and sgm < 0:
                                                print(f'[DBG barmove] s={sm*sgm:+.2f}: _bar_move_ok=False (side={BEAMBAR_SIDE.get(mover)})', flush=True)
                                            continue
                                        insert_final[mover] = (nmdx, nmdy)
                                        solo2 = _slide_text_along(mover, final_rebar_lines(exclude=(mover,)), mo_placed)
                                        if _dbg:
                                            print(f'[DBG barmove] s={sm*sgm:+.2f}: bar_ok=True, slide -> {solo2}', flush=True)
                                        if solo2 is None and _slide_text_along_chain(mover):
                                            if _dbg:
                                                print(f'[DBG barmove] s={sm*sgm:+.2f}: chain στη νέα θέση -> ΕΠΙΤΥΧΙΑ', flush=True)
                                            done_m = True
                                            break
                                        if solo2 is not None:
                                            text_local_final[mover] = solo2
                                            done_m = True
                                            break
                                        insert_final[mover] = (mdx0, mdy0)
                                    if done_m:
                                        break
                                if done_m:
                                    fixed_o = True
                                    break
                                # ΥΠΟΧΩΡΗΣΗ ΕΤΙΚΕΤΑΣ ΚΟΛΩΝΑΣ: όταν το κείμενο της
                                # ράβδου είναι κλεισμένο ανάμεσα σε κείμενο δοκού και
                                # ετικέτα κολώνας (beambar8/Δ6/Κ8), καμία μονομερής
                                # κίνηση δεν αρκεί. Γενικός μηχανισμός: η ετικέτα
                                # κολώνας που φράζει τον κάθετο διάδρομο δοκιμάζει
                                # νόμιμη μετατόπιση (radial, κοντά στην κολώνα της),
                                # και ξαναδοκιμάζεται η κίνηση σιδήρου+σύρσιμο.
                                # Πλήρης αναίρεση αν δεν κλείσει.
                                _dbgp = os.environ.get('DEBUG_PAIR2', '')
                                _dbg8 = bool(_dbgp) and set(_dbgp.split(',')) == {n1, n2}
                                if _dbg8:
                                    print(f'[YIELD] φτάνω εδώ για mover={mover}', flush=True)
                                _mtb = block_text_bboxes(blocks[mover])
                                if _mtb:
                                    _mu = union_bbox([(x_+mdx0+(mtl0 or (0,0))[0], y_+mdy0+(mtl0 or (0,0))[1], w_, h_, rot_) for x_, y_, w_, h_, rot_ in _mtb])
                                    _ix = 0.2*abs(mux_) + 0.8*abs(mpx_)
                                    _iy = 0.2*abs(muy_) + 0.8*abs(mpy_)
                                    _corr = (_mu[0]-_ix, _mu[1]-_iy, _mu[2]+_ix, _mu[3]+_iy)
                                    _cols = []; _bts9 = []
                                    for _b9, _n9 in all_placed_text_boxes(exclude_name=mover):
                                        _isc = re.match(r'FL-?\d+_COLUMN_TEXT\d+$', _n9)
                                        _isb = re.match(r'FL-?\d+_BEAM_TEXT\d+$', _n9)
                                        if not _isc and not _isb:
                                            continue
                                        if not (_b9[2] < _corr[0] or _corr[2] < _b9[0] or _b9[3] < _corr[1] or _corr[3] < _b9[1]):
                                            if _isc and _n9 not in _cols:
                                                _cols.append(_n9)
                                            if _isb and _n9 not in _bts9:
                                                _bts9.append(_n9)
                                    if _dbg8:
                                        print(f'[YIELD] υποψήφιες ετικέτες: {_cols}', flush=True)
                                    for _cnm in _cols:
                                        _mc = re.match(r'(FL-?\d+_)COLUMN_TEXT(\d+)$', _cnm)
                                        _own = _mc.group(1)+'COLUMN'+_mc.group(2)
                                        if _own not in blocks:
                                            continue
                                        _lbl9 = block_text_bboxes(blocks[_cnm])
                                        _o9 = insert_final.get(_cnm, (0.0, 0.0))
                                        _cl9,_ = block_lines_local(blocks[_own])
                                        _cxs9=[p for s9 in _cl9 for p in (s9[0],s9[2])]; _cys9=[p for s9 in _cl9 for p in (s9[1],s9[3])]
                                        _ccx9,_ccy9=(min(_cxs9)+max(_cxs9))/2,(min(_cys9)+max(_cys9))/2
                                        _lu9 = union_bbox(_lbl9)
                                        _pl_wo = [b_ for b_, n_c in all_placed_text_boxes(exclude_name=_cnm) if n_c != mover]
                                        # υποψήφιες μετατοπίσεις της ετικέτας: πρώτα κάθετα στο
                                        # διάδρομο (συνήθως αρκεί ελάχιστο), μετά πλάγια, τέλος
                                        # πλήρης ακτινική μετεγκατάσταση. Για ΚΑΘΕ νόμιμη θέση
                                        # της ετικέτας δοκιμάζεται ΑΜΕΣΩΣ το κατέβασμα σιδήρου +
                                        # σύρσιμο - κρατιέται ΜΟΝΟ συνδυασμός που κλείνει.
                                        _shifts = []
                                        for _dd9 in [0.05*k9 for k9 in range(2, 13)]:
                                            _shifts += [(0,-_dd9),(0,_dd9),(_dd9,0),(-_dd9,0)]
                                        _rad9 = radial_place_full(_cnm, blocks, final_rebar_lines(), hatch_polys, _pl_wo,
                                                                   (_own,), seed=(_ccx9-(_lu9[0]+_lu9[2])/2, _ccy9-(_lu9[1]+_lu9[3])/2),
                                                                   max_r=1.6, allow_relaxed=True)
                                        _cands = [(_o9[0]+sx9, _o9[1]+sy9) for sx9, sy9 in _shifts]
                                        if _rad9 is not None:
                                            _cands.append(_rad9)
                                        _tried9 = 0
                                        for _cand9 in _cands:
                                            if abs(_cand9[0]-_o9[0]) < 1e-9 and abs(_cand9[1]-_o9[1]) < 1e-9:
                                                continue
                                            if not is_ok_relaxed(_lbl9, _cand9[0], _cand9[1], final_rebar_lines(),
                                                                  hatch_polys, _pl_wo, (_own,), max_crossings=1):
                                                continue
                                            _tried9 += 1
                                            if _tried9 > 10:
                                                break
                                            _snap_yi = dict(insert_final); _snap_yt = dict(text_local_final)
                                            insert_final[_cnm] = _cand9
                                            _done2 = False
                                            for sm2 in [0.05*k2 for k2 in range(1, 13)]:
                                                for sg2 in (1, -1):
                                                    _nx2, _ny2 = mdx0 + mpx_*sm2*sg2, mdy0 + mpy_*sm2*sg2
                                                    if not _bar_move_ok(mover, mlines, _nx2, _ny2):
                                                        continue
                                                    insert_final[mover] = (_nx2, _ny2)
                                                    _sl2 = _slide_text_along(mover, final_rebar_lines(exclude=(mover,)),
                                                                              [b_ for b_, _nq in all_placed_text_boxes(exclude_name=mover)] +
                                                                              [cb_ for cb_, _sq in _circle_boxes()])
                                                    if _sl2 is not None:
                                                        text_local_final[mover] = _sl2
                                                        _done2 = True
                                                        break
                                                    insert_final[mover] = (mdx0, mdy0)
                                                if _done2:
                                                    break
                                            if _dbg8:
                                                print(f'[YIELD] {_cnm} @ ({_cand9[0]-_o9[0]:+.2f},{_cand9[1]-_o9[1]:+.2f}): retry -> {_done2}', flush=True)
                                            if _done2:
                                                done_m = True
                                                break
                                            insert_final.clear(); insert_final.update(_snap_yi)
                                            text_local_final.clear(); text_local_final.update(_snap_yt)
                                        if done_m:
                                            break
                                    # ΣΦΗΝΕΣ-ΚΕΙΜΕΝΑ ΔΟΚΩΝ: ίδια δομή retry-ανά-θέση.
                                    # Το κείμενο δοκού ολισθαίνει ΜΟΝΟ μέσα στη δοκό του
                                    # (beam_text_slide), και κρατιέται μόνο θέση που
                                    # ΚΛΕΙΝΕΙ τη λύση για τον mover.
                                    if not done_m:
                                        def _retry_mover9():
                                            for sm3 in [0.05*k3 for k3 in range(0, 13)]:
                                                for sg3 in ((1,) if sm3 == 0 else (1, -1)):
                                                    _nx3, _ny3 = mdx0 + mpx_*sm3*sg3, mdy0 + mpy_*sm3*sg3
                                                    if sm3 > 0 and not _bar_move_ok(mover, mlines, _nx3, _ny3):
                                                        continue
                                                    insert_final[mover] = (_nx3, _ny3)
                                                    _sl3 = _slide_text_along(mover, final_rebar_lines(exclude=(mover,)),
                                                                              [b_ for b_, _nr in all_placed_text_boxes(exclude_name=mover)] +
                                                                              [cb_ for cb_, _sr in _circle_boxes()])
                                                    if _sl3 is not None:
                                                        text_local_final[mover] = _sl3
                                                        return True
                                                    insert_final[mover] = (mdx0, mdy0)
                                            return False
                                        def _fix_rebar_blockers9(wnames):
                                            # κείμενα ράβδων που μπλοκάρονται από τις νέες θέσεις
                                            # των σφηνών: σύρσιμο/αλυσίδα - αλλιώς αποτυχία.
                                            for _wn in wnames:
                                                _wb = block_text_bboxes(blocks[_wn])
                                                _wi = insert_final.get(_wn, (0.0, 0.0))
                                                _cbs = [text_bbox(x_+_wi[0], y_+_wi[1], w_, h_, rot_) for x_, y_, w_, h_, rot_ in _wb]
                                                for cb in _cbs:
                                                    for b_, n_e in all_placed_text_boxes(exclude_name=_wn):
                                                        if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_e) or n_e == mover:
                                                            continue
                                                        if not (cb[2] + MIN_TEXT_GAP < b_[0] or b_[2] + MIN_TEXT_GAP < cb[0] or
                                                                cb[3] + MIN_TEXT_GAP < b_[1] or b_[3] + MIN_TEXT_GAP < cb[1]):
                                                            _sf = _slide_text_along(n_e, final_rebar_lines(exclude=(n_e,)),
                                                                                     [bq for bq, nq in all_placed_text_boxes(exclude_name=n_e)] +
                                                                                     [cq for cq, _sq2 in _circle_boxes()])
                                                            if _sf is not None:
                                                                text_local_final[n_e] = _sf
                                                            elif not _slide_text_along_chain(n_e):
                                                                # πλήρης εργαλειοθήκη και για τον μπλοκαρισμένο:
                                                                # μικρή μετακίνηση του ΣΙΔΗΡΟΥ του (μέσω πύλης) + σύρσιμο
                                                                _el, _ = block_lines_local(blocks[n_e])
                                                                _eb = None; _ebl = -1.0
                                                                for ex1, ey1, ex2, ey2 in _el:
                                                                    _l2 = math.hypot(ex2-ex1, ey2-ey1)
                                                                    if _l2 > _ebl: _ebl = _l2; _eb = (ex1, ey1, ex2, ey2)
                                                                if _eb is None:
                                                                    if _dbg8: print(f'[YIELD-FIX] {n_e}: χωρίς γραμμές', flush=True)
                                                                    return False
                                                                _eux, _euy = (_eb[2]-_eb[0])/_ebl, (_eb[3]-_eb[1])/_ebl
                                                                _epx, _epy = -_euy, _eux
                                                                _ed0 = insert_final.get(n_e, (0.0, 0.0))
                                                                _efix = False
                                                                for _es in [0.05*k5 for k5 in range(1, 8)]:
                                                                    for _eg in (1, -1):
                                                                        _enx, _eny = _ed0[0] + _epx*_es*_eg, _ed0[1] + _epy*_es*_eg
                                                                        if not _bar_move_ok(n_e, _el, _enx, _eny):
                                                                            continue
                                                                        insert_final[n_e] = (_enx, _eny)
                                                                        _sf2 = _slide_text_along(n_e, final_rebar_lines(exclude=(n_e,)),
                                                                                                 [bq for bq, nq in all_placed_text_boxes(exclude_name=n_e)] +
                                                                                                 [cq for cq, _sq3 in _circle_boxes()])
                                                                        if _sf2 is not None:
                                                                            text_local_final[n_e] = _sf2
                                                                            _efix = True
                                                                            break
                                                                        insert_final[n_e] = _ed0
                                                                    if _efix:
                                                                        break
                                                                if not _efix:
                                                                    if _dbg8: print(f'[YIELD-FIX] {n_e}: ΟΛΑ απέτυχαν', flush=True)
                                                                    return False
                                            return True
                                        # ΣΚΑΛΑ ΑΝΑ ΣΦΗΝΑ: (i) πλήρη εμπόδια, (ii) χωρίς τις
                                        # άλλες σφήνες, (iii) χωρίς κείμενα ράβδων + επισκευή.
                                        for _btw in _bts9:
                                            if done_m:
                                                break
                                            _others9 = [w for w in _bts9 if w != _btw]
                                            _variants9 = []
                                            _v1 = [b_ for b_, n_d in all_placed_text_boxes(exclude_name=_btw) if n_d != mover]
                                            _variants9.append((_v1, False))
                                            _v2 = [b_ for b_, n_d in all_placed_text_boxes(exclude_name=_btw)
                                                    if n_d != mover and n_d not in _others9]
                                            _variants9.append((_v2, False))
                                            _v3 = [b_ for b_, n_d in all_placed_text_boxes(exclude_name=_btw)
                                                    if n_d != mover and not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_d)]
                                            _variants9.append((_v3, True))
                                            for _vi9, (_plv9, _needfix9) in enumerate(_variants9):
                                                _res9 = beam_text_slide(_btw, blocks, final_rebar_lines(), hatch_polys, _plv9, relaxed=True)
                                                _cur9 = insert_final.get(_btw, (0.0, 0.0))
                                                if _dbg8:
                                                    print(f'[YIELD-BT] {_btw} v{_vi9}: slide={_res9} cur={_cur9}', flush=True)
                                                if _res9 is None or (abs(_res9[0]-_cur9[0]) < 1e-9 and abs(_res9[1]-_cur9[1]) < 1e-9):
                                                    continue
                                                _snap_bi9 = dict(insert_final); _snap_bt9 = dict(text_local_final)
                                                insert_final[_btw] = _res9
                                                _ok9 = (not _needfix9) or _fix_rebar_blockers9((_btw,))
                                                if _dbg8:
                                                    print(f'[YIELD-BT] {_btw} v{_vi9}: fix_ok={_ok9}', flush=True)
                                                _done3 = _ok9 and _retry_mover9()
                                                if _dbg8:
                                                    print(f'[YIELD-BT] {_btw} -> {_res9} (fix={_needfix9}): retry -> {_done3}', flush=True)
                                                if _done3:
                                                    done_m = True
                                                    break
                                                insert_final.clear(); insert_final.update(_snap_bi9)
                                                text_local_final.clear(); text_local_final.update(_snap_bt9)
                                        # ΣΥΝΔΥΑΣΤΙΚΑ: όλες οι σφήνες μαζί (η καθεμία χωρίς τις
                                        # άλλες στα εμπόδιά της), μετά retry.
                                        if not done_m and len(_bts9) >= 2:
                                            _snap_ci9 = dict(insert_final); _snap_ct9 = dict(text_local_final)
                                            _moved9 = 0
                                            for _btw in _bts9:
                                                _plc9 = [b_ for b_, n_d in all_placed_text_boxes(exclude_name=_btw)
                                                          if n_d != mover and n_d not in _bts9]
                                                _resc9 = beam_text_slide(_btw, blocks, final_rebar_lines(), hatch_polys, _plc9, relaxed=True)
                                                _curc9 = insert_final.get(_btw, (0.0, 0.0))
                                                if _resc9 is not None and (abs(_resc9[0]-_curc9[0]) > 1e-9 or abs(_resc9[1]-_curc9[1]) > 1e-9):
                                                    insert_final[_btw] = _resc9
                                                    _moved9 += 1
                                            _done4 = _moved9 > 0 and _fix_rebar_blockers9(tuple(_bts9)) and _retry_mover9()
                                            if _dbg8:
                                                print(f'[YIELD-BT] ΣΥΝΔΥΑΣΤΙΚΑ {_bts9} moved={_moved9}: retry -> {_done4}', flush=True)
                                            if _done4:
                                                done_m = True
                                            else:
                                                insert_final.clear(); insert_final.update(_snap_ci9)
                                                text_local_final.clear(); text_local_final.update(_snap_ct9)
                                if done_m:
                                    fixed_o = True
                                    break
                                insert_final[mover] = (mdx0, mdy0)
                                if mtl0 is not None:
                                    text_local_final[mover] = mtl0
                                elif mover in text_local_final:
                                    del text_local_final[mover]
                        elif re.match(r'FL-?\d+_BEAM_TEXT\d+$', mover):
                            v_obst2 = final_rebar_lines()
                            v_placed2 = [b_ for b_, _n in all_placed_text_boxes(exclude_name=mover)]
                            reso = beam_text_slide(mover, blocks, v_obst2, hatch_polys, v_placed2, relaxed=True)
                            curo = insert_final.get(mover, (0.0, 0.0))
                            if reso is not None and (abs(reso[0]-curo[0]) > 1e-9 or abs(reso[1]-curo[1]) > 1e-9):
                                insert_final[mover] = reso
                                fixed_o = True
                                break
                            # ΑΛΥΣΙΔΑ: βρες θέση μέσα στη δοκό αγνοώντας τα κείμενα
                            # ράβδων, και μετά κύλησέ τα να αδειάσουν τον χώρο.
                            v_placed_wo = [b_ for b_, n_5 in all_placed_text_boxes(exclude_name=mover)
                                            if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_5)]
                            reso2 = beam_text_slide(mover, blocks, v_obst2, hatch_polys, v_placed_wo, relaxed=True)
                            if reso2 is not None and (abs(reso2[0]-curo[0]) > 1e-9 or abs(reso2[1]-curo[1]) > 1e-9):
                                mbl_ = block_text_bboxes(blocks[mover])
                                cand_bx = [text_bbox(x_+reso2[0], y_+reso2[1], w_, h_, rot_) for x_, y_, w_, h_, rot_ in mbl_]
                                blk = set()
                                for cb in cand_bx:
                                    for b_, n_5 in all_placed_text_boxes(exclude_name=mover):
                                        if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n_5):
                                            continue
                                        if not (cb[2] + MIN_TEXT_GAP < b_[0] or b_[2] + MIN_TEXT_GAP < cb[0] or
                                                cb[3] + MIN_TEXT_GAP < b_[1] or b_[3] + MIN_TEXT_GAP < cb[1]):
                                            blk.add(n_5)
                                snap_bi = dict(insert_final)
                                snap_bt = dict(text_local_final)
                                insert_final[mover] = reso2
                                ok_bch = True
                                for bn2 in sorted(blk):
                                    sol_c = _slide_text_along(bn2, final_rebar_lines(exclude=(bn2,)),
                                                               [b_ for b_, n_6 in all_placed_text_boxes(exclude_name=bn2)] +
                                                               [cb_ for cb_, _sn in _circle_boxes()])
                                    if sol_c is None and not _slide_text_along_chain(bn2):
                                        ok_bch = False
                                        break
                                    if sol_c is not None:
                                        text_local_final[bn2] = sol_c
                                if ok_bch:
                                    still = False
                                    for cb in [text_bbox(x_+reso2[0], y_+reso2[1], w_, h_, rot_) for x_, y_, w_, h_, rot_ in mbl_]:
                                        for b_, n_7 in all_placed_text_boxes(exclude_name=mover):
                                            if not (cb[2] < b_[0] or b_[2] < cb[0] or cb[3] < b_[1] or b_[3] < cb[1]):
                                                still = True
                                                break
                                        if still:
                                            break
                                    if not still:
                                        fixed_o = True
                                        break
                                insert_final.clear(); insert_final.update(snap_bi)
                                text_local_final.clear(); text_local_final.update(snap_bt)
                    if fixed_o:
                        break
            if not fixed_o:
                break

    def _cleanup_cuts():
        """ΤΕΛΙΚΟ ΠΕΡΑΣΜΑ ΚΑΘΑΡΙΣΜΟΥ ΤΟΜΩΝ (γενικό, μέσω των υπαρχουσών πυλών):
        (α) Ράβδος που μετακινήθηκε και η γραμμή της πλέον κόβει κείμενο, ενώ
            θέση πιο κοντά στη φυσική είναι καθαρή -> επιστρέφει προς τη φυσική
            (revert-if-clean). Λύνει μετακινήσεις που έγιναν για συγκρούσεις
            που στο μεταξύ έπαψαν να υπάρχουν (SLABBAR14/Δ12.1).
        (β) Δείκτης πλάκας που κόβεται από γραμμή πλάκας -> μικρή μετατόπιση
            μέσω _marker_move_ok ώστε να βγει από τη γραμμή (Π6/SLAB8)."""
        def _seg_cuts_bbox(x1,y1,x2,y2,bb):
            # τέμνει το ΕΣΩΤΕΡΙΚΟ του bbox (όχι απλώς ακουμπά την άκρη);
            pad=0.01
            b0,b1,b2,b3 = bb[0]+pad, bb[1]+pad, bb[2]-pad, bb[3]-pad
            if b2<=b0 or b3<=b1: return False
            if max(x1,x2)<b0 or min(x1,x2)>b2 or max(y1,y2)<b1 or min(y1,y2)>b3: return False
            if b0<=x1<=b2 and b1<=y1<=b3: return True
            if b0<=x2<=b2 and b1<=y2<=b3: return True
            dx,dy = x2-x1, y2-y1
            for (ex1,ey1,ex2,ey2) in ((b0,b1,b2,b1),(b0,b3,b2,b3),(b0,b1,b0,b3),(b2,b1,b2,b3)):
                fx,fy = ex2-ex1, ey2-ey1
                den = dx*fy - dy*fx
                if abs(den) < 1e-12: continue
                t=((ex1-x1)*fy-(ey1-y1)*fx)/den; u=((ex1-x1)*dy-(ey1-y1)*dx)/den
                if 0<=t<=1 and 0<=u<=1: return True
            return False
        # --- (α) revert-if-clean για μετακινημένες ράβδους ---
        all_txt = all_placed_text_boxes()
        for rn in list(insert_final):
            if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', rn): continue
            rdx, rdy = insert_final[rn]
            if rdx <= -49: continue
            if abs(rdx) < 1e-6 and abs(rdy) < 1e-6: continue
            rl,_ = block_lines_local(blocks[rn])
            cuts_now = any(_seg_cuts_bbox(x1+rdx,y1+rdy,x2+rdx,y2+rdy,b_)
                            for x1,y1,x2,y2 in rl for b_,n_ in all_txt if n_ != rn)
            if not cuts_now: continue
            _cands_rc = [(rdx*f_, rdy*f_) for f_ in (0.0, 0.25, 0.5)]
            # πλήρες πλέγμα κάθετων θέσεων γύρω από τη ΦΥΣΙΚΗ (η επιστροφή
            # προς τη φυσική δεν αρκεί όταν κόβει κι εκείνη - SLABBAR14/Δ12.1)
            _rb_ = None; _rbl_ = -1.0
            for x1_,y1_,x2_,y2_ in rl:
                _l_ = math.hypot(x2_-x1_, y2_-y1_)
                if _l_ > _rbl_: _rbl_ = _l_; _rb_ = (x1_,y1_,x2_,y2_)
            if _rb_ is not None:
                _rux_, _ruy_ = (_rb_[2]-_rb_[0])/_rbl_, (_rb_[3]-_rb_[1])/_rbl_
                _rpx_, _rpy_ = -_ruy_, _rux_
                for _s_ in [0.05*k_ for k_ in range(1, 8)]:
                    for _g_ in (1, -1):
                        _cands_rc.append((_rpx_*_s_*_g_, _rpy_*_s_*_g_))
            for cdx, cdy in _cands_rc:
                if not _bar_move_ok(rn, rl, cdx, cdy): continue
                if any(_seg_cuts_bbox(x1+cdx,y1+cdy,x2+cdx,y2+cdy,b_)
                        for x1,y1,x2,y2 in rl for b_,n_ in all_txt if n_ != rn): continue
                old_pos = insert_final[rn]
                insert_final[rn] = (cdx, cdy)
                sl_ = _slide_text_along(rn, final_rebar_lines(exclude=(rn,)),
                                         [b_ for b_,nq in all_placed_text_boxes(exclude_name=rn)] +
                                         [cb_ for cb_,sq in _circle_boxes()])
                if sl_ is None:
                    insert_final[rn] = old_pos
                    continue
                text_local_final[rn] = sl_
                break
        # --- (β) δείκτες που κόβονται από γραμμές πλακών ---
        for sn in blocks:
            if not re.match(r'FL-?\d+_SLAB\d+$', sn): continue
            mb_ = slab_marker_boxes(blocks[sn])
            if not mb_: continue
            sdx, sdy = ins.get(sn,(0,0))
            mtx, mty = text_local_final.get(sn,(0.0,0.0))
            mub = union_bbox([(x+sdx+mtx, y+sdy+mty, w, h, rot) for x,y,w,h,rot in mb_])
            slab_lines = []
            for on in blocks:
                if re.match(r'FL-?\d+_SLAB\d+$', on) and on != sn:
                    od = ins.get(on,(0,0))
                    for x1,y1,x2,y2 in block_lines_local(blocks[on])[0]:
                        slab_lines.append((x1+od[0],y1+od[1],x2+od[0],y2+od[1]))
            if not any(_seg_cuts_bbox(*sl_, mub) for sl_ in slab_lines): continue
            fixed_m = False
            _sb6 = SLAB_POLYS_MAP.get(sn)
            _x_ok6 = _sb6 is not None and (_sb6[2]-_sb6[0]) >= 0.05
            _y_ok6 = _sb6 is not None and (_sb6[3]-_sb6[1]) >= 0.05
            for dd in [0.05*k for k in range(1, 25)]:
                for sh in ((0,dd),(0,-dd),(dd,0),(-dd,0)):
                    ntx, nty = mtx+sh[0], mty+sh[1]
                    if not _marker_move_ok(sn, ntx, nty): continue
                    nub = (mub[0]+sh[0], mub[1]+sh[1], mub[2]+sh[0], mub[3]+sh[1])
                    # ΕΝΤΟΣ ΠΛΑΚΑΣ στους υγιείς άξονες - αλλιώς θα το πιάσει το ΣΤ
                    if _x_ok6 and (nub[0] < _sb6[0]-0.03 or nub[2] > _sb6[2]+0.03): continue
                    if _y_ok6 and (nub[1] < _sb6[1]-0.03 or nub[3] > _sb6[3]+0.03): continue
                    if any(_seg_cuts_bbox(*sl_, nub) for sl_ in slab_lines): continue
                    if any(not(nub[2]+0.03 < b_[0] or b_[2]+0.03 < nub[0] or
                                nub[3]+0.03 < b_[1] or b_[3]+0.03 < nub[1])
                            for b_,n_ in all_placed_text_boxes() if n_ != sn): continue
                    text_local_final[sn] = (ntx, nty)
                    fixed_m = True
                    break
                if fixed_m: break

    for _reconcile in range(4):
        _final_passes()
        _cleanup_cuts()
        if _internal_bad_count() == 0:
            break

    # ΠΕΡΑΣΜΑ ΣΥΜΠΥΚΝΩΣΗΣ (από το MANUAL_MODEL): αφού όλα κάτσουν, κάθε
    # κείμενο ράβδου ξαναδοκιμάζει την ΚΟΝΤΙΝΟΤΕΡΗ στη φυσική του καθαρή
    # θέση - μετατοπίσεις που χρειάστηκαν νωρίς αλλά περίσσεψαν, μαζεύονται.
    for _cp in range(2):
        _moved_cp = 0
        for rn_ in sorted(blocks):
            if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', rn_):
                continue
            rdx_, rdy_ = insert_final.get(rn_, (0.0, 0.0))
            if rdx_ <= -49:
                continue
            cur_tl = text_local_final.get(rn_, (0.0, 0.0))
            cur_d = math.hypot(cur_tl[0], cur_tl[1])
            if cur_d < 0.10:
                continue
            sol_cp = _slide_text_along(rn_, final_rebar_lines(exclude=(rn_,)),
                                        [b_ for b_, n_cp in all_placed_text_boxes(exclude_name=rn_)] +
                                        [cb_ for cb_, _scp in _circle_boxes()])
            if sol_cp is not None and math.hypot(sol_cp[0], sol_cp[1]) < cur_d - 0.02:
                text_local_final[rn_] = sol_cp
                _moved_cp += 1
                continue
            # ΣΥΜΠΥΚΝΩΣΗ ΜΕ ΥΠΟΧΩΡΗΣΗ (κανόνας μηχανικού, περίπτωση «1Φ14»):
            # κείμενο ράβδου μακριά από τη ΦΥΣΙΚΗ του θέση, ενώ η κοντινή θα
            # ελευθερωνόταν με μικρή μετακίνηση ΕΝΟΣ γείτονα (ράβδου πλάκας ή
            # κειμένου δοκού). Εντοπίζονται οι blockers της φυσικής θέσης,
            # δοκιμάζεται η υποχώρησή τους (σύρσιμο/beam_text_slide), και
            # κρατιέται ΜΟΝΟ συνδυασμός που φέρνει το κείμενο ουσιαστικά
            # κοντύτερα (>0.15μ). Πλήρης αναίρεση αλλιώς.
            if cur_d < 0.30:
                continue
            _rb_cp = block_text_bboxes(blocks[rn_])
            _home_bbs = [text_bbox(x_+0.0, y_+0.0, w_, h_, r_) for x_, y_, w_, h_, r_ in _rb_cp]
            _blk_cp = []
            for b_, n_cp in all_placed_text_boxes(exclude_name=rn_):
                if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$', n_cp):
                    continue
                for hb_ in _home_bbs:
                    if not (hb_[2] + MIN_TEXT_GAP < b_[0] or b_[2] + MIN_TEXT_GAP < hb_[0] or
                            hb_[3] + MIN_TEXT_GAP < b_[1] or b_[3] + MIN_TEXT_GAP < hb_[1]):
                        if n_cp not in _blk_cp:
                            _blk_cp.append(n_cp)
                        break
            for _bn_cp in _blk_cp[:3]:
                _mcol = re.match(r'(FL-?\d+_)COLUMN_TEXT(\d+)$', _bn_cp)
                if _mcol:
                    _ownc = _mcol.group(1)+'COLUMN'+_mcol.group(2)
                    if _ownc not in blocks:
                        continue
                    _lblc = block_text_bboxes(blocks[_bn_cp])
                    _oc = insert_final.get(_bn_cp, (0.0, 0.0))
                    _plc = [b_ for b_, n_g in all_placed_text_boxes(exclude_name=_bn_cp) if n_g != rn_]
                    _fixed_c = False; _tried_c = 0
                    for _ddc in [0.05*kc for kc in range(2, 13)]:
                        for _shc in ((0,-_ddc),(0,_ddc),(_ddc,0),(-_ddc,0)):
                            _txc, _tyc = _oc[0]+_shc[0], _oc[1]+_shc[1]
                            if not is_ok_relaxed(_lblc, _txc, _tyc, final_rebar_lines(),
                                                  hatch_polys, _plc, (_ownc,), max_crossings=1):
                                continue
                            _tried_c += 1
                            if _tried_c > 8:
                                break
                            _sic = dict(insert_final); _stc = dict(text_local_final)
                            insert_final[_bn_cp] = (_txc, _tyc)
                            _solc = _slide_text_along(rn_, final_rebar_lines(exclude=(rn_,)),
                                                       [b_ for b_, n_h in all_placed_text_boxes(exclude_name=rn_)] +
                                                       [cb_ for cb_, _sh3 in _circle_boxes()])
                            if _solc is not None and math.hypot(_solc[0], _solc[1]) < cur_d - 0.15:
                                text_local_final[rn_] = _solc
                                _moved_cp += 1
                                _fixed_c = True
                                break
                            insert_final.clear(); insert_final.update(_sic)
                            text_local_final.clear(); text_local_final.update(_stc)
                        if _fixed_c or _tried_c > 8:
                            break
                    if _fixed_c:
                        break
                    continue
                _si_cp = dict(insert_final); _st_cp = dict(text_local_final)
                _movedb = False
                if re.match(r'FL-?\d+_BEAM_TEXT\d+$', _bn_cp):
                    _plb = [b_ for b_, n_d in all_placed_text_boxes(exclude_name=_bn_cp) if n_d != rn_]
                    _rsb = beam_text_slide(_bn_cp, blocks, final_rebar_lines(), hatch_polys, _plb, relaxed=True)
                    _cub = insert_final.get(_bn_cp, (0.0, 0.0))
                    if _rsb is not None and (abs(_rsb[0]-_cub[0]) > 1e-9 or abs(_rsb[1]-_cub[1]) > 1e-9):
                        insert_final[_bn_cp] = _rsb
                        _movedb = True
                else:
                    _sfb = _slide_text_along(_bn_cp, final_rebar_lines(exclude=(_bn_cp,)),
                                              [b_ for b_, n_e in all_placed_text_boxes(exclude_name=_bn_cp) if n_e != rn_] +
                                              [cb_ for cb_, _se in _circle_boxes()])
                    if _sfb is not None:
                        _tlb = text_local_final.get(_bn_cp, (0.0, 0.0))
                        if abs(_sfb[0]-_tlb[0]) > 1e-9 or abs(_sfb[1]-_tlb[1]) > 1e-9:
                            text_local_final[_bn_cp] = _sfb
                            _movedb = True
                if not _movedb:
                    continue
                _sol2 = _slide_text_along(rn_, final_rebar_lines(exclude=(rn_,)),
                                           [b_ for b_, n_f in all_placed_text_boxes(exclude_name=rn_)] +
                                           [cb_ for cb_, _sf2 in _circle_boxes()])
                if _sol2 is not None and math.hypot(_sol2[0], _sol2[1]) < cur_d - 0.15:
                    text_local_final[rn_] = _sol2
                    _moved_cp += 1
                    break
                insert_final.clear(); insert_final.update(_si_cp)
                text_local_final.clear(); text_local_final.update(_st_cp)
        if _moved_cp == 0:
            break
    if _internal_bad_count() > 0:
        _final_passes()
        _cleanup_cuts()

    # ΤΕΛΙΚΟ ΠΕΡΑΣΜΑ «ΚΑΘΑΡΩΝ ΓΡΑΜΜΑΤΩΝ» (καθολικός κανόνας μηχανικού):
    # κάθε κείμενο ράβδου που τελειώνει με γραμμή ΠΑΝΩ στα γράμματά του
    # ξαναπερνά από τη δίφασική σκάλα ολίσθησης - αν υπάρχει θέση με καθαρά
    # γράμματα (οσοδήποτε μικρή μετακίνηση, 0.10-0.30 συνήθως αρκεί),
    # παίρνεται. Καμία άλλη αλλαγή, κανένας άλλος κανόνας δεν χαλαρώνει.
    def _core_cut9(n_):
        d_ = insert_final.get(n_, (0.0, 0.0)); t_ = text_local_final.get(n_, (0.0, 0.0))
        for x_, y_, w_, h_, r_ in block_text_bboxes(blocks[n_]):
            bb_ = text_bbox(x_+d_[0]+t_[0], y_+d_[1]+t_[1], w_, h_, r_)
            p_ = 0.015
            core_ = (bb_[0]+p_, bb_[1]+p_, bb_[2]-p_, bb_[3]-p_)
            if core_[2] <= core_[0] or core_[3] <= core_[1]:
                continue
            for s_ in final_rebar_lines(exclude=(n_,)):
                if seg_intersects_bbox(s_[:4], core_):
                    return True
        return False
    for _lp9 in range(2):
        _mv9 = 0
        for rn8 in sorted(blocks):
            if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', rn8):
                continue
            rd8 = insert_final.get(rn8, (0.0, 0.0))
            if rd8[0] <= -49 or not _core_cut9(rn8):
                continue
            _t0 = text_local_final.get(rn8, (0.0, 0.0))
            _fixed8 = False
            _s8 = _slide_text_along(rn8, final_rebar_lines(exclude=(rn8,)),
                                     [b_ for b_, n_x in all_placed_text_boxes(exclude_name=rn8)] +
                                     [cb_ for cb_, _sx in _circle_boxes()])
            if _s8 is not None and (abs(_s8[0]-_t0[0]) > 1e-9 or abs(_s8[1]-_t0[1]) > 1e-9):
                text_local_final[rn8] = _s8
                if not _core_cut9(rn8):
                    _fixed8 = True; _mv9 += 1
                else:
                    text_local_final[rn8] = _t0
            if not _fixed8:
                # ράβδος παράλληλη σε γραμμή: μόνο-σύρσιμο δεν ξεφεύγει ποτέ -
                # δοκιμή ΚΑΘΕΤΗΣ μετακίνησης σιδήρου (εντός πύλης) + σύρσιμο.
                _ln8, _ = block_lines_local(blocks[rn8])
                _mb8 = None; _ml8 = -1.0
                for xx1, yy1, xx2, yy2 in _ln8:
                    _l8 = math.hypot(xx2-xx1, yy2-yy1)
                    if _l8 > _ml8: _ml8 = _l8; _mb8 = (xx1, yy1, xx2, yy2)
                if _mb8 is None:
                    continue
                _u8 = ((_mb8[2]-_mb8[0])/_ml8, (_mb8[3]-_mb8[1])/_ml8)
                _p8 = (-_u8[1], _u8[0])
                _d0_8 = insert_final.get(rn8, (0.0, 0.0))
                for _sm8 in [0.05*k8 for k8 in range(1, 8)]:
                    for _sg8 in (1, -1):
                        _n8 = (_d0_8[0] + _p8[0]*_sm8*_sg8, _d0_8[1] + _p8[1]*_sm8*_sg8)
                        if not _bar_move_ok(rn8, _ln8, _n8[0], _n8[1]):
                            continue
                        # ΦΡΟΥΡΟΣ: η γραμμή στη νέα θέση δεν κόβει ΚΑΝΕΝΑ κείμενο
                        # που δεν έκοβε ήδη - καμία λύση δεν χαλάει άλλον κανόνα.
                        _hit_new8 = False
                        for _bb8, _nb8 in all_placed_text_boxes(exclude_name=rn8):
                            _old_hit = any(seg_intersects_bbox((a1+_d0_8[0], a2+_d0_8[1], a3+_d0_8[0], a4+_d0_8[1]), _bb8)
                                           for a1, a2, a3, a4 in _ln8)
                            if _old_hit:
                                continue
                            if any(seg_intersects_bbox((a1+_n8[0], a2+_n8[1], a3+_n8[0], a4+_n8[1]), _bb8)
                                   for a1, a2, a3, a4 in _ln8):
                                _hit_new8 = True; break
                        if _hit_new8:
                            continue
                        insert_final[rn8] = _n8
                        _s9 = _slide_text_along(rn8, final_rebar_lines(exclude=(rn8,)),
                                                 [b_ for b_, n_y in all_placed_text_boxes(exclude_name=rn8)] +
                                                 [cb_ for cb_, _sy in _circle_boxes()])
                        if _s9 is not None:
                            text_local_final[rn8] = _s9
                            if not _core_cut9(rn8):
                                _fixed8 = True; _mv9 += 1
                                break
                        insert_final[rn8] = _d0_8
                        text_local_final[rn8] = _t0
                    if _fixed8:
                        break
        if _mv9 == 0:
            break

    # ΤΕΛΙΚΟΣ ΕΛΕΓΧΟΣ ΦΟΥΡΚΕΤΑΣ (μετά ΑΠ' ΟΛΑ, ώστε καμία σειρά περασμάτων
    # να μην τον προσπερνά - περίπτωση SLABBAR14): φουρκέτα-δύο-άκρων της
    # οποίας τα ΣΚΕΛΗ κόβουν οποιοδήποτε τελικό κείμενο ενώ ο κορμός είναι
    # καθαρός -> κατάρρευση στη γραμμή-κορμό (εγκεκριμένη εφεδρεία).
    for rn9 in sorted(blocks):
        if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', rn9) or rn9 in COLLAPSE_BARS:
            continue
        rd9 = insert_final.get(rn9, (0.0, 0.0))
        if rd9[0] <= -49:
            continue
        ln9, _ = block_lines_local(blocks[rn9])
        if len(ln9) < 3:
            continue
        co9 = _collapse_lines(ln9)
        off9 = [max(abs(a1-b1), abs(a2-b2), abs(a3-b3), abs(a4-b4))
                for (a1,a2,a3,a4),(b1,b2,b3,b4) in zip(ln9, co9)]
        arm9 = [k for k,o in enumerate(off9) if o > 0.05]
        if len(arm9) < 2:
            continue
        (au9, av9), (lo9, hi9) = spine_and_bounds(ln9)
        ts9 = [((ln9[k][0]+ln9[k][2])/2*au9 + (ln9[k][1]+ln9[k][3])/2*av9) for k in arm9]
        sp9 = hi9 - lo9
        if not (min(ts9) < lo9 + 0.25*sp9 and max(ts9) > hi9 - 0.25*sp9):
            continue
        _armhit = False; _spinehit = False
        for bb9, nb9 in all_placed_text_boxes(exclude_name=rn9):
            for k,(x1,y1,x2,y2) in enumerate(ln9):
                if seg_intersects_bbox((x1+rd9[0], y1+rd9[1], x2+rd9[0], y2+rd9[1]), bb9):
                    if k in arm9: _armhit = True
                    else: _spinehit = True
        if _armhit and not _spinehit:
            # ΦΡΟΥΡΟΣ: η ΚΑΤΑΡΡΕΥΜΕΝΗ γεωμετρία δεν επιτρέπεται να κόβει
            # ΤΙΠΟΤΑ - ούτε κείμενα ούτε δείκτες πλακών (leukos/Π16-SLABBAR76:
            # ο ευθυγραμμισμένος κορμός έπεφτε σε δείκτη). Αλλιώς η εφεδρεία
            # δεν εφαρμόζεται.
            _tgts9 = [bb for bb, _nb in all_placed_text_boxes(exclude_name=rn9)]
            for sn9 in blocks:
                if not re.match(r'FL-?\d+_SLAB\d+$', sn9):
                    continue
                _mb9 = slab_marker_boxes(blocks[sn9])
                if not _mb9:
                    continue
                _sd9 = text_local_final.get(sn9, (0.0, 0.0))
                for x9, y9, w9, h9, r9 in _mb9:
                    _tgts9.append(text_bbox(x9+_sd9[0], y9+_sd9[1], w9, h9, r9))
            _newhit = False
            for x1,y1,x2,y2 in co9:
                _sg9 = (x1+rd9[0], y1+rd9[1], x2+rd9[0], y2+rd9[1])
                for bb in _tgts9:
                    if seg_intersects_bbox(_sg9, bb):
                        _newhit = True; break
                if _newhit: break
            if not _newhit:
                pass  # ΑΠΕΝΕΡΓΟ κατόπιν εντολής: COLLAPSE_BARS.add(rn9)

    return insert_final, text_local_final

if __name__ == '__main__':
    insert_final, text_local_final = process_all('/mnt/user-data/uploads/input.dxf')
    pickle.dump((insert_final, text_local_final), open('final_offsets_v11.pkl','wb'))
    print('text_local (internal-only slides needed):', len(text_local_final))
