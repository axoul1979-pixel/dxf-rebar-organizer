"""
slab_region.py - Πραγματικό (κλειστό) περίγραμμα κάθε πλάκας SLABn, για τον
έλεγχο ζώνης οπλισμού πλάκας (ΚΑΝΟΝΙΣΜΟΣ §0, §3): η ράβδος μένει ΠΑΝΤΑ εντός
των ορίων της πλάκας της, χωρίς άλλο όριο απόστασης - άρα χρειάζεται το
πραγματικό όριο, όχι bbox από αραιά διαθέσιμο slab_poly.

ΚΑΝΟΝΑΣ ΟΡΙΟΥ (επαληθευμένος σε πραγματικό έργο):
  - Οι γραμμές του layer `slab_poly` ΜΕΣΑ στο ίδιο το SLABn block έχουν
    ΠΡΟΤΕΡΑΙΟΤΗΤΑ - είναι το σχέδιο του μηχανικού και ΔΕΝ παρακάμπτονται.
  - Πολλά FESPA αρχεία σχεδιάζουν slab_poly ΜΟΝΟ στις ελεύθερες πλευρές μιας
    πλάκας (οι κοινές ακμές με γειτονικές πλάκες λείπουν) - οπότε ΜΟΝΟ εκεί
    που λείπει slab_poly, το κενό συμπληρώνεται από τις παρειές δοκών/κολωνών
    γύρω. Δοκοί ΠΟΤΕ δεν τεμαχίζουν υπαρκτό slab_poly.
  - Ανακατασκευή με flood fill σε κάνναβο (ίδιο μοτίβο με το perimeter.py),
    σπαρμένο από το κέντρο του δείκτη-πλάκας (κύκλος slab_center) που κάθεται
    πάντα μέσα στο σώμα της πλάκας.
  - Λωρίδα στενότερη από MIN_SLAB_WIDTH ΔΕΝ είναι πλάκα - είναι το εσωτερικό
    μιας δοκού. Χωρίς αυτό το γέμισμα μπερδεύει τη λωρίδα μιας δοκού με πλάκα
    όταν ο σπόρος (κέντρο προβόλου) κάθεται πάνω σε δοκό.
  - Οι παρειές δοκών/κολωνών προεκτείνονται GAP_CLOSE στα δύο άκρα τους ώστε
    οι γωνίες να σφραγίζουν· χωρίς αυτό το γέμισμα διέφευγε από μικρά κενά
    ανάμεσα σε σχεδόν εφαπτόμενες παρειές γειτονικών δοκών.
"""
import re, math
from collections import deque
from analyze import entities_from_pairs, to_dict
from engine import load_all, block_lines_local

CELL = 0.10
GRID_R = 15.0            # ακτίνα καννάβου γύρω από τον σπόρο (αρκεί για μία πλάκα)
GAP_CLOSE = 0.06          # προέκταση άκρων παρειών δοκού/κολώνας για σφράγιση γωνιών
MIN_SLAB_WIDTH = 0.50     # λωρίδα στενότερη από αυτό δεν είναι πλάκα
HOOK_MAX_SEG = 0.30       # μήκος τμήματος κάτω από αυτό μετράει ως πιθανό άγκιστρο
HOOK_MIN_SEGS = 3         # ελάχιστος αριθμός μικροτμημάτων για να αναγνωριστεί άγκιστρο

STRUCT_RE = r'FL-?\d+_(BEAM|COLUMN)\d*$'
SLAB_RE = r'FL-?\d+_SLAB\d+$'

_cache = {}


def bar_reference_point(lines):
    """Σημείο αναφοράς μιας ράβδου πλάκας, για να ξέρουμε ΠΟΙΑ πλάκα την
    «κατέχει»: το άκρο με το ΑΓΚΙΣΤΡΟ αν η ράβδος είναι πρόβολος (το άγκιστρο
    είναι το διπλωμένο άκρο - εκεί όπου συγκεντρώνονται πολλά κοντά τμήματα),
    αλλιώς το γεωμετρικό κέντρο. Χωρίς αυτό, το κέντρο ενός προβόλου συχνά
    πέφτει πάνω στη δοκό στην οποία στηρίζεται και "δεν βρίσκεται πλάκα"."""
    if not lines:
        return None
    pts = [(x1, y1) for x1, y1, x2, y2 in lines] + [(x2, y2) for x1, y1, x2, y2 in lines]
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    short = [s for s in lines if math.hypot(s[2]-s[0], s[3]-s[1]) < HOOK_MAX_SEG]
    if len(short) < HOOK_MIN_SEGS:
        return (cx, cy)
    hx = sum((s[0]+s[2])/2 for s in short) / len(short)
    hy = sum((s[1]+s[3])/2 for s in short) / len(short)
    return (hx, hy)


def _slab_poly_lines_by_name(blocks, ins):
    out = {}
    for name, pairlist in blocks.items():
        if not re.match(SLAB_RE, name):
            continue
        ox, oy = ins.get(name, (0, 0))
        lines = []
        for e in entities_from_pairs(pairlist):
            if e[0][1] != 'LINE':
                continue
            d = to_dict(e)
            if d.get(8, [''])[0] != 'slab_poly':
                continue
            x1 = float(d[10][0])+ox; y1 = float(d[20][0])+oy
            x2 = float(d[11][0])+ox; y2 = float(d[21][0])+oy
            lines.append((x1, y1, x2, y2))
        out[name] = lines
    return out


def _struct_edges(blocks, ins):
    segs = []
    for name, pairlist in blocks.items():
        if not re.match(STRUCT_RE, name) or 'TEXT' in name:
            continue
        ox, oy = ins.get(name, (0, 0))
        lines, _ = block_lines_local(pairlist)
        for x1, y1, x2, y2 in lines:
            segs.append((x1+ox, y1+oy, x2+ox, y2+oy))
    return segs


def _slab_marker_center(blocks, ins, name):
    ox, oy = ins.get(name, (0, 0))
    for e in entities_from_pairs(blocks[name]):
        if e[0][1] != 'CIRCLE':
            continue
        d = to_dict(e)
        if d.get(8, [''])[0] != 'slab_center':
            continue
        return float(d[10][0])+ox, float(d[20][0])+oy
    return None


def build_slab_region(input_path, name, seed=None):
    """Το πραγματικό περίγραμμα της πλάκας `name`, ως (x1,y1,x2,y2) γύρω από
    την περιοχή που γεμίζει το flood fill. Επιστρέφει None αν δεν βρεθεί
    σπόρος (κύκλος δείκτη) ή αν η περιοχή είναι εκφυλισμένη/πολύ στενή.
    `seed` προαιρετικό (cx,cy) - αν λείπει, χρησιμοποιείται το κέντρο του
    κύκλου slab_center του ίδιου block."""
    ins, blocks = load_all(input_path)
    if seed is None:
        seed = _slab_marker_center(blocks, ins, name)
    if seed is None:
        return None
    cx, cy = seed
    slab_lines = _slab_poly_lines_by_name(blocks, ins).get(name, [])
    struct = _struct_edges(blocks, ins)

    R = GRID_R
    x0, y0 = cx - R, cy - R
    n = int(2*R/CELL) + 2
    wall = bytearray(n*n)

    def mark(ix, iy):
        if 0 <= ix < n and 0 <= iy < n:
            wall[iy*n+ix] = 1

    def draw(ax, ay, bx, by, extend=0.0):
        L = math.hypot(bx-ax, by-ay)
        if L < 1e-9:
            return
        if extend:
            ux, uy = (bx-ax)/L, (by-ay)/L
            ax -= ux*extend; ay -= uy*extend
            bx += ux*extend; by += uy*extend
            L = math.hypot(bx-ax, by-ay)
        steps = max(2, int(L/(CELL/3.0))+1)
        for k in range(steps+1):
            t = k/steps
            mark(int((ax+(bx-ax)*t-x0)/CELL), int((ay+(by-ay)*t-y0)/CELL))

    for x1, y1, x2, y2 in slab_lines:
        if min(x1, x2) > cx+R or max(x1, x2) < cx-R: continue
        if min(y1, y2) > cy+R or max(y1, y2) < cy-R: continue
        draw(x1, y1, x2, y2)                       # slab_poly: όριο, δεν προεκτείνεται
    for x1, y1, x2, y2 in struct:
        if min(x1, x2) > cx+R or max(x1, x2) < cx-R: continue
        if min(y1, y2) > cy+R or max(y1, y2) < cy-R: continue
        draw(x1, y1, x2, y2, extend=GAP_CLOSE)      # δοκοί/κολώνες: συμπληρώνουν κενά

    six, siy = int((cx-x0)/CELL), int((cy-y0)/CELL)
    if not (0 <= six < n and 0 <= siy < n) or wall[siy*n+six]:
        return None

    inside = bytearray(n*n)
    dq = deque([(six, siy)])
    inside[siy*n+six] = 1
    while dq:
        ix, iy = dq.popleft()
        for jx, jy in ((ix+1, iy), (ix-1, iy), (ix, iy+1), (ix, iy-1)):
            if 0 <= jx < n and 0 <= jy < n:
                j = jy*n+jx
                if not wall[j] and not inside[j]:
                    inside[j] = 1
                    dq.append((jx, jy))

    xs = []; ys = []
    for iy in range(n):
        row = iy*n
        for ix in range(n):
            if inside[row+ix]:
                xs.append(x0+ix*CELL); ys.append(y0+iy*CELL)
    if not xs:
        return None
    bb = (min(xs), min(ys), max(xs)+CELL, max(ys)+CELL)
    if (bb[2]-bb[0]) < MIN_SLAB_WIDTH or (bb[3]-bb[1]) < MIN_SLAB_WIDTH:
        return None
    # δικλείδα διαρροής: αν γέμισε σχεδόν όλο τον κάνναβο αναζήτησης γύρω από
    # τον σπόρο, δεν βρήκε πραγματικό κλειστό περίγραμμα - διέφυγε προς τα έξω
    # (π.χ. σπόρος εκτός κτιρίου, κενό/προβληματικό block). Καλύτερα «δεν
    # βρέθηκε» παρά μια ψεύτικη τεράστια πλάκα.
    if (bb[2]-bb[0]) > 0.85*2*R or (bb[3]-bb[1]) > 0.85*2*R:
        return None
    return bb


def all_slab_regions(input_path):
    """dict: SLABn name -> (x1,y1,x2,y2), για όλα τα SLAB blocks που έχουν
    αναγνωρίσιμο σπόρο (κύκλο δείκτη) και μη-εκφυλισμένη περιοχή."""
    if input_path in _cache:
        return _cache[input_path]
    ins, blocks = load_all(input_path)
    out = {}
    for name in blocks:
        if not re.match(SLAB_RE, name):
            continue
        bb = build_slab_region(input_path, name)
        if bb:
            out[name] = bb
    _cache[input_path] = out
    return out


if __name__ == '__main__':
    import sys
    from engine import load_all as _load_all
    _, _blocks = _load_all(sys.argv[1])
    _total = sum(1 for n in _blocks if re.match(SLAB_RE, n))
    regs = all_slab_regions(sys.argv[1])
    for n in sorted(regs, key=lambda n: int(re.search(r'\d+$', n).group())):
        x1, y1, x2, y2 = regs[n]
        print(f'{n:12s} x[{x1:.2f}-{x2:.2f}] y[{y1:.2f}-{y2:.2f}]')
    print(f'{len(regs)} / {_total} πλάκες αναγνωρίστηκαν')



# ============================================================================
# ΟΡΙΣΜΟΣ ΜΗΧΑΝΙΚΟΥ (ΓΙΑ SLABBAR ΜΟΝΟ):
# «Η πλάκα στην οποία ανήκει ένα slabbar είναι το πολύγωνο από slab_poly &
#  beam_poly_* που περιβάλλει την ΕΤΙΚΕΤΑ του οπλισμού, ΜΟΝΟ στο αρχείο-μήτρα»
# Ο σπόρος πρέπει να είναι η ΑΡΧΙΚΗ θέση της ετικέτας: σε επόμενα αρχεία η
# ετικέτα μπορεί να έχει ολισθήσει και να έχει μπει σε ΔΙΠΛΑΝΗ πλάκα, οπότε
# ο ίδιος υπολογισμός θα έδινε λάθος (γειτονική) πλάκα και θα «νομιμοποιούσε»
# μετακινήσεις εκτός της πραγματικής πλάκας της ράβδου.
# Τα όρια αυτού του πολυγώνου ΕΙΝΑΙ τα όρια μέσα στα οποία επιτρέπεται να
# κινηθεί η ράβδος.
# ============================================================================

def poly_barrier_segments(blocks, ins):
    """Όλες οι γραμμές των layer `slab_poly` και `beam_poly_*`, σε απόλυτες
    συντεταγμένες. Αυτά - και μόνο αυτά - ορίζουν το πολύγωνο της πλάκας."""
    segs = []
    for name, pairlist in blocks.items():
        ox, oy = ins.get(name, (0, 0))
        for e in entities_from_pairs(pairlist):
            if e[0][1] != 'LINE':
                continue
            d = to_dict(e)
            lay = (d.get(8, ['']) or [''])[0]
            if lay != 'slab_poly' and not lay.startswith('beam_poly'):
                continue
            try:
                segs.append((float(d[10][0])+ox, float(d[20][0])+oy,
                             float(d[11][0])+ox, float(d[21][0])+oy))
            except (KeyError, IndexError, ValueError):
                continue
    return segs


def region_around_point(input_path, cx, cy, _barrier_cache={}):
    """Το κλειστό πολύγωνο (ως bbox) από slab_poly & beam_poly_* που περιβάλλει
    το σημείο (cx, cy). Επιστρέφει None αν το σημείο πέφτει πάνω σε όριο ή αν
    το γέμισμα διαρρεύσει (δεν βρέθηκε κλειστό περίγραμμα)."""
    # ΠΡΑΚΤΙΚΗ ΘΕΩΡΗΣΗ ΤΟΥ ΟΡΙΣΜΟΥ: στα αρχεία FESPA το layer beam_poly_* έχει
    # ΜΙΑ γραμμή ανά δοκό (τον άξονα), όχι τις δύο παρειές - μετρήθηκε: 11
    # slab_poly + 38 beam_poly για 9 πλάκες/36 δοκούς. Με αυτά ΜΟΝΟ, καμία
    # πλάκα δεν κλείνει και το γέμισμα διαρρέει σε όλο το σχέδιο (μετρήθηκε:
    # 25/30 ράβδοι χωρίς περιοχή). Άρα το «πολύγωνο από slab_poly & beam_poly»
    # υλοποιείται με: slab_poly (προτεραιότητα, το σχέδιο του μηχανικού) ΣΥΝ
    # τις ΠΡΑΓΜΑΤΙΚΕΣ ΠΑΡΕΙΕΣ δοκών/κολώνων - που είναι ακριβώς το φυσικό
    # όριο που εννοεί ο ορισμός. Ο ΣΠΟΡΟΣ μένει αυτός που όρισε ο μηχανικός:
    # η ετικέτα του οπλισμού στο ΑΡΧΕΙΟ-ΜΗΤΡΑ.
    if input_path not in _barrier_cache:
        ins, blocks = load_all(input_path)
        segs_ = poly_barrier_segments(blocks, ins)
        segs_ += _struct_edges(blocks, ins)
        _barrier_cache[input_path] = segs_
    segs = _barrier_cache[input_path]

    R = GRID_R
    x0, y0 = cx - R, cy - R
    n = int(2*R/CELL) + 2
    wall = bytearray(n*n)

    def mark(ix, iy):
        if 0 <= ix < n and 0 <= iy < n:
            wall[iy*n+ix] = 1

    def draw(ax, ay, bx, by, extend=GAP_CLOSE):
        L = math.hypot(bx-ax, by-ay)
        if L < 1e-9:
            return
        if extend:
            ux, uy = (bx-ax)/L, (by-ay)/L
            ax -= ux*extend; ay -= uy*extend
            bx += ux*extend; by += uy*extend
            L = math.hypot(bx-ax, by-ay)
        steps = max(2, int(L/(CELL/3.0))+1)
        for k in range(steps+1):
            t = k/steps
            mark(int((ax+(bx-ax)*t-x0)/CELL), int((ay+(by-ay)*t-y0)/CELL))

    for x1, y1, x2, y2 in segs:
        if min(x1, x2) > cx+R or max(x1, x2) < cx-R:
            continue
        if min(y1, y2) > cy+R or max(y1, y2) < cy-R:
            continue
        draw(x1, y1, x2, y2)

    six, siy = int((cx-x0)/CELL), int((cy-y0)/CELL)
    if not (0 <= six < n and 0 <= siy < n) or wall[siy*n+six]:
        return None

    inside = bytearray(n*n)
    dq = deque([(six, siy)])
    inside[siy*n+six] = 1
    while dq:
        ix, iy = dq.popleft()
        for jx, jy in ((ix+1, iy), (ix-1, iy), (ix, iy+1), (ix, iy-1)):
            if 0 <= jx < n and 0 <= jy < n:
                j = jy*n+jx
                if not wall[j] and not inside[j]:
                    inside[j] = 1
                    dq.append((jx, jy))

    xs = []; ys = []
    for iy in range(n):
        row = iy*n
        for ix in range(n):
            if inside[row+ix]:
                xs.append(x0+ix*CELL); ys.append(y0+iy*CELL)
    if not xs:
        return None
    bb = (min(xs), min(ys), max(xs)+CELL, max(ys)+CELL)
    # διαρροή: γέμισε σχεδόν όλον τον κάνναβο -> δεν βρέθηκε κλειστό περίγραμμα
    if (bb[2]-bb[0]) > 0.85*2*R or (bb[3]-bb[1]) > 0.85*2*R:
        return None
    return bb
