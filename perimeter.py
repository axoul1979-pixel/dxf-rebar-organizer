"""
perimeter.py - Εντοπισμός ΠΕΡΙΜΕΤΡΙΚΩΝ κολωνών και της κατεύθυνσης "προς τα έξω".

Σκοπός: το κείμενο οπλισμού μιας περιμετρικής κολώνας να βγαίνει ΕΞΩ από το
περίγραμμα του κτιρίου, ώστε να μην μπλέκεται με τα κείμενα οπλισμών δοκών και
πλακών που συνωστίζονται στο εσωτερικό.

ΑΠΟΤΥΠΩΜΑ ΚΤΙΡΙΟΥ (footprint)
-----------------------------
Δεν χρησιμοποιείται convex hull (γεμίζει τις εσοχές σε κατόψεις Γ/Π/Τ και χάνει
τις κολώνες τους), ούτε ένωση bbox πλακών: τα slab_poly υπάρχουν συχνά ΜΟΝΟ στις
ελεύθερες ακμές - οι κοινές ακμές μεταξύ γειτονικών πλακών λείπουν, οπότε το
αποτύπωμα βγαίνει τρύπιο και μερικές πλάκες δίνουν ακόμη και εκφυλισμένο bbox.

Αντ' αυτού: ΨΗΦΙΔΟΠΟΙΗΣΗ + FLOOD FILL. Ο σκελετός (δοκοί + κολώνες + ό,τι
slab_poly υπάρχει) αποτυπώνεται σε κάνναβο· γεμίζει με νερό η περιοχή έξω από τα
όρια του σχεδίου· ό,τι ΔΕΝ βράχηκε είναι το εσωτερικό του κτιρίου. Έτσι κάθε
δωμάτιο που περικλείεται από δοκούς μετράει ως "μέσα" χωρίς να χρειάζεται καμία
πληροφορία πλάκας, και κάθε εσοχή/κοίλο σχήμα αποδίδεται σωστά.

Ο κάνναβος είναι 8-συνδεδεμένος για τους τοίχους και 4-συνδεδεμένος για το
γέμισμα - ο συνδυασμός είναι υδατοστεγής, δεν διαρρέει από διαγώνιες δοκούς.

ΚΑΤΕΥΘΥΝΣΗ ΕΞΟΔΟΥ
-----------------
Από κάθε κολώνα ρίχνονται 72 ακτίνες. Μια κατεύθυνση μετράει "έξω" μόνο αν είναι
έξω σε ΟΛΕΣ τις αποστάσεις ελέγχου, ώστε ένας εσωτερικός φωταγωγός (που ξαναμπαίνει
μέσα πιο πέρα) να μην περνιέται για ύπαιθρο. Η φαρδύτερη συνεχής "έξω" γωνία δίνει
τη διχοτόμο: γωνιακές κολώνες βγάζουν φυσικά διαγώνια, ενδιάμεσες περιμετρικές
κάθετα στην πλευρά τους.
"""

import re, math
from collections import deque
from engine import block_lines_local

# --- παράμετροι εντοπισμού -------------------------------------------------
CELL = 0.10             # ψηφίδα καννάβου (μ)
GRID_MARGIN = 3.0       # περιθώριο γύρω από το σχέδιο, για να έχει το flood fill
                         # από πού να ξεκινήσει
DETECT_PAD = 0.30       # διόγκωση στον ΕΝΤΟΠΙΣΜΟ: μια ακτίνα που ξύνει την παρειά
                         # δεν πρέπει να μετρήσει σαν "έξω"
OUTSIDE_PAD = 0.10      # διόγκωση στον ΕΛΕΓΧΟ τελικής θέσης κειμένου
RAY_MAX = 6.0           # μήκος ακτίνας ελέγχου (μ)
RAY_STEP = 0.15
EXIT_DIST = 2.5         # η ακτίνα πρέπει να έχει βγει από το κτίριο ΜΕΧΡΙ εδώ
N_PROBE_DIRS = 72
MIN_SECTOR_DEG = 35.0   # κάτω από αυτό το άνοιγμα, δεν θεωρείται περιμετρική

STRUCT_RE = r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE)\d*$'


def structural_segments(blocks, ins):
    """Όλες οι γραμμές του φέροντος σκελετού σε world coords."""
    segs = []
    for name, pairlist in blocks.items():
        if not re.match(STRUCT_RE, name) or 'TEXT' in name:
            continue
        ox, oy = ins.get(name, (0, 0))
        lines, _ = block_lines_local(pairlist)
        for x1, y1, x2, y2 in lines:
            segs.append((x1+ox, y1+oy, x2+ox, y2+oy, name))
    return segs


def build_footprint(blocks, ins, slab_boxes=None, cell=CELL, margin=GRID_MARGIN):
    """Αποτύπωμα κτιρίου ως κάνναβος occupancy. Επιστρέφει dict με το mask
    'inside' (bytearray, row-major) και τη γεωμετρία του καννάβου."""
    segs = structural_segments(blocks, ins)
    rects = list(slab_boxes.values()) if slab_boxes else []
    # απόρριψη εκφυλισμένων bbox (πλάκες με ελλιπείς slab_poly ακμές)
    rects = [r for r in rects if (r[2]-r[0]) > 1e-6 and (r[3]-r[1]) > 1e-6]

    if not segs and not rects:
        return None

    xs = [p for s in segs for p in (s[0], s[2])] + [r[0] for r in rects] + [r[2] for r in rects]
    ys = [p for s in segs for p in (s[1], s[3])] + [r[1] for r in rects] + [r[3] for r in rects]
    x0, y0 = min(xs)-margin, min(ys)-margin
    x1, y1 = max(xs)+margin, max(ys)+margin
    nx = int((x1-x0)/cell)+2
    ny = int((y1-y0)/cell)+2

    wall = bytearray(nx*ny)

    def mark(ix, iy):
        if 0 <= ix < nx and 0 <= iy < ny:
            wall[iy*nx+ix] = 1

    # ψηφιδοποίηση γραμμών: δειγματοληψία ανά cell/3 -> 8-συνδεδεμένος "τοίχος"
    for ax, ay, bx, by, _ in segs:
        L = math.hypot(bx-ax, by-ay)
        n = max(2, int(L/(cell/3.0))+1)
        for k in range(n+1):
            t = k/n
            px, py = ax+(bx-ax)*t, ay+(by-ay)*t
            mark(int((px-x0)/cell), int((py-y0)/cell))

    # όποια bbox πλάκας είναι έγκυρη, γεμίζει κιόλας (βοηθάει όπου λείπουν δοκοί)
    for rx1, ry1, rx2, ry2 in rects:
        for iy in range(max(0, int((ry1-y0)/cell)), min(ny, int((ry2-y0)/cell)+1)):
            for ix in range(max(0, int((rx1-x0)/cell)), min(nx, int((rx2-x0)/cell)+1)):
                wall[iy*nx+ix] = 1

    # flood fill 4-συνδεδεμένο από ΟΛΟ το περίγραμμα του καννάβου
    outside = bytearray(nx*ny)
    dq = deque()
    for ix in range(nx):
        for iy in (0, ny-1):
            i = iy*nx+ix
            if not wall[i] and not outside[i]:
                outside[i] = 1; dq.append((ix, iy))
    for iy in range(ny):
        for ix in (0, nx-1):
            i = iy*nx+ix
            if not wall[i] and not outside[i]:
                outside[i] = 1; dq.append((ix, iy))
    while dq:
        ix, iy = dq.popleft()
        for jx, jy in ((ix+1, iy), (ix-1, iy), (ix, iy+1), (ix, iy-1)):
            if 0 <= jx < nx and 0 <= jy < ny:
                j = jy*nx+jx
                if not wall[j] and not outside[j]:
                    outside[j] = 1; dq.append((jx, jy))

    # "μέσα" = ό,τι δεν βράχηκε (περιλαμβάνει τον ίδιο τον σκελετό)
    inside = bytearray(nx*ny)
    for i in range(nx*ny):
        if not outside[i]:
            inside[i] = 1

    n_in = sum(inside)
    return {'inside': inside, 'x0': x0, 'y0': y0, 'cell': cell, 'nx': nx, 'ny': ny,
            'n_inside': n_in, 'area': n_in*cell*cell, 'rects': rects}


def point_inside(fp, x, y, pad=0.0):
    """True αν το σημείο (ή οτιδήποτε μέσα σε ακτίνα pad) πέφτει στο κτίριο."""
    if not fp:
        return False
    cell = fp['cell']; nx = fp['nx']; ny = fp['ny']
    inside = fp['inside']
    ix1 = int((x-pad-fp['x0'])/cell); ix2 = int((x+pad-fp['x0'])/cell)
    iy1 = int((y-pad-fp['y0'])/cell); iy2 = int((y+pad-fp['y0'])/cell)
    for iy in range(max(0, iy1), min(ny-1, iy2)+1):
        row = iy*nx
        for ix in range(max(0, ix1), min(nx-1, ix2)+1):
            if inside[row+ix]:
                return True
    return False


def bbox_outside(fp, bb, pad=OUTSIDE_PAD):
    """True αν ΟΛΟ το πλαίσιο κειμένου είναι έξω από το κτίριο. Ελέγχεται με
    δειγματοληψία σε κάνναβο πάνω στο πλαίσιο (όχι μόνο γωνίες): ένα μακρύ κείμενο
    μπορεί να έχει και τις 4 γωνίες έξω ενώ η μέση του περνάει πάνω από το κτίριο."""
    if not fp:
        return True
    x1, y1, x2, y2 = bb
    step = fp['cell']*2
    nxs = max(2, int((x2-x1)/step)+1)
    nys = max(2, int((y2-y1)/step)+1)
    for i in range(nxs+1):
        px = x1 + (x2-x1)*i/nxs
        for j in range(nys+1):
            py = y1 + (y2-y1)*j/nys
            if point_inside(fp, px, py, pad):
                return False
    return True


def ray_exits(cx, cy, ux, uy, fp, pad=DETECT_PAD, ray_max=RAY_MAX,
              step=RAY_STEP, exit_dist=EXIT_DIST):
    """True αν η ακτίνα ΒΓΑΙΝΕΙ από το κτίριο μέσα σε `exit_dist` και ΔΕΝ ξαναμπαίνει
    μέχρι το `ray_max`.

    Αυτός είναι ο σωστός ορισμός του "προς τα έξω", και όχι το "είναι έξω σε
    συγκεκριμένες αποστάσεις ελέγχου": μια κολώνα που απέχει ένα μέτρο από την
    παρειά (πολύ συνηθισμένο) θα κοβόταν από τον απλό έλεγχο, ενώ μια ακτίνα προς
    εσωτερικό φωταγωγό θα περνούσε. Η συνθήκη "δεν ξαναμπαίνει" απορρίπτει τον
    φωταγωγό, η συνθήκη "βγαίνει γρήγορα" δέχεται την οπισθοχωρημένη κολώνα."""
    last_inside = -1.0
    d = 0.0
    while d <= ray_max:
        if point_inside(fp, cx+ux*d, cy+uy*d, pad):
            last_inside = d
        d += step
    return 0.0 <= last_inside <= exit_dist or last_inside < 0.0


def outward_direction(cx, cy, fp, n_dirs=N_PROBE_DIRS,
                      min_sector_deg=MIN_SECTOR_DEG, pad=DETECT_PAD):
    """Κατεύθυνση εξόδου για κολώνα με κέντρο (cx,cy).
    Επιστρέφει ((ux,uy), sector_deg) ή None αν η κολώνα είναι εσωτερική."""
    if not fp:
        return None

    outside = []
    for k in range(n_dirs):
        ang = 2*math.pi*k/n_dirs
        outside.append(ray_exits(cx, cy, math.cos(ang), math.sin(ang), fp, pad))

    if all(outside) or not any(outside):
        return None

    best_start, best_len = None, 0
    i = 0
    while i < n_dirs:
        if not outside[i]:
            i += 1
            continue
        j = i
        while j - i < n_dirs and outside[j % n_dirs]:
            j += 1
        if j - i > best_len:
            best_len, best_start = j - i, i
        i = j if j > i else i + 1

    if best_start is None:
        return None
    sector_deg = best_len * 360.0 / n_dirs
    if sector_deg < min_sector_deg:
        return None

    mid = (best_start + (best_len - 1) / 2.0) % n_dirs
    ang = 2*math.pi*mid/n_dirs
    return ((math.cos(ang), math.sin(ang)), sector_deg)


def _bbox_of_lines(lines, ox=0.0, oy=0.0):
    if not lines:
        return None
    xs = [p for seg in lines for p in (seg[0], seg[2])]
    ys = [p for seg in lines for p in (seg[1], seg[3])]
    return (min(xs)+ox, min(ys)+oy, max(xs)+ox, max(ys)+oy)


def column_centers(blocks, ins):
    """dict: όνομα block κολώνας -> (cx, cy) σε world coords."""
    out = {}
    for name, pairlist in blocks.items():
        if re.match(r'FL-?\d+_COLUMN\d*$', name) and 'TEXT' not in name:
            ox, oy = ins.get(name, (0, 0))
            lines, _ = block_lines_local(pairlist)
            bb = _bbox_of_lines(lines, ox, oy)
            if bb:
                out[name] = ((bb[0]+bb[2])/2, (bb[1]+bb[3])/2)
    return out


def column_outward_dirs(blocks, ins, fp=None, slab_boxes=None):
    """dict: όνομα block κολώνας -> ((ux,uy), sector_deg), μόνο για τις περιμετρικές."""
    if fp is None:
        fp = build_footprint(blocks, ins, slab_boxes)
    dirs = {}
    for cname, (cx, cy) in column_centers(blocks, ins).items():
        res = outward_direction(cx, cy, fp)
        if res is not None:
            dirs[cname] = res
    return dirs
