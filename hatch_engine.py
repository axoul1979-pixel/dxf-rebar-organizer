import re
from analyze import get_all_blocks, entities_from_pairs, to_dict
from engine import load_all

def get_hatch_polys(input_path):
    """Return list of (poly_points, block_name) for every HATCH entity across all blocks,
    in WORLD coords. Properly parses the HATCH boundary-path structure: vertices are the
    10/20 pairs that appear between the edge-count code (93) and the next major section
    (97 = source boundary objects count), using each line-edge's START point (72=1 edges).
    The very first 10/20 in a HATCH (before code 91) is the unrelated 'elevation point' and
    must NOT be treated as a boundary vertex - including it corrupts the whole polygon."""
    ins, blocks = load_all(input_path)
    polys = []
    for name, pairlist in blocks.items():
        ox, oy = ins.get(name, (0,0))
        ents = entities_from_pairs(pairlist)
        for e in ents:
            if e[0][1] != 'HATCH':
                continue
            pts = []
            in_boundary = False
            x = None
            for c, v in e:
                if c == 93:
                    in_boundary = True
                    continue
                if c == 97:
                    in_boundary = False
                    continue
                if not in_boundary:
                    continue
                if c == 10:
                    x = float(v)
                elif c == 20 and x is not None:
                    y = float(v)
                    pts.append((x+ox, y+oy))
                    x = None
            if len(pts) >= 3:
                polys.append((pts, name))
    return polys

def point_in_poly(px, py, poly):
    n = len(poly)
    inside = False
    x1,y1 = poly[-1]
    for x2,y2 in poly:
        if ((y1>py) != (y2>py)) and (px < (x2-x1)*(py-y1)/(y2-y1+1e-15)+x1):
            inside = not inside
        x1,y1 = x2,y2
    return inside

def _seg_intersect(p1, p2, p3, p4):
    """True αν το τμήμα p1-p2 τέμνει το τμήμα p3-p4 (γνήσια ή σε επαφή)."""
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    d1 = cross(p3, p4, p1); d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3); d4 = cross(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    def on_seg(p, q, r):
        return min(p[0],r[0])-1e-9 <= q[0] <= max(p[0],r[0])+1e-9 and \
               min(p[1],r[1])-1e-9 <= q[1] <= max(p[1],r[1])+1e-9
    if abs(d1) < 1e-9 and on_seg(p3, p1, p4): return True
    if abs(d2) < 1e-9 and on_seg(p3, p2, p4): return True
    if abs(d3) < 1e-9 and on_seg(p1, p3, p2): return True
    if abs(d4) < 1e-9 and on_seg(p1, p4, p2): return True
    return False

def bbox_poly_overlap(bb, poly):
    """Πραγματική τομή κουτιού-πολυγώνου (Κανονισμός §7: το hatch είναι εμπόδιο
    για όλα τα κείμενα, με το ΠΡΑΓΜΑΤΙΚΟ του σχήμα, όχι το bbox του). Η παλιά
    υλοποίηση συνέκρινε bbox-πολυγώνου εναντίον bbox-κειμένου: ένα Γ-σχήμα hatch
    (π.χ. δοκός με απότμηση/γωνία) έχει πολύ μικρότερο πραγματικό εμβαδόν από το
    ορθογώνιο περίγραμμά του, οπότε "εμπόδιζε" και τεράστια άδεια περιοχή γύρω
    του - 92 ψευδή ευρήματα στο DAMAR07. Τώρα: γρήγορη απόρριψη με bbox, μετά
    κορυφή-πολυγώνου-μέσα-σε-κουτί, γωνία-κουτιού-μέσα-σε-πολύγωνο, και τομή
    ακμών για στενές λωρίδες που διαπερνούν χωρίς καμία γωνία μέσα στην άλλη."""
    x1,y1,x2,y2 = bb
    pxs = [p[0] for p in poly]; pys = [p[1] for p in poly]
    px1,py1,px2,py2 = min(pxs),min(pys),max(pxs),max(pys)
    if x2 < px1 or px2 < x1 or y2 < py1 or py2 < y1:
        return False
    for px, py in poly:
        if x1 <= px <= x2 and y1 <= py <= y2:
            return True
    corners = [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
    for c in corners:
        if point_in_poly(c[0], c[1], poly):
            return True
    box_edges = [(corners[i], corners[(i+1)%4]) for i in range(4)]
    n = len(poly)
    for i in range(n):
        p3, p4 = poly[i], poly[(i+1)%n]
        for p1, p2 in box_edges:
            if _seg_intersect(p1, p2, p3, p4):
                return True
    return False
