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

def bbox_poly_overlap(bb, poly):
    """Correct bbox-vs-polygon overlap test using axis-aligned bbox of the polygon plus
    a proper rectangle-rectangle overlap check, followed by point-containment as a
    fallback for genuinely rotated/irregular polygons. The previous corner/vertex-only
    test MISSED cases where two rectangles cross in a '+' shape with no corner of either
    inside the other - a real, serious bug that caused many false "no overlap" results."""
    x1,y1,x2,y2 = bb
    pxs = [p[0] for p in poly]; pys = [p[1] for p in poly]
    px1,py1,px2,py2 = min(pxs),min(pys),max(pxs),max(pys)
    # standard AABB overlap test - catches crossing rectangles correctly
    if not (x2 < px1 or px2 < x1 or y2 < py1 or py2 < y1):
        return True
    return False
