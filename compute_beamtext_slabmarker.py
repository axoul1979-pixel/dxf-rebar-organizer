import re, math
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox, translate_bbox, collides

STEP = 0.15
MAX_RADIUS = 4.0

def spiral_place(name, blocks, obstacle_lines, placed_boxes, exclude_names=()):
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    home_bb = union_bbox(boxes_local)
    my_obstacles = [s for s in obstacle_lines if s[4] not in exclude_names]
    best = None
    r = 0.0
    while r <= MAX_RADIUS:
        n_dirs = max(8, int(r/STEP)) if r>0 else 1
        for k in range(n_dirs):
            ang = 2*math.pi*k/n_dirs if r>0 else 0
            ddx, ddy = r*math.cos(ang), r*math.sin(ang)
            cand_bb = translate_bbox(home_bb, ddx, ddy)
            if not collides(cand_bb, my_obstacles, placed_boxes):
                return (ddx, ddy, cand_bb)
        r += STEP
    return None

def compute_beam_text_offsets(input_path, obstacle_lines, placed_boxes):
    ins, blocks = load_all(input_path)
    names = sorted([n for n in blocks if re.match(r'FL-?\d+_BEAM_TEXT\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    results = {}
    for name in names:
        own_beam = 'FL0_BEAM'+re.search(r'\d+$',name).group()
        # Determine required vertical side from content: Άνω (top) => must end up ABOVE the
        # beam's own line; Κάτω (bottom) => must end up BELOW it. This is a hard semantic
        # constraint, not just "nearest free spot".
        boxes_local = block_text_bboxes(blocks[name])
        if not boxes_local:
            continue
        all_content = ' '.join(_mtext_raw_content(blocks[name]))
        wants_above = ('\\U+0391\\U+03BD\\U+03C9' in all_content) or ('Ανω' in all_content) or ('Άνω' in all_content)
        wants_below = ('\\U+039A\\U+03AC\\U+03C4\\U+03C9' in all_content) or ('Κάτω' in all_content) or ('Κατω' in all_content)

        beam_lines,_ = block_lines_local(blocks.get(own_beam, [])) if own_beam in blocks else ([], None)
        beam_y = None
        if beam_lines:
            ys = [p for seg in beam_lines for p in (seg[1], seg[3])]
            beam_y = sum(ys)/len(ys)

        res = spiral_place_directional(name, blocks, obstacle_lines, placed_boxes, (own_beam,),
                                        beam_y=beam_y, wants_above=wants_above, wants_below=wants_below)
        if res is None:
            continue
        ddx, ddy, cand_bb = res
        placed_boxes.append(cand_bb)
        results[name] = (ddx, ddy)
    return results

def _mtext_raw_content(pairlist):
    from analyze import entities_from_pairs, to_dict
    ents = entities_from_pairs(pairlist)
    out = []
    for e in ents:
        if e[0][1]=='MTEXT':
            d = to_dict(e)
            out.append(d.get(1,[''])[0])
    return out

def spiral_place_directional(name, blocks, obstacle_lines, placed_boxes, exclude_names,
                              beam_y=None, wants_above=False, wants_below=False):
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    home_bb = union_bbox(boxes_local)
    my_obstacles = [s for s in obstacle_lines if s[4] not in exclude_names]

    def side_ok(cand_bb):
        if beam_y is None:
            return True
        cy = (cand_bb[1]+cand_bb[3])/2
        if wants_above and not wants_below:
            return cy >= beam_y
        if wants_below and not wants_above:
            return cy <= beam_y
        return True

    best = None
    r = 0.0
    while r <= MAX_RADIUS:
        n_dirs = max(8, int(r/STEP)) if r>0 else 1
        for k in range(n_dirs):
            ang = 2*math.pi*k/n_dirs if r>0 else 0
            ddx, ddy = r*math.cos(ang), r*math.sin(ang)
            cand_bb = translate_bbox(home_bb, ddx, ddy)
            if not side_ok(cand_bb):
                continue
            if not collides(cand_bb, my_obstacles, placed_boxes):
                return (ddx, ddy, cand_bb)
        r += STEP
    # if constrained search failed entirely, fall back to unconstrained (better than nothing)
    r = 0.0
    while r <= MAX_RADIUS:
        n_dirs = max(8, int(r/STEP)) if r>0 else 1
        for k in range(n_dirs):
            ang = 2*math.pi*k/n_dirs if r>0 else 0
            ddx, ddy = r*math.cos(ang), r*math.sin(ang)
            cand_bb = translate_bbox(home_bb, ddx, ddy)
            if not collides(cand_bb, my_obstacles, placed_boxes):
                return (ddx, ddy, cand_bb)
        r += STEP
    return None

def slab_marker_boxes(pairlist):
    """Extract just the marker MTEXT (slab_name/slab_prefix_name layers), not slab_poly lines.
    Width from actual decoded characters, NOT code 41 (which is an oversized reference width)."""
    from analyze import entities_from_pairs, to_dict
    from beambar_engine import text_width
    ents = entities_from_pairs(pairlist)
    boxes = []
    for e in ents:
        if e[0][1] != 'MTEXT':
            continue
        d = to_dict(e)
        layer = d.get(8,[''])[0]
        if layer not in ('slab_name','slab_prefix_name'):
            continue
        x = float(d[10][0]); y = float(d[20][0])
        h = float(d.get(40,['0.1'])[0])
        # FESPA MTEXT συχνά ΔΕΝ έχει κωδικό 50 - η στροφή δηλώνεται στο
        # διάνυσμα 11/21. Χωρίς αυτό, κάθε κατακόρυφο κείμενο μοντελοποιείται
        # ξαπλωμένο και οι συγκρούσεις του γίνονται αόρατες.
        from beambar_engine import mtext_rotation_deg
        rot = mtext_rotation_deg(d)
        content = d.get(1,[''])[0]
        w = text_width(content, h)
        boxes.append((x,y,w,h,rot))
    return boxes

def compute_slab_marker_offsets(input_path, obstacle_lines, placed_boxes):
    ins, blocks = load_all(input_path)
    names = sorted([n for n in blocks if re.match(r'FL-?\d+_SLAB\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))
    results = {}
    for name in names:
        boxes_local = slab_marker_boxes(blocks[name])
        if not boxes_local:
            continue
        home_bb = union_bbox(boxes_local)
        my_obstacles = [s for s in obstacle_lines if s[4] != name]
        best = None
        r = 0.0
        while r <= MAX_RADIUS:
            n_dirs = max(8, int(r/STEP)) if r>0 else 1
            for k in range(n_dirs):
                ang = 2*math.pi*k/n_dirs if r>0 else 0
                ddx, ddy = r*math.cos(ang), r*math.sin(ang)
                cand_bb = translate_bbox(home_bb, ddx, ddy)
                if not collides(cand_bb, my_obstacles, placed_boxes):
                    best = (ddx, ddy, cand_bb)
                    break
            if best: break
            r += STEP
        if best is None:
            continue
        ddx, ddy, cand_bb = best
        placed_boxes.append(cand_bb)
        results[name] = (ddx, ddy)
    return results
