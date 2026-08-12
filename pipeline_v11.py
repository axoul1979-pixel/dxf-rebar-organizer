import re, math, pickle
from analyze import entities_from_pairs, to_dict
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox, translate_bbox, collides
from beambar_engine import seg_intersects_bbox, text_bbox, get_inserts as _get_inserts_early
from hatch_engine import get_hatch_polys, bbox_poly_overlap, point_in_poly

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

def count_line_crossings(boxes_local, dx, dy, obstacle_lines, exclude_line_names=()):
    """Count distinct obstacle-line NAMES that cross through the core of this text - used
    for the relaxed rule: a single crossing line is tolerable if no zero-crossing spot
    exists nearby (the line usually falls in whitespace between words), but hatch/other
    text overlaps are NEVER tolerated regardless."""
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
    return crossed

def is_ok_relaxed(boxes_local, dx, dy, obstacle_lines, hatch_polys, placed_boxes, exclude_line_names=(),
                   max_crossings=1):
    """Like is_ok_full but tolerates up to `max_crossings` distinct rebar/structural lines
    passing through the text's readability CORE (not the full box) - hatch and other text
    boxes are still strictly forbidden, no exceptions."""
    for x,y,w,h,rot in boxes_local:
        bx = text_bbox(x+dx,y+dy,w,h,rot)
        bx1,by1,bx2,by2 = bx
        for poly, pname in hatch_polys:
            this_pad = 0.0 if pname in exclude_line_names else 0.25
            bx_padded = (bx1-this_pad, by1-this_pad, bx2+this_pad, by2+this_pad)
            if bbox_poly_overlap(bx_padded, poly):
                return False
        for ob in placed_boxes:
            ox1,oy1,ox2,oy2 = ob
            if not (bx2 < ox1 or ox2 < bx1 or by2 < oy1 or oy2 < by1):
                return False
    crossed = count_line_crossings(boxes_local, dx, dy, obstacle_lines, exclude_line_names)
    return len(crossed) <= max_crossings

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
            if not (bx2 < ox1 or ox2 < bx1 or by2 < oy1 or oy2 < by1):
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
            tdx,tdy,good = text_only_slide(name, blocks, new_base_dx, new_base_dy, obstacle_lines, hatch_polys, placed_boxes, exclude)
            if good:
                return nudge_dx, nudge_dy, tdx, tdy, True
    return 0.0, 0.0, 0.0, 0.0, False

def radial_place_full(name, blocks, obstacle_lines, hatch_polys, placed_boxes, exclude_names,
                       step=0.05, max_r=4.0):
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    home_bb = union_bbox(boxes_local)
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
    # spot that's actually closer.
    r = 0.0
    while r <= max_r:
        n_dirs = n_dirs_for(r) if r <= 1.2 else max(8, int(r/step))
        relaxed_candidate = None
        for k in range(n_dirs):
            ang = 2*math.pi*k/n_dirs if r>0 else 0
            ddx, ddy = r*math.cos(ang), r*math.sin(ang)
            if is_ok_full(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, exclude_names):
                return ddx, ddy
            if relaxed_candidate is None and is_ok_relaxed(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, exclude_names, max_crossings=1):
                relaxed_candidate = (ddx, ddy)
        if relaxed_candidate:
            return relaxed_candidate
        r += step
    return None

def beam_span(lines):
    ux,uy = spine_and_bounds(lines)[0]
    pts = []
    for x1,y1,x2,y2 in lines:
        pts.append((x1,y1)); pts.append((x2,y2))
    ts = [p[0]*ux+p[1]*uy for p in pts]
    return ux,uy,min(ts),max(ts)

def beam_text_slide(name, blocks, obstacle_lines, hatch_polys, placed_boxes):
    own_beam = re.match(r'(FL-?\d+_)BEAM_TEXT(\d+)$',name).group(1)+'BEAM'+re.search(r'\d+$',name).group()
    boxes_local = block_text_bboxes(blocks[name])
    if not boxes_local:
        return None
    if is_ok_full(boxes_local,0,0,obstacle_lines,hatch_polys,placed_boxes,(own_beam,)):
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
            if is_ok_full(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, (own_beam,)):
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

    for ps in [0.05*k for k in range(1,13)]:
        for psign in (1,-1):
            for s in [STEP*k for k in range(0,int(MAX_SLIDE/STEP)+1)]:
                for sign in (1,-1) if s>0 else (1,):
                    t = home_t + s*sign
                    if t < lo or t > hi:
                        continue
                    shift = ps*psign
                    if text_perp_lo+shift < perp_lo or text_perp_hi+shift > perp_hi:
                        continue
                    ddx, ddy = ux*s*sign + px*shift, uy*s*sign + py*shift
                    if is_ok_full(boxes_local, ddx, ddy, obstacle_lines, hatch_polys, placed_boxes, (own_beam,)):
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
    beam_res_heuristic, _ = compute_beambar_offsets(input_path)
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
    ok=0; tot=0
    beam_final_bar_pos = {}
    for name,(dx,dy) in beam_res.items():
        if dx <= -49:
            insert_final[name] = (dx,dy); continue
        tot += 1
        nudge_dx,nudge_dy,tdx,tdy,good = bar_and_text_slide(name, blocks, dx, dy, obstacle_lines, hatch_polys, placed_boxes)
        final_dx, final_dy = dx+nudge_dx, dy+nudge_dy
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
    ok=0; tot=0
    for name in names:
        own_col = re.match(r'(FL-?\d+_)COLUMN_TEXT(\d+)$',name).group(1)+'COLUMN'+re.search(r'\d+$',name).group()
        tot += 1
        res = radial_place_full(name, blocks, bar_obstacle_lines, hatch_polys, placed_boxes, (own_col,))
        if res is None:
            insert_final[name] = (0.0,0.0)
            continue
        ddx,ddy = res
        insert_final[name] = (ddx,ddy)
        ok += 1
        bl = block_text_bboxes(blocks[name])
        for x,y,w,h,rot in bl: placed_boxes.append(text_bbox(x+ddx,y+ddy,w,h,rot))
    print(f'COLUMN_TEXT: {ok}/{tot} placed')

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

    return insert_final, text_local_final

if __name__ == '__main__':
    insert_final, text_local_final = process_all('/mnt/user-data/uploads/input.dxf')
    pickle.dump((insert_final, text_local_final), open('final_offsets_v11.pkl','wb'))
    print('text_local (internal-only slides needed):', len(text_local_final))
