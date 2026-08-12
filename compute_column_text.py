import re, math
from engine import load_all, block_lines_local
from beambar_engine import strip_mtext_formatting, text_width, text_bbox, seg_intersects_bbox, bbox_overlap

PAD = 0.05
STEP = 0.15
MAX_RADIUS = 4.0

def block_text_bboxes(pairlist):
    """Return list of (x,y,w,h,rot) rotated-rect boxes for all MTEXT in block (local coords).
    IMPORTANT: code 41 (MTEXT reference width) is NOT the tight width of the rendered text -
    it can be several times larger (a column/reference width unrelated to content length).
    The true box must stop where the last character ends, so width is estimated from the
    actual decoded character content instead."""
    from analyze import entities_from_pairs, to_dict
    from beambar_engine import strip_mtext_formatting, text_width
    ents = entities_from_pairs(pairlist)
    boxes = []
    for e in ents:
        if e[0][1] != 'MTEXT':
            continue
        d = to_dict(e)
        x = float(d[10][0]); y = float(d[20][0])
        h = float(d.get(40,['0.1'])[0])
        rot = float(d.get(50,['0'])[0])
        content = d.get(1,[''])[0]
        w = text_width(content, h)
        boxes.append((x, y, w, h, rot))
    return boxes

def union_bbox(boxes):
    xs1=[];ys1=[];xs2=[];ys2=[]
    for x,y,w,h,rot in boxes:
        bx = text_bbox(x,y,w,h,rot)
        xs1.append(bx[0]); ys1.append(bx[1]); xs2.append(bx[2]); ys2.append(bx[3])
    return (min(xs1),min(ys1),max(xs2),max(ys2))

def translate_bbox(bb, dx, dy):
    x1,y1,x2,y2 = bb
    return (x1+dx,y1+dy,x2+dx,y2+dy)

def collides(bb, obstacle_lines, obstacle_boxes, shrink=0.4):
    # shrink bbox toward its center before testing against LINE obstacles: bounding boxes are
    # conservative (declared width), but a thin line grazing the edge/whitespace of a text box
    # is tolerated in practice - only real intrusion into the glyph-dense core counts.
    x1,y1,x2,y2 = bb
    cx,cy = (x1+x2)/2, (y1+y2)/2
    hw,hh = (x2-x1)/2*shrink, (y2-y1)/2*shrink
    core = (cx-hw,cy-hh,cx+hw,cy+hh)
    for seg in obstacle_lines:
        if seg_intersects_bbox(seg[:4], core):
            return True
    for ob in obstacle_boxes:
        if bbox_overlap(bb, ob, pad=PAD):
            return True
    return False

def compute_column_text_offsets(input_path):
    ins, blocks = load_all(input_path)

    # obstacle lines: all COLUMN, BEAM, SLAB*, FREENODE geometry (world, using given ins - beams already at raw 0,0 here
    # since we run this independently; caller may pass beam-updated ins if desired)
    obstacle_lines = []
    for name, pairlist in blocks.items():
        if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE)\d*$', name) and 'TEXT' not in name:
            ox,oy = ins.get(name,(0,0))
            lines,_ = block_lines_local(pairlist)
            for x1,y1,x2,y2 in lines:
                obstacle_lines.append((x1+ox,y1+oy,x2+ox,y2+oy,name))

    def own_column_name(ct_name):
        num = re.search(r'\d+$', ct_name).group()
        return f'FL0_COLUMN{num}'

    placed_boxes = []  # boxes for column_texts already placed (avoid overlapping each other)
    results = {}
    debug = {}

    names = sorted([n for n in blocks if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))

    for name in names:
        boxes_local = block_text_bboxes(blocks[name])
        if not boxes_local:
            continue
        local_bb = union_bbox(boxes_local)  # in local block coords
        # search origin = NATIVE local position (offset 0,0), not the input's existing offset
        home_bb = local_bb

        own_col = own_column_name(name)
        my_obstacles = [s for s in obstacle_lines if s[4] != own_col]

        best = None
        r = 0.0
        found = False
        while r <= MAX_RADIUS and not found:
            n_dirs = max(8, int(r/STEP)) if r>0 else 1
            for k in range(n_dirs):
                ang = 2*math.pi*k/n_dirs if r>0 else 0
                ddx, ddy = r*math.cos(ang), r*math.sin(ang)
                cand_bb = translate_bbox(home_bb, ddx, ddy)
                if not collides(cand_bb, my_obstacles, placed_boxes):
                    best = (ddx, ddy, cand_bb)
                    found = True
                    break
            r += STEP
        if best is None:
            debug[name] = 'NO FREE SPOT FOUND'
            continue
        ddx, ddy, cand_bb = best
        placed_boxes.append(cand_bb)
        results[name] = (ddx, ddy)
        debug[name] = f'moved r={math.hypot(ddx,ddy):.3f}'
    return results, debug

if __name__ == '__main__':
    res, debug = compute_column_text_offsets('/mnt/user-data/uploads/input.dxf')
    from beambar_engine import get_inserts
    ins_out = dict((n,(x,y)) for n,x,y in get_inserts('/mnt/user-data/uploads/output.dxf'))
    for n in sorted(res, key=lambda n:int(re.search(r'\d+$',n).group())):
        dx,dy = res[n]
        rx,ry = ins_out.get(n,(0,0))
        print(f'{n:18s} mine=({dx:+.3f},{dy:+.3f}) real=({rx:+.3f},{ry:+.3f})  {debug.get(n,"")}')
