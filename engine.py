import re, math
from analyze import get_all_blocks, entities_from_pairs, to_dict
from parse_dxf import load_lines, parse_pairs, get_entities_section, split_entities, entity_to_dict
from beambar_engine import get_inserts, strip_mtext_formatting, text_width, longest_segment, text_bbox, bbox_overlap, seg_intersects_bbox

# ---------- World geometry loading ----------

def load_all(path):
    ins = dict((n,(x,y)) for n,x,y in get_inserts(path))
    blocks = get_all_blocks(path)
    return ins, blocks

def block_lines_local(pairlist):
    ents = entities_from_pairs(pairlist)
    lines=[]; mtexts=[]
    for e in ents:
        d = to_dict(e)
        typ = e[0][1]
        if typ=='LINE':
            lines.append((float(d[10][0]),float(d[20][0]),float(d[11][0]),float(d[21][0])))
        elif typ=='MTEXT':
            mtexts.append(dict(x=float(d[10][0]), y=float(d[20][0]),
                                h=float(d.get(40,['0.1'])[0]), rot=float(d.get(50,['0'])[0]),
                                content=d.get(1,[''])[0]))
    return lines, mtexts

def all_world_lines(ins, blocks, exclude_prefixes=()):
    """All LINE segments from all blocks, in world coords (using current ins offsets)."""
    out = []
    for name, pairlist in blocks.items():
        if any(name.startswith(p) or re.match(p, name) for p in exclude_prefixes):
            continue
        ox, oy = ins.get(name, (0,0))
        lines, _ = block_lines_local(pairlist)
        for x1,y1,x2,y2 in lines:
            out.append((x1+ox,y1+oy,x2+ox,y2+oy,name))
    return out

def all_world_texts(ins, blocks):
    out = []
    for name, pairlist in blocks.items():
        ox, oy = ins.get(name, (0,0))
        _, mtexts = block_lines_local(pairlist)
        for m in mtexts:
            out.append(dict(x=m['x']+ox, y=m['y']+oy, h=m['h'], rot=m['rot'], content=m['content'], block=name))
    return out

# ---------- geometry helpers ----------

def dist_point_seg(px,py,x1,y1,x2,y2):
    dx,dy = x2-x1,y2-y1
    l2 = dx*dx+dy*dy
    if l2 < 1e-12: return math.hypot(px-x1,py-y1)
    t = max(0,min(1, ((px-x1)*dx+(py-y1)*dy)/l2))
    cx,cy = x1+t*dx, y1+t*dy
    return math.hypot(px-cx,py-cy)

def min_dist_to_segs(px,py, segs):
    best = 1e9
    for x1,y1,x2,y2,*_ in segs:
        d = dist_point_seg(px,py,x1,y1,x2,y2)
        if d<best: best=d
    return best

def has_diagonal(lines):
    for x1,y1,x2,y2 in lines:
        if math.hypot(x2-x1,y2-y1) < 1e-6: continue
        ang = math.degrees(math.atan2(y2-y1,x2-x1)) % 90
        if min(ang, 90-ang) > 5:
            return True
    return False
