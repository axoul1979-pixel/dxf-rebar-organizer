import re, math
from analyze import get_all_blocks, entities_from_pairs, to_dict
from parse_dxf import load_lines, parse_pairs, get_entities_section, split_entities, entity_to_dict

def get_inserts(path):
    lines = load_lines(path)
    pairs = parse_pairs(lines)
    sec = get_entities_section(pairs)
    ents = split_entities(sec)
    ents = [entity_to_dict(e) for e in ents if e[0][1]=='INSERT']
    out = []
    for e in ents:
        c = e['codes']
        name = c.get(2,[''])[0]
        x = float(c.get(10,['0'])[0]); y = float(c.get(20,['0'])[0])
        out.append((name, x, y))
    return out

def char_width_factor(ch):
    # rough relative widths (relative to text height). Calibrated up ~1.35x from initial
    # estimate after confirming against real rendered text in AutoCAD - the original factors
    # were systematically too narrow.
    if ch in ' .,\'':
        return 0.34
    if ch in 'iIl|':
        return 0.40
    if ch.isupper() or ch.isdigit():
        return 0.84
    return 0.74

def strip_mtext_formatting(raw):
    def repl(m):
        return chr(int(m.group(1), 16))
    s = re.sub(r'\\U\+([0-9A-Fa-f]{4})', repl, raw)
    s = re.sub(r'\\[A-Za-z](\{[^}]*\}|[^\\;]*;)?', '', s)
    s = s.replace('\\P', ' ').replace('{','').replace('}','')
    return s

def text_width(content, height):
    s = strip_mtext_formatting(content)
    w = sum(char_width_factor(ch) for ch in s) * height
    return max(w, height*0.5)

class Block:
    def __init__(self, name, ents):
        self.name = name
        self.ents = ents
        self.lines = []   # list of (x1,y1,x2,y2)
        self.text = None  # dict: x,y,height,rot,content,raw_entity_index
        for idx, e in enumerate(ents):
            d = to_dict(e)
            typ = e[0][1]
            if typ == 'LINE':
                x1=float(d[10][0]); y1=float(d[20][0]); x2=float(d[11][0]); y2=float(d[21][0])
                self.lines.append((x1,y1,x2,y2))
            elif typ == 'MTEXT':
                x=float(d[10][0]); y=float(d[20][0])
                h=float(d.get(40,['0.1'])[0])
                rot=float(d.get(50,['0'])[0])
                content = d.get(1,[''])[0]
                self.text = dict(x=x,y=y,h=h,rot=rot,content=content, ent_idx=idx)

def load_beambars(path, prefix='BEAMBAR'):
    blocks = get_all_blocks(path)
    result = {}
    for name in blocks:
        if re.match(rf'FL\d+_{prefix}\d+$', name):
            ents = entities_from_pairs(blocks[name])
            b = Block(name, ents)
            if b.text is not None and b.lines:
                result[name] = b
    return result

def longest_segment(lines):
    best = None; bestlen=-1
    for seg in lines:
        x1,y1,x2,y2 = seg
        l = math.hypot(x2-x1,y2-y1)
        if l > bestlen:
            bestlen = l; best = seg
    return best

def text_bbox(x, y, w, h, rot, attach=1):
    # DXF MTEXT attachment point 1 = Top-Left: box extends right and DOWN from (x,y)
    corners = [(0,0),(w,0),(w,-h),(0,-h)]
    rad = math.radians(rot)
    cos,sin = math.cos(rad), math.sin(rad)
    pts = [(x + cx*cos - cy*sin, y + cx*sin + cy*cos) for cx,cy in corners]
    xs = [p[0] for p in pts]; ys=[p[1] for p in pts]
    return (min(xs),min(ys),max(xs),max(ys))

def bbox_overlap(a,b, pad=0.0):
    ax1,ay1,ax2,ay2 = a
    bx1,by1,bx2,by2 = b
    return not (ax2+pad < bx1 or bx2+pad < ax1 or ay2+pad < by1 or by2+pad < ay1)

def seg_intersects_bbox(seg, bbox):
    x1,y1,x2,y2 = seg
    bx1,by1,bx2,by2 = bbox
    # quick reject
    if max(x1,x2) < bx1 or min(x1,x2) > bx2 or max(y1,y2) < by1 or min(y1,y2) > by2:
        return False
    # sample points along segment (cheap robust check for near-axis-aligned bar lines)
    n = 20
    for i in range(n+1):
        t = i/n
        px = x1+(x2-x1)*t
        py = y1+(y2-y1)*t
        if bx1<=px<=bx2 and by1<=py<=by2:
            return True
    return False
