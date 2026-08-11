import re, math
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox, translate_bbox, collides
from beambar_engine import seg_intersects_bbox

STEP = 0.03
MAX_S = 3.0

def spine_dir(lines):
    best=None;bl=-1
    for seg in lines:
        x1,y1,x2,y2=seg
        l=math.hypot(x2-x1,y2-y1)
        if l>bl: bl=l;best=seg
    x1,y1,x2,y2=best
    L=bl
    if L<1e-9: return (1,0)
    return ((x2-x1)/L,(y2-y1)/L)

def compute_slabbar_offsets(input_path):
    ins, blocks = load_all(input_path)

    obstacle_lines = []
    for name, pairlist in blocks.items():
        if re.match(r'FL\d+_(COLUMN|BEAM|SLAB|FREENODE)\d*$', name) and 'TEXT' not in name and 'SLABBAR' not in name:
            ox,oy = ins.get(name,(0,0))
            lines,_ = block_lines_local(pairlist)
            for x1,y1,x2,y2 in lines:
                obstacle_lines.append((x1+ox,y1+oy,x2+ox,y2+oy,name))

    names = sorted([n for n in blocks if re.match(r'FL\d+_SLABBAR\d+$', n)],
                    key=lambda n: int(re.search(r'\d+$', n).group()))

    placed_boxes = []
    placed_lines = list(obstacle_lines)
    results = {}
    debug = {}

    for name in names:
        lines, _ = block_lines_local(blocks[name])
        boxes_local = block_text_bboxes(blocks[name])
        if not lines or not boxes_local:
            continue
        home_bb = union_bbox(boxes_local)
        ux, uy = spine_dir(lines)
        px, py = -uy, ux  # perpendicular

        best = None
        # try s=0 first (maybe already readable), then expand perpendicular in both directions
        for s in [0.0] + [STEP*k for k in range(1,int(MAX_S/STEP)+1)]:
            for sign in ([1] if s==0 else [1,-1]):
                ddx, ddy = px*s*sign, py*s*sign
                cand_bb = translate_bbox(home_bb, ddx, ddy)
                cand_line_bb = [(x1+ddx,y1+ddy,x2+ddx,y2+ddy) for x1,y1,x2,y2 in lines]
                if collides(cand_bb, placed_lines, placed_boxes):
                    continue
                best = (ddx, ddy, cand_bb, cand_line_bb)
                break
            if best is not None:
                break
        if best is None:
            debug[name]='NO FREE SPOT'
            continue
        ddx, ddy, cand_bb, cand_line_bb = best
        placed_boxes.append(cand_bb)
        for lseg in cand_line_bb:
            placed_lines.append((lseg[0],lseg[1],lseg[2],lseg[3],name))
        results[name] = (ddx, ddy)
        debug[name] = f's={math.hypot(ddx,ddy):.3f}'
    return results, debug

if __name__ == '__main__':
    res, debug = compute_slabbar_offsets('/mnt/user-data/uploads/input.dxf')
    from beambar_engine import get_inserts
    ins_out = dict((n,(x,y)) for n,x,y in get_inserts('/mnt/user-data/uploads/output.dxf'))
    ok=0; tot=0
    for n in sorted(res, key=lambda n:int(re.search(r'\d+$',n).group())):
        dx,dy = res[n]
        rx,ry = ins_out.get(n,(0,0))
        err = math.hypot(dx-rx,dy-ry)
        tot+=1
        if err<0.25: ok+=1
        flag='OK' if err<0.25 else 'DIFF'
        print(f'{n:16s} mine=({dx:+.3f},{dy:+.3f}) real=({rx:+.3f},{ry:+.3f}) err={err:.3f} {flag}')
    print(ok,'/',tot)
