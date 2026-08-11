import re, math
from engine import load_all, block_lines_local, has_diagonal, dist_point_seg

TARGET_GAP = 0.12

def seg_angle_family(x1,y1,x2,y2):
    ang = math.degrees(math.atan2(y2-y1, x2-x1)) % 180
    return ang

def load_beams(input_path, ins, blocks):
    beams = {}
    for name, pairlist in blocks.items():
        if re.match(r'FL\d+_BEAM\d+$', name):
            ox, oy = ins.get(name,(0,0))
            lines, _ = block_lines_local(pairlist)
            segs = [(x1+ox,y1+oy,x2+ox,y2+oy) for x1,y1,x2,y2 in lines]
            if segs:
                beams[name] = segs
    return beams

def beam_axis_info(segs):
    # beams here = 2 parallel long edges; determine axis direction + the two edge lines
    # pick the longest segment to define axis direction
    best=None;bl=-1
    for s in segs:
        x1,y1,x2,y2=s
        l=math.hypot(x2-x1,y2-y1)
        if l>bl: bl=l; best=s
    x1,y1,x2,y2 = best
    ux,uy = (x2-x1)/bl, (y2-y1)/bl
    return ux,uy

def project_range(seg, ux, uy):
    x1,y1,x2,y2 = seg
    t1 = x1*ux+y1*uy
    t2 = x2*ux+y2*uy
    return min(t1,t2), max(t1,t2)

def compute_beambar_offsets(input_path):
    ins, blocks = load_all(input_path)
    beams = load_beams(input_path, ins, blocks)
    beam_axis = {name: beam_axis_info(segs) for name,segs in beams.items()}

    results = {}
    debug = {}
    for name, pairlist in blocks.items():
        if not re.match(r'FL\d+_BEAMBAR\d+$', name):
            continue
        lines, mtexts = block_lines_local(pairlist)
        if not lines or not mtexts:
            continue
        if has_diagonal(lines):
            results[name] = (-50.0, 0.0)
            debug[name] = 'Z-shape'
            continue
        # spine = longest segment (along-beam direction)
        best=None; bl=-1
        for seg in lines:
            x1,y1,x2,y2=seg
            l = math.hypot(x2-x1,y2-y1)
            if l>bl: bl=l; best=seg
        x1,y1,x2,y2 = best
        L = bl
        if L < 1e-9:
            continue
        ux,uy = (x2-x1)/L, (y2-y1)/L
        nx,ny = -uy, ux
        spine_mid = ((x1+x2)/2, (y1+y2)/2)
        spine_t = spine_mid[0]*ux + spine_mid[1]*uy   # along-axis coord of bar (using bar's own axis)

        # find candidate beams: axis direction parallel (within 3 deg) to bar spine direction
        candidates = []
        for bname, (bux,buy) in beam_axis.items():
            # angle between (ux,uy) and (bux,buy), mod 180
            dot = abs(ux*bux+uy*buy)
            dot = min(1.0,dot)
            ang = math.degrees(math.acos(dot))
            if ang < 5:  # parallel
                # check overlap in axis range
                segs = beams[bname]
                lo,hi = None,None
                for s in segs:
                    a,b = project_range(s, bux, buy)
                    lo = a if lo is None else min(lo,a)
                    hi = b if hi is None else max(hi,b)
                bar_t = spine_mid[0]*bux+spine_mid[1]*buy
                if lo - 0.3 <= bar_t <= hi + 0.3:
                    # perpendicular distance from spine_mid to beam centerline
                    # use avg of two edges' perpendicular offset
                    perp_vals = []
                    for s in segs:
                        sx1,sy1,sx2,sy2 = s
                        # perpendicular coordinate of a point on this edge
                        pv = sx1*nx+sy1*ny
                        perp_vals.append(pv)
                    bar_perp = spine_mid[0]*nx+spine_mid[1]*ny
                    mean_perp = sum(perp_vals)/len(perp_vals)
                    dist = abs(bar_perp-mean_perp)
                    candidates.append((dist, bname, segs))
        if not candidates:
            debug[name]='no candidate beam'
            continue
        candidates.sort(key=lambda c: c[0])
        dist, bname, segs = candidates[0]

        # among segs (edges) of chosen beam, pick the nearer edge to spine_mid, compute needed shift along n
        bar_perp = spine_mid[0]*nx+spine_mid[1]*ny
        edge_perps = [ (sx1*nx+sy1*ny) for sx1,sy1,sx2,sy2 in segs ]
        # nearest edge perp value
        nearest_edge_perp = min(edge_perps, key=lambda e: abs(e-bar_perp))
        # required shift so bar_perp + s = nearest_edge_perp -+ TARGET_GAP (stay on same side)
        sign = 1 if nearest_edge_perp > bar_perp else -1
        target_perp = nearest_edge_perp - sign*TARGET_GAP
        s = target_perp - bar_perp
        dx, dy = nx*s, ny*s
        results[name] = (dx,dy)
        debug[name] = f'beam={bname} dist={dist:.3f} s={s:.3f}'
    return results, debug

if __name__ == '__main__':
    res, debug = compute_beambar_offsets('/mnt/user-data/uploads/input.dxf')
    from engine import load_all
    ins,_ = load_all('/mnt/user-data/uploads/input.dxf')
    from beambar_engine import get_inserts
    ins_out = dict((n,(x,y)) for n,x,y in get_inserts('/mnt/user-data/uploads/output.dxf'))
    for n in sorted(res):
        dx,dy = res[n]
        rx,ry = ins_out.get(n,(0,0))
        err = ((dx-rx)**2+(dy-ry)**2)**0.5
        flag = 'OK' if err<0.15 else 'DIFF'
        print(f'{n:18s} mine=({dx:+.3f},{dy:+.3f}) real=({rx:+.3f},{ry:+.3f}) err={err:.3f} {flag}  {debug.get(n,"")}')
