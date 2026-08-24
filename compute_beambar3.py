import re, math
from engine import load_all, block_lines_local, has_diagonal

TARGET_GAP = 0.12

def project_range(seg, ux, uy):
    x1,y1,x2,y2 = seg
    t1 = x1*ux+y1*uy; t2 = x2*ux+y2*uy
    return min(t1,t2), max(t1,t2)

def compute_beambar_offsets(input_path):
    ins, blocks = load_all(input_path)
    beam_edges = {}
    for name, pairlist in blocks.items():
        if re.match(r'FL-?\d+_BEAM\d+$', name):
            ox, oy = ins.get(name,(0,0))
            lines, _ = block_lines_local(pairlist)
            segs = [(x1+ox,y1+oy,x2+ox,y2+oy) for x1,y1,x2,y2 in lines]
            if segs:
                beam_edges[name] = segs

    results = {}
    debug = {}
    for name, pairlist in blocks.items():
        if not re.match(r'FL-?\d+_BEAMBAR\d+$', name):
            continue
        lines, mtexts = block_lines_local(pairlist)
        if not lines or not mtexts:
            continue
        if has_diagonal(lines):
            results[name] = (-50.0, 0.0); debug[name]='Z-shape'
            continue

        best=None; bl=-1
        for seg in lines:
            x1,y1,x2,y2=seg
            l = math.hypot(x2-x1,y2-y1)
            if l>bl: bl=l; best=seg
        x1,y1,x2,y2 = best
        L = bl
        if L < 1e-9: continue
        ux,uy = (x2-x1)/L, (y2-y1)/L
        nx,ny = -uy, ux
        spine_mid = ((x1+x2)/2, (y1+y2)/2)

        # hook direction: the OTHER (shorter) segment's direction away from the spine -
        # a real physical clue about which beam the bar anchors into.
        hook_dir = None
        for seg2 in lines:
            if seg2 == best: continue
            hx1,hy1,hx2,hy2 = seg2
            hl = math.hypot(hx2-hx1,hy2-hy1)
            if hl < 1e-9: continue
            # direction from the shared joint outward
            if math.hypot(hx1-x1,hy1-y1) < 1e-6 or math.hypot(hx1-x2,hy1-y2) < 1e-6:
                hook_dir = ((hx2-hx1)/hl, (hy2-hy1)/hl)
            else:
                hook_dir = ((hx1-hx2)/hl, (hy1-hy2)/hl)
            break

        candidates = []
        for bname, segs in beam_edges.items():
            bbest=None; bbl=-1
            for s in segs:
                sx1,sy1,sx2,sy2=s
                l=math.hypot(sx2-sx1,sy2-sy1)
                if l>bbl: bbl=l; bbest=s
            if bbl < 1e-9: continue
            bx1,by1,bx2,by2 = bbest
            bux,buy = (bx2-bx1)/bbl,(by2-by1)/bbl
            dot = min(1.0, abs(ux*bux+uy*buy))
            ang = math.degrees(math.acos(dot))
            if ang > 5:
                continue
            lo,hi = None,None
            for s in segs:
                a,b = project_range(s, bux, buy)
                lo = a if lo is None else min(lo,a)
                hi = b if hi is None else max(hi,b)
            bar_lo, bar_hi = project_range(best, bux, buy)
            overlap = min(hi,bar_hi) - max(lo,bar_lo)
            bar_len = max(bar_hi-bar_lo, 1e-6)
            overlap_frac = max(0.0, overlap) / bar_len
            perp_vals = [sx1*nx+sy1*ny for sx1,sy1,sx2,sy2 in segs]
            mean_perp = sum(perp_vals)/len(perp_vals)
            bar_perp = spine_mid[0]*nx+spine_mid[1]*ny
            dist = abs(bar_perp-mean_perp)
            candidates.append((dist, bname, segs, dist, overlap_frac))
        if not candidates:
            debug[name]='no candidate beam'
            continue
        # global combined score across ALL candidates - overlap is a strong structural
        # signal (a bar genuinely spanning most of a beam's length is very likely the
        # correct match even if a poorly-aligned closer beam exists) so it must compete
        # globally, not just as a tie-break among near-equal distances.
        def combined_score(c):
            _, bname, segs, dist, overlap_frac = c
            hook_bonus = 0.0
            if hook_dir:
                to_beam = ( sum(s[0]+s[2] for s in segs)/(2*len(segs)) - spine_mid[0],
                            sum(s[1]+s[3] for s in segs)/(2*len(segs)) - spine_mid[1] )
                tbl = math.hypot(*to_beam)
                if tbl > 1e-6:
                    to_beam_n = (to_beam[0]/tbl, to_beam[1]/tbl)
                    align = hook_dir[0]*to_beam_n[0] + hook_dir[1]*to_beam_n[1]
                    hook_bonus = -0.3*align
            return dist + (1.0-overlap_frac)*2.0 + hook_bonus
        candidates.sort(key=combined_score)
        best = candidates[0]
        score, bname, segs, dist, overlap_frac = best

        bar_perp = spine_mid[0]*nx+spine_mid[1]*ny
        edge_perps = [sx1*nx+sy1*ny for sx1,sy1,sx2,sy2 in segs]
        nearest_edge_perp = min(edge_perps, key=lambda e: abs(e-bar_perp))
        sign = 1 if nearest_edge_perp > bar_perp else -1
        target_perp = nearest_edge_perp - sign*TARGET_GAP
        s = target_perp - bar_perp
        dx, dy = nx*s, ny*s
        results[name] = (dx,dy)
        debug[name] = f'beam={bname} dist={dist:.3f} overlap={overlap_frac:.2f} s={s:.3f}'
    return results, debug

if __name__ == '__main__':
    res, debug = compute_beambar_offsets('/mnt/user-data/uploads/karaisk_input.dxf')
    for n in ['FL1_BEAMBAR4','FL1_BEAMBAR8','FL1_BEAMBAR10','FL1_BEAMBAR11','FL1_BEAMBAR44']:
        print(n, debug.get(n))
