from pipeline_v11 import *
import pipeline_v11 as _P   # για ΖΩΝΤΑΝΗ πρόσβαση στο FOOTPRINT/COLUMN_OUTWARD που
                             # γεμίζουν μέσα στο process_all (το import * θα έπαιρνε
                             # στιγμιότυπο των αρχικών, άδειων τιμών)
from perimeter import bbox_outside
from analyze import entities_from_pairs, to_dict
import pickle, re, math

def own_of(name):
    m = re.match(r'(FL-?\d+_)(BEAM|COLUMN)_TEXT(\d+)$', name)
    if m: return f'{m.group(1)}{m.group(2)}{m.group(3)}'
    m = re.match(r'(FL-?\d+_)(BEAMBAR|SLABBAR)(\d+)$', name)
    if m:
        kind = 'BEAM' if m.group(2)=='BEAMBAR' else 'SLAB'
        return f'{m.group(1)}{kind}{m.group(3)}'
    return None

def get_all_boxes(blocks, ins, insert_final, text_local_final, exclude=None):
    boxes = []
    for name in blocks:
        if name == exclude: continue
        bl = block_text_bboxes(blocks[name])
        if not bl: continue
        dx,dy = insert_final.get(name,(0,0))
        if re.match(r'FL-?\d+_BEAMBAR',name) and dx<=-49: continue
        tdx,tdy = text_local_final.get(name,(0,0))
        for x,y,w,h,rot in bl:
            boxes.append((text_bbox(x+dx+tdx,y+dy+tdy,w,h,rot), name))
    return boxes

def get_all_lines(blocks, ins, insert_final, exclude=None):
    lines_out = []
    for name in blocks:
        if name == exclude: continue
        if not re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE|BEAMBAR|SLABBAR)\d*$', name) or 'TEXT' in name:
            continue
        dx,dy = insert_final.get(name,(0,0))
        if re.match(r'FL-?\d+_BEAMBAR',name) and dx<=-49: continue
        lines,_ = block_lines_local(blocks[name])
        for x1,y1,x2,y2 in lines:
            lines_out.append((x1+dx,y1+dy,x2+dx,y2+dy,name))
    return lines_out

def get_all_circles(blocks, ins, insert_final, text_local_final, exclude=None):
    circles = []
    for name in blocks:
        if name == exclude: continue
        if not re.match(r'FL-?\d+_SLAB\d+$', name): continue
        ox,oy = ins.get(name,(0,0))
        tdx,tdy = text_local_final.get(name,(0,0))
        for e in entities_from_pairs(blocks[name]):
            if e[0][1]=='CIRCLE':
                d=to_dict(e)
                if d.get(8,[''])[0]=='slab_center':
                    cx=float(d[10][0])+ox+tdx; cy=float(d[20][0])+oy+tdy; r=float(d[40][0])
                    circles.append((cx-r,cy-r,cx+r,cy+r,name))
    return circles

def find_conflicts(blocks, ins, insert_final, text_local_final, hatch_polys, rebar_names):
    all_lines = get_all_lines(blocks, ins, insert_final)
    all_boxes = get_all_boxes(blocks, ins, insert_final, text_local_final)
    all_circles = get_all_circles(blocks, ins, insert_final, text_local_final)
    conflicts = []
    for bb, name in all_boxes:
        own = own_of(name)
        bx1,by1,bx2,by2 = bb
        for x1,y1,x2,y2,oname in all_lines:
            if oname==name or oname==own: continue
            pad = 0.03
            test = (bx1-pad,by1-pad,bx2+pad,by2+pad)
            if seg_intersects_bbox((x1,y1,x2,y2), test):
                conflicts.append(('line', name, oname))
        for poly,pname in hatch_polys:
            if bbox_poly_overlap(bb, poly):
                conflicts.append(('hatch', name, pname))
        for cx1,cy1,cx2,cy2,cname in all_circles:
            if cname==name: continue
            if not(bx2<cx1 or cx2<bx1 or by2<cy1 or cy2<by1):
                conflicts.append(('circle', name, cname))
    for i in range(len(all_boxes)):
        for j in range(i+1,len(all_boxes)):
            b1,n1=all_boxes[i]; b2,n2=all_boxes[j]
            if n1==n2: continue
            if not(b1[2]<b2[0] or b2[2]<b1[0] or b1[3]<b2[1] or b2[3]<b1[1]):
                conflicts.append(('text', n1, n2))
    return conflicts

def try_move_text_only(name, blocks, insert_final, text_local_final, obstacle_lines, hatch_polys, placed_boxes, extra_bad_names=()):
    """try sliding just the text of `name` along its own bar/beam axis (or radially if it's
    a column_text/slab-marker with free movement) to clear ALL given obstacles."""
    if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', name):
        own_col = own_of(name)
        bl = block_text_bboxes(blocks[name])
        home_bb = union_bbox(bl)
        outward = _P.COLUMN_OUTWARD.get(name)
        fp = _P.FOOTPRINT
        # ίδιο seeding με το κύριο πέρασμα: κέντρο αναζήτησης = η ΠΡΑΓΜΑΤΙΚΗ κολώνα,
        # όχι το τοπικό μηδέν του κειμένου (που μπορεί να απέχει μέτρα)
        sdx, sdy = 0.0, 0.0
        col_lines,_ = block_lines_local(blocks[own_col]) if own_col in blocks else ([],None)
        if col_lines and bl:
            cxs=[p[0] for l in col_lines for p in [(l[0],l[1]),(l[2],l[3])]]
            cys=[p[1] for l in col_lines for p in [(l[0],l[1]),(l[2],l[3])]]
            sdx = (min(cxs)+max(cxs))/2 - (home_bb[0]+home_bb[2])/2
            sdy = (min(cys)+max(cys))/2 - (home_bb[1]+home_bb[3])/2
        # Ίδια λογική κόστους με το κύριο πέρασμα: αν η κολώνα είναι περιμετρική,
        # η επιδιόρθωση δεν επιτρέπεται να ξανατραβήξει το κείμενο προς τα μέσα.
        if _P.PERIM_OUTWARD and outward is not None:
            for require_outside in (True, False):
                if require_outside and fp is None:
                    continue
                c=0.0
                while c<=_P.PERIM_MAX_COST:
                    n_dirs=max(8,int(c/0.1)) if c>0 else 1
                    for k in range(n_dirs):
                        ang=2*math.pi*k/n_dirs if c>0 else 0
                        ux,uy=math.cos(ang),math.sin(ang)
                        f=1.0+_P.PERIM_DIR_WEIGHT*(1.0-(ux*outward[0]+uy*outward[1]))/2.0
                        r=c/f
                        if r>_P.PERIM_HARD_MAX_R:
                            continue
                        ddx,ddy=sdx+r*ux,sdy+r*uy
                        if require_outside and not bbox_outside(fp, full_bbox(bl,ddx,ddy)):
                            continue
                        if is_ok_full(bl,ddx,ddy,obstacle_lines,hatch_polys,placed_boxes,(own_col,)):
                            return ('insert', ddx,ddy)
                    c+=0.1
        r=0.0
        while r<=4.0:
            n_dirs=max(8,int(r/0.1)) if r>0 else 1
            for k in range(n_dirs):
                ang=2*math.pi*k/n_dirs if r>0 else 0
                ddx,ddy=sdx+r*math.cos(ang),sdy+r*math.sin(ang)
                if is_ok_full(bl,ddx,ddy,obstacle_lines,hatch_polys,placed_boxes,(own_col,)):
                    return ('insert', ddx,ddy)
            r+=0.1
        return None
    if re.match(r'FL-?\d+_SLAB\d+$', name):
        return None  # handled separately (has circle)
    if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', name):
        dx,dy = insert_final.get(name,(0,0))
        bl = block_lines_local(blocks[name])[0]
        boxes_local = block_text_bboxes(blocks[name])
        if not bl or not boxes_local: return None
        tdx,tdy,good = text_only_slide(name, blocks, dx, dy, obstacle_lines, hatch_polys, placed_boxes)
        if good: return ('text_local', tdx, tdy)
        return None
    if re.match(r'FL-?\d+_BEAM_TEXT\d+$', name):
        res = beam_text_slide(name, blocks, obstacle_lines, hatch_polys, placed_boxes)
        if res: return ('insert', res[0], res[1])
        return None
    return None

def try_move_bar(name, blocks, insert_final, obstacle_lines, hatch_polys, placed_boxes, slab_polys=None):
    if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', name):
        return None
    lines,_ = block_lines_local(blocks[name])
    if not lines: return None
    best=None;bl_=-1
    for x1,y1,x2,y2 in lines:
        l=math.hypot(x2-x1,y2-y1)
        if l>bl_: bl_=l;best=(x1,y1,x2,y2)
    x1,y1,x2,y2=best
    if bl_<1e-9: return None
    ux,uy=(x2-x1)/bl_,(y2-y1)/bl_
    px,py=-uy,ux
    dx,dy = insert_final.get(name,(0,0))
    boxes_local = block_text_bboxes(blocks[name])

    # SLABBAR must never leave its own slab's boundary during a nudge.
    poly = None
    if slab_polys and re.match(r'FL-?\d+_SLABBAR\d+$', name):
        native_xs = [p[0] for l in lines for p in [(l[0],l[1]),(l[2],l[3])]]
        native_ys = [p[1] for l in lines for p in [(l[0],l[1]),(l[2],l[3])]]
        ncx = (min(native_xs)+max(native_xs))/2 + dx
        ncy = (min(native_ys)+max(native_ys))/2 + dy
        best_slab=None; bd=1e9
        for sname, sbox in slab_polys.items():
            sx1,sy1,sx2,sy2 = sbox
            if sx1<=ncx<=sx2 and sy1<=ncy<=sy2:
                poly = sbox; break
            ccx,ccy = (sx1+sx2)/2,(sy1+sy2)/2
            d = math.hypot(ccx-ncx,ccy-ncy)
            if d<bd: bd=d; best_slab=sbox
        if poly is None:
            poly = best_slab

    for s in [0.05*k for k in range(1,30)]:
        for sign in (1,-1):
            ndx,ndy = dx+px*s*sign, dy+py*s*sign
            if poly:
                lines_chk = [(a+ndx,b+ndy,c+ndx,d+ndy) for a,b,c,d in lines]
                nxs = [p[0] for seg in lines_chk for p in [(seg[0],seg[1]),(seg[2],seg[3])]]
                nys = [p[1] for seg in lines_chk for p in [(seg[0],seg[1]),(seg[2],seg[3])]]
                px1,py1,px2,py2 = poly
                if min(nxs)<px1-0.01 or max(nxs)>px2+0.01 or min(nys)<py1-0.01 or max(nys)>py2+0.01:
                    continue
            lines_new = [(a+ndx,b+ndy,c+ndx,d+ndy,name) for a,b,c,d in lines]
            bar_ok = True
            for seg in lines_new:
                for ox1,oy1,ox2,oy2 in [(pb[0],pb[1],pb[2],pb[3]) for pb in placed_boxes]:
                    if seg_intersects_bbox(seg[:4], (ox1,oy1,ox2,oy2)):
                        bar_ok=False; break
                if not bar_ok: break
            if not bar_ok: continue
            if is_ok_full(boxes_local, ndx, ndy, obstacle_lines, hatch_polys, placed_boxes, (name,)):
                return (ndx,ndy)
    return None

def try_move_marker(name, blocks, ins, insert_final, text_local_final, obstacle_lines, hatch_polys, placed_boxes, slab_polys):
    boxes_local = slab_marker_boxes(blocks[name])
    if not boxes_local: return None
    circle_bbox_local = None
    for e in entities_from_pairs(blocks[name]):
        if e[0][1]=='CIRCLE':
            d=to_dict(e)
            if d.get(8,[''])[0]=='slab_center':
                cx=float(d[10][0]); cy=float(d[20][0]); rad=float(d[40][0])
                circle_bbox_local=(cx-rad,cy-rad,cx+rad,cy+rad)
    home_bb = union_bbox(boxes_local)
    if circle_bbox_local:
        home_bb=(min(home_bb[0],circle_bbox_local[0]),min(home_bb[1],circle_bbox_local[1]),
                  max(home_bb[2],circle_bbox_local[2]),max(home_bb[3],circle_bbox_local[3]))
    poly = slab_polys.get(name)
    r=0.0
    while r<=4.0:
        n_dirs=max(8,int(r/0.1)) if r>0 else 1
        for k in range(n_dirs):
            ang=2*math.pi*k/n_dirs if r>0 else 0
            ddx,ddy=r*math.cos(ang),r*math.sin(ang)
            cand_bb=translate_bbox(home_bb,ddx,ddy)
            if poly:
                px1,py1,px2,py2=poly
                if cand_bb[0]<px1 or cand_bb[1]<py1 or cand_bb[2]>px2 or cand_bb[3]>py2:
                    continue
            if not is_ok_full(boxes_local,ddx,ddy,obstacle_lines,hatch_polys,placed_boxes,(name,)):
                continue
            if circle_bbox_local:
                ccb=translate_bbox(circle_bbox_local,ddx,ddy)
                clear=True
                for ox1,oy1,ox2,oy2 in placed_boxes:
                    if not(ccb[2]<ox1 or ox2<ccb[0] or ccb[3]<oy1 or oy2<ccb[1]):
                        clear=False;break
                if not clear: continue
            return (ddx,ddy)
        r+=0.1
    return None
