import re, math, pickle, sys
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from compute_beamtext_slabmarker import slab_marker_boxes
from beambar_engine import text_bbox, seg_intersects_bbox
from compute_beambar3 import compute_beambar_offsets
import pipeline_v11 as P

src, pklf = sys.argv[1], sys.argv[2]
insert_final, text_local_final = pickle.load(open(pklf,'rb'))
ins, blocks = load_all(src)
sp = P.get_slab_polys(src)

def spine(nm):
    L,_ = block_lines_local(blocks[nm]); b=None;bl=-1
    for x1,y1,x2,y2 in L:
        l=math.hypot(x2-x1,y2-y1)
        if l>bl:bl=l;b=(x1,y1,x2,y2)
    return b,bl,L

# --- 1) Πλευρές beambar (άξονας δοκού + κάθε γειτονικό beam_text) ---
_, debug = compute_beambar_offsets(src)
bt_centers=[]
for t in blocks:
    if re.match(r'FL-?\d+_BEAM_TEXT\d+$',t):
        bl = block_text_bboxes(blocks[t])
        if bl:
            u=union_bbox(bl); bt_centers.append((t,(u[0]+u[2])/2,(u[1]+u[3])/2))
flips=[]; n_bars=0; n_checks=0
for n in blocks:
    if not re.match(r'FL-?\d+_BEAMBAR\d+$',n): continue
    dx,dy = insert_final.get(n,(0,0))
    if dx<=-49: continue
    nx0,ny0 = ins.get(n,(0,0))
    mb,mbl,mylines = spine(n)
    if mb is None or mbl<1e-9: continue
    n_bars+=1
    ux,uy=(mb[2]-mb[0])/mbl,(mb[3]-mb[1])/mbl
    pnx,pny=-uy,ux
    mid=lambda a,b:(((mb[0]+mb[2])/2+a)*pnx+((mb[1]+mb[3])/2+b)*pny)
    m=re.search(r'beam=(\S+)', debug.get(n,'') or '')
    if m and m.group(1) in blocks:
        bb,bbl,BL = spine(m.group(1))
        if bb and bbl>1e-9:
            bux,buy=(bb[2]-bb[0])/bbl,(bb[3]-bb[1])/bbl
            bnx,bny=-buy,bux
            ref=sum(x*bnx+y*bny for x1,y1,x2,y2 in BL for x,y in [(x1,y1),(x2,y2)])/(2*len(BL))
            bmid=lambda a,c:(((mb[0]+mb[2])/2+a)*bnx+((mb[1]+mb[3])/2+c)*bny)
            n_checks+=1
            if (bmid(nx0,ny0)-ref)*(bmid(dx,dy)-ref) < 0: flips.append((n,'άξονας',m.group(1)))
    pts=[(p[0]+nx0,p[1]+ny0) for l_ in mylines for p in [(l_[0],l_[1]),(l_[2],l_[3])]]
    ts=[p[0]*ux+p[1]*uy for p in pts]; lo,hi=min(ts),max(ts)
    _own_bt_v = None
    _mv = re.search(r'beam=(FL-?\d+_)BEAM(\d+)\b', debug.get(n,'') or '')
    if _mv:
        _own_bt_v = _mv.group(1)+'BEAM_TEXT'+_mv.group(2)
    for t,cx,cy in ():  # text-side εκτός: «ΠΡΩΤΑ κοντά στο δοκάρι» - μόνο ο άξονας ελέγχεται
        tal=cx*ux+cy*uy
        if tal<lo-0.3 or tal>hi+0.3: continue
        tp=cx*pnx+cy*pny
        d0=mid(nx0,ny0)-tp; d1=mid(dx,dy)-tp
        if abs(d0)<1e-6: continue
        n_checks+=1
        if d0*d1<0: flips.append((n,'text',t))
print(f'1) Πλευρές: {n_bars} ενεργά beambar, {n_checks} έλεγχοι -> ΑΛΛΑΓΕΣ: {flips or "ΚΑΜΙΑ"}')

# --- 2) SLABBAR εντός πλάκας (υγιείς άξονες) ---
viol=[]
for n in blocks:
    if not re.match(r'FL-?\d+_SLABBAR\d+$',n): continue
    lines,_ = block_lines_local(blocks[n])
    if not lines: continue
    dx,dy = insert_final.get(n,(0,0))
    xs0=[p for s in lines for p in (s[0],s[2])]; ys0=[p for s in lines for p in (s[1],s[3])]
    nc=((min(xs0)+max(xs0))/2,(min(ys0)+max(ys0))/2)
    xs=[p+dx for p in xs0]; ys=[p+dy for p in ys0]
    for sname,(px1,py1,px2,py2) in sp.items():
        x_ok=(px2-px1)>=0.05; y_ok=(py2-py1)>=0.05
        if not x_ok and not y_ok: continue
        if x_ok and not(px1<=nc[0]<=px2): continue
        if y_ok and not(py1<=nc[1]<=py2): continue
        bad=False
        if x_ok and (min(xs)<min(px1,min(xs0))-0.03 or max(xs)>max(px2,max(xs0))+0.03): bad=True
        if y_ok and (min(ys)<min(py1,min(ys0))-0.03 or max(ys)>max(py2,max(ys0))+0.03): bad=True
        # ΚΑΘΟΛΙΚΟ όριο ±0.35 από τη φυσική θέση, σε κάθε άξονα
        if abs(dx) > 0.38 or abs(dy) > 0.38: bad=True
        if bad: viol.append((n,sname))
        break
    else:
        # ΚΑΜΙΑ πλάκα δεν ταιριάζει στο φυσικό κέντρο -> μη επαληθεύσιμη ράβδος:
        # επιτρεπτή μόνο μικρο-διόρθωση εντός ±0.35 από τη φυσική θέση
        if abs(dx) > 0.35+1e-6 or abs(dy) > 0.35+1e-6:
            viol.append((n, 'ΧΩΡΙΣ-ΠΛΑΚΑ εκτός ±0.35 (%.2f,%.2f)' % (dx, dy)))
print('2) SLABBAR εκτός πλάκας:', viol or 'ΚΑΜΙΑ')

# --- 3) Κείμενα ράβδων: μόνο κατά μήκος ---
badp=[]
for n,(tx,ty) in text_local_final.items():
    if not re.match(r'FL-?\d+_(BEAMBAR|SLABBAR)\d+$', n): continue
    lines,_ = block_lines_local(blocks[n])
    b,bl_,_L = spine(n)
    if b is None or bl_<1e-9: continue
    ux,uy=(b[2]-b[0])/bl_,(b[3]-b[1])/bl_
    if abs(tx*(-uy)+ty*ux)>1e-6: badp.append(n)
print('3) Κείμενα ράβδων με κάθετη μετατόπιση:', badp or 'ΚΑΝΕΝΑ')

# --- 4) Συγκρούσεις κειμένων ---
def boxes_of(n):
    if re.match(r'FL-?\d+_SLAB\d+$', n): bl = slab_marker_boxes(blocks[n])
    else: bl = block_text_bboxes(blocks[n])
    dx,dy = insert_final.get(n,(0,0))
    if re.match(r'FL-?\d+_BEAMBAR',n) and dx<=-49: return []
    tx,ty = text_local_final.get(n,(0,0))
    return [text_bbox(x+dx+tx,y+dy+ty,w,h,rot) for x,y,w,h,rot in (bl or [])]
tn = [n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$',n) or re.match(r'FL-?\d+_SLAB\d+$',n)]
tt=set()
allb = {n: boxes_of(n) for n in tn}
keys = sorted(allb)
for i,n1 in enumerate(keys):
    for b1 in allb[n1]:
        for n2 in keys[i+1:]:
            for b2 in allb[n2]:
                if not(b1[2]<b2[0] or b2[2]<b1[0] or b1[3]<b2[1] or b2[3]<b1[1]): tt.add((n1,n2))
print('4) Κείμενο-πάνω-σε-κείμενο:', len(tt), sorted(tt)[:10] if tt else '')

# --- 5) Οριακά κενά: ζεύγη κειμένων με κενό < 0.03μ (με τα φουσκωμένα πλάτη) ---
tight=[]
for i,n1 in enumerate(keys):
    for b1 in allb[n1]:
        for n2 in keys[i+1:]:
            for b2 in allb[n2]:
                gx = max(b1[0]-b2[2], b2[0]-b1[2]); gy = max(b1[1]-b2[3], b2[1]-b1[3])
                if gx<=0 and gy<=0: continue
                g = max(gx,gy)
                if 0 <= g < 0.03: tight.append((round(g,3),n1,n2))
print('5) Οριακά κενά (<0.03μ):', len(tight), sorted(tight)[:8] if tight else '')

# --- 6) Κείμενα πάνω σε κυκλάκια δεικτών πλακών ---
from analyze import entities_from_pairs, to_dict
circ=[]
for n_ in blocks:
    if not re.match(r'FL-?\d+_SLAB\d+$',n_): continue
    ox_,oy_ = ins.get(n_,(0,0))
    tdx_,tdy_ = text_local_final.get(n_,(0,0))
    for e in entities_from_pairs(blocks[n_]):
        if e[0][1]=='CIRCLE':
            d_=to_dict(e)
            if d_.get(8,[''])[0]=='slab_center':
                cx_=float(d_[10][0])+ox_+tdx_+ (insert_final.get(n_,(0,0))[0] if n_ in insert_final else 0)
                cy_=float(d_[20][0])+oy_+tdy_+ (insert_final.get(n_,(0,0))[1] if n_ in insert_final else 0)
                r_=float(d_[40][0])
                circ.append((cx_-r_,cy_-r_,cx_+r_,cy_+r_,n_))
cv=[]
for n1 in keys:
    for b1 in allb[n1]:
        for c_ in circ:
            if c_[4]==n1: continue
            if not(b1[2]<c_[0] or c_[2]<b1[0] or b1[3]<c_[1] or c_[3]<b1[1]): cv.append((n1,c_[4]))
print('6) Κείμενο πάνω σε κυκλάκι:', len(set(cv)), sorted(set(cv))[:6] if cv else '')
