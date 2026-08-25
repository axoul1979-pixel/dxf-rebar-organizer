"""ΑΝΕΞΑΡΤΗΤΟΣ έλεγχος πάνω στο ΤΕΛΙΚΟ DXF. Α: επικαλύψεις κειμένων.
Β: οριακά κενά. Γ: κείμενα σε κυκλάκια. Δ: παράλληλες τομές (εξαιρείται το
ίδιο το block ΚΑΙ, για BEAM_TEXTn, η δική του δοκός BEAMn - η ετικέτα ζει
μέσα στη δοκό της, οι παρειές της περνούν δίπλα της εξ ορισμού)."""
import re, math, sys
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes
from compute_beamtext_slabmarker import slab_marker_boxes
from beambar_engine import text_bbox, seg_intersects_bbox
from analyze import entities_from_pairs, to_dict

out_path = sys.argv[1]
ins, blocks = load_all(out_path)

def own_struct(n):
    m = re.match(r'(FL-?\d+_)BEAM_TEXT(\d+)$', n)
    if m: return {n, m.group(1)+'BEAM'+m.group(2)}
    m = re.match(r'(FL-?\d+_)COLUMN_TEXT(\d+)$', n)
    if m: return {n, m.group(1)+'COLUMN'+m.group(2)}
    return {n}

def boxes_of(n):
    bl = slab_marker_boxes(blocks[n]) if re.match(r'FL-?\d+_SLAB\d+$', n) else block_text_bboxes(blocks[n])
    dx,dy = ins.get(n,(0,0))
    if re.match(r'FL-?\d+_BEAMBAR',n) and dx<=-49: return []
    return [(text_bbox(x+dx,y+dy,w,h,rot), rot) for x,y,w,h,rot in (bl or [])]

tn = [n for n in blocks if re.match(r'FL-?\d+_(BEAMBAR|SLABBAR|BEAM_TEXT|COLUMN_TEXT)\d+$',n) or re.match(r'FL-?\d+_SLAB\d+$',n)]
allb = {n: boxes_of(n) for n in tn}
keys = sorted(allb)

overlaps=set(); tight=[]
for i,n1 in enumerate(keys):
    for b1,_ in allb[n1]:
        for n2 in keys[i+1:]:
            for b2,_ in allb[n2]:
                gx = max(b1[0]-b2[2], b2[0]-b1[2]); gy = max(b1[1]-b2[3], b2[1]-b1[3])
                if gx<=0 and gy<=0: overlaps.add((n1,n2))
                else:
                    g=max(gx,gy)
                    if 0<=g<0.03: tight.append((round(g,3),n1,n2))
print('Α) Επικαλύψεις κειμένων:', len(overlaps), sorted(overlaps)[:8])
print('Β) Οριακά κενά (<0.03μ):', len(tight), sorted(tight)[:8])

circ=[]
for n in blocks:
    if not re.match(r'FL-?\d+_SLAB\d+$',n): continue
    ox,oy = ins.get(n,(0,0))
    for e in entities_from_pairs(blocks[n]):
        if e[0][1]=='CIRCLE':
            d=to_dict(e)
            if d.get(8,[''])[0]=='slab_center':
                cx=float(d[10][0])+ox; cy=float(d[20][0])+oy; r=float(d[40][0])
                circ.append((cx-r,cy-r,cx+r,cy+r,n))
cv=set()
for n1 in keys:
    for b1,_ in allb[n1]:
        for c in circ:
            if c[4]==n1: continue
            if not(b1[2]<c[0] or c[2]<b1[0] or b1[3]<c[1] or c[3]<b1[1]): cv.add((n1,c[4]))
print('Γ) Κείμενο πάνω σε κυκλάκι:', len(cv), sorted(cv)[:6])

all_lines=[]
for n in blocks:
    if re.match(r'FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE|BEAMBAR|SLABBAR)\d*$',n) and 'TEXT' not in n:
        d = ins.get(n,(0,0))
        if re.match(r'FL-?\d+_BEAMBAR',n) and d[0]<=-49: continue
        for x1,y1,x2,y2 in block_lines_local(blocks[n])[0]:
            all_lines.append((x1+d[0],y1+d[1],x2+d[0],y2+d[1],n))
from compute_beambar3 import compute_beambar_offsets as _cbo
_src_in = out_path  # ίδια blocks
try:
    _res_map, _dbg = _cbo(sys.argv[2]) if len(sys.argv)>2 else ({}, {})
except Exception:
    _dbg = {}
_bar_of_beam = {}
for _bn, _ds in _dbg.items():
    _m = re.search(r'beam=(\S+)', _ds or '')
    if _m:
        _bar_of_beam.setdefault(_m.group(1), set()).add(_bn)

pcut=set()
for n1 in keys:
    excl = own_struct(n1)
    m_bt = re.match(r'(FL-?\d+_)BEAM_TEXT(\d+)$', n1)
    is_beam_text = bool(m_bt)
    if is_beam_text:
        excl = set(excl) | _bar_of_beam.get(m_bt.group(1)+'BEAM'+m_bt.group(2), set())
    for bb,rot in allb[n1]:
        tx_,ty_ = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        for s in all_lines:
            if s[4] in excl: continue
            if is_beam_text and re.match(r'FL-?\d+_SLAB\d+$', s[4]):
                continue  # τα όρια πλακών τρέχουν πάνω στις παρειές των δοκών εξ ορισμού
            if not seg_intersects_bbox(s[:4], bb): continue
            lx,ly=s[2]-s[0],s[3]-s[1]; ll=math.hypot(lx,ly)
            if ll<1e-9: continue
            if abs((lx*tx_+ly*ty_)/ll)>0.94: pcut.add((n1,s[4]))
print('Δ) Παράλληλες τομές:', len(pcut), sorted(pcut)[:8])
import json
# Ε) SLABBAR χωρίς επαληθεύσιμη πλάκα που έχουν μετακινηθεί (input vs output insert)
eviol=[]
if len(sys.argv)>2:
    from pipeline_v11 import get_slab_polys as _gsp
    ins_in, blocks_in = load_all(sys.argv[2])
    sp_ = _gsp(sys.argv[2])
    for n_ in blocks:
        if not re.match(r'FL-?\d+_SLABBAR\d+$', n_): continue
        lines_,_ = block_lines_local(blocks_in[n_])
        if not lines_: continue
        xs0=[p for s_ in lines_ for p in (s_[0],s_[2])]; ys0=[p for s_ in lines_ for p in (s_[1],s_[3])]
        ncx,ncy=(min(xs0)+max(xs0))/2,(min(ys0)+max(ys0))/2
        has_home=False
        for sname,(px1,py1,px2,py2) in sp_.items():
            x_ok=(px2-px1)>=0.05; y_ok=(py2-py1)>=0.05
            if not x_ok and not y_ok: continue
            if x_ok and not(px1<=ncx<=px2): continue
            if y_ok and not(py1<=ncy<=py2): continue
            has_home=True; break
        d_out = ins.get(n_,(0,0)); d_in = ins_in.get(n_,(0,0))
        ddx,ddy = d_out[0]-d_in[0], d_out[1]-d_in[1]
        # ΚΑΘΟΛΙΚΗ πολιτική: κάθε ράβδος πλάκας μέσα σε ±0.35 από τη φυσική θέση
        if abs(ddx) > 0.35+1e-6 or abs(ddy) > 0.35+1e-6:
            eviol.append((n_, round(ddx,2), round(ddy,2)))
print('Ε) Ράβδοι πλακών εκτός ±0.35 από τη φυσική θέση:', len(eviol), eviol[:6])
# ΣΤ) Δείκτες πλακών (κείμενα+κύκλος): εντός της πλάκας τους στους υγιείς
# άξονες, και ±0.35 από τη ΦΥΣΙΚΗ θέση σε εκφυλισμένους άξονες ή σε πλάκες
# χωρίς αναγνωρίσιμο περίγραμμα. Ο,τι δεν μετριέται εδώ μπορεί να παραβιαστεί
# σιωπηλά - γι' αυτό ΚΑΘΕ κανόνας θέσης δείκτη μετριέται ρητά.
stviol=[]
if len(sys.argv)>2:
    def _marker_parts(blks, n_, ox, oy):
        mbl = slab_marker_boxes(blks[n_])
        if not mbl: return None, None
        xs=[]; ys=[]
        for x,y,w,h,rot in mbl:
            bb=text_bbox(x+ox,y+oy,w,h,rot); xs+=[bb[0],bb[2]]; ys+=[bb[1],bb[3]]
        ub=(min(xs),min(ys),max(xs),max(ys))
        cbx=None
        for e in entities_from_pairs(blks[n_]):
            if e[0][1]=='CIRCLE':
                d_=to_dict(e)
                if d_.get(8,[''])[0]=='slab_center':
                    cx=float(d_[10][0])+ox; cy=float(d_[20][0])+oy; r_=float(d_[40][0])
                    cbx=(cx-r_,cy-r_,cx+r_,cy+r_)
        return ub, cbx
    for n_ in sorted(blocks):
        if not re.match(r'FL-?\d+_SLAB\d+$', n_): continue
        ub_o, cb_o = _marker_parts(blocks, n_, *ins.get(n_,(0,0)))
        ub_i, cb_i = _marker_parts(blocks_in, n_, *ins_in.get(n_,(0,0)))
        if ub_o is None or ub_i is None: continue
        mvx=(ub_o[0]+ub_o[2])/2-(ub_i[0]+ub_i[2])/2
        mvy=(ub_o[1]+ub_o[3])/2-(ub_i[1]+ub_i[3])/2
        poly = sp_.get(n_)
        pieces=[ub_o]+([cb_o] if cb_o else [])
        bad=[]
        if poly:
            x_ok=(poly[2]-poly[0])>=0.05; y_ok=(poly[3]-poly[1])>=0.05
            for pb in pieces:
                if x_ok and (pb[0]<poly[0]-0.05 or pb[2]>poly[2]+0.05): bad.append('εκτός πλάκας x')
                if y_ok and (pb[1]<poly[1]-0.05 or pb[3]>poly[3]+0.05): bad.append('εκτός πλάκας y')
            if not x_ok and abs(mvx)>0.35+1e-6: bad.append('>±0.35 x')
            if not y_ok and abs(mvy)>0.35+1e-6: bad.append('>±0.35 y')
        else:
            if abs(mvx)>0.35+1e-6 or abs(mvy)>0.35+1e-6: bad.append('μη αναγν. πλάκα >±0.35')
        if bad:
            stviol.append((n_, '/'.join(sorted(set(bad))), round(mvx,2), round(mvy,2)))
print('ΣΤ) Δείκτες πλακών εκτός ορίων:', len(stviol), stviol[:6])
print('AUDIT_TOTAL', len(overlaps)+len(tight)+len(cv)+len(pcut)+len(eviol)+len(stviol))
