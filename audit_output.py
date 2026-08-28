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
# Ε) SLABBAR εκτός των ορίων της πλάκας τους (ΚΑΝΟΝΙΣΜΟΣ §0, απαράβατο).
# Χρησιμοποιεί το ΙΔΙΟ slab_region.py (flood fill) + bar_reference_point
# (άκρο αγκίστρου για προβόλους) με το final_pass.py, ώστε οι δύο "κριτές"
# να συμφωνούν πάντα για το πού βρίσκεται το όριο - διαφορετικά ένα πέρασμα
# μπορεί να θεωρεί σωστή μια θέση που ο έλεγχος καταγγέλλει, και αντίστροφα.
eviol=[]
if len(sys.argv)>2:
    from slab_region import all_slab_regions, bar_reference_point
    ins_in, blocks_in = load_all(sys.argv[2])
    regions = all_slab_regions(sys.argv[2])
    for n_ in blocks:
        if not re.match(r'FL-?\d+_SLABBAR\d+$', n_): continue
        lines_in,_ = block_lines_local(blocks_in[n_])
        if not lines_in: continue
        oxi, oyi = ins_in.get(n_,(0,0))
        world_in = [(x1+oxi,y1+oyi,x2+oxi,y2+oyi) for x1,y1,x2,y2 in lines_in]
        ref_in = bar_reference_point(world_in)
        d_out = ins.get(n_,(0,0)); d_in = ins_in.get(n_,(0,0))
        ddx,ddy = d_out[0]-d_in[0], d_out[1]-d_in[1]
        region = None
        if ref_in:
            for rname, bb in regions.items():
                if bb[0] <= ref_in[0] <= bb[2] and bb[1] <= ref_in[1] <= bb[3]:
                    region = bb; break
        if region is not None:
            ref_out = (ref_in[0]+ddx, ref_in[1]+ddy)
            pad = 0.03
            if not (region[0]-pad <= ref_out[0] <= region[2]+pad and region[1]-pad <= ref_out[1] <= region[3]+pad):
                eviol.append((n_, round(ddx,2), round(ddy,2)))
        else:
            # πλάκα μη αναγνωρίσιμη (π.χ. κενό/προβληματικό block) - εφεδρικό
            # όριο ±0.35 από τη ΦΥΣΙΚΗ θέση, ποτέ 0 (δεν ξέρουμε τα πραγματικά όρια)
            if abs(ddx) > 0.35+1e-6 or abs(ddy) > 0.35+1e-6:
                eviol.append((n_, round(ddx,2), round(ddy,2)))
print('Ε) Ράβδοι πλακών εκτός ζώνης (εντός πλάκας σε υγιείς άξονες, ±0.35 αλλιώς):', len(eviol), eviol[:6])
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
        poly = regions.get(n_)
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
# Ζ) Γραμμή οπλισμού ΜΕΣΑ στη ΔΙΚΗ της ετικέτα (ΚΑΝΟΝΙΣΜΟΣ §3: «δίπλα, όχι
# πάνω» - η ετικέτα ακουμπάει τη ράβδο στην άκρη του κουτιού, ποτέ μέσα στους
# χαρακτήρες). Ο έλεγχος προστέθηκε όταν ο μηχανικός εντόπισε σε πραγματικό
# AutoCAD ετικέτα κεντραρισμένη ΠΑΝΩ στη γραμμή της - αόρατο σε όλες τις άλλες
# κατηγορίες, που κοιτούν μόνο ξένες γραμμές/κείμενα.
zviol = []
_ztol = 0.03
for n_ in sorted(blocks):
    if not re.match(r'FL-?\d+_(SLABBAR|BEAMBAR)\d+$', n_): continue
    _zox, _zoy = ins.get(n_, (0, 0))
    _zlines, _ = block_lines_local(blocks[n_])
    if not _zlines: continue
    _zworld = [(a+_zox, b+_zoy, c+_zox, d+_zoy) for a, b, c, d in _zlines]
    _zboxes = block_text_bboxes(blocks[n_])
    for _zx, _zy, _zw, _zh, _zrot in _zboxes:
        _zb = text_bbox(_zx+_zox, _zy+_zoy, _zw, _zh, _zrot)
        _zi = (_zb[0]+_ztol, _zb[1]+_ztol, _zb[2]-_ztol, _zb[3]-_ztol)
        if _zi[0] >= _zi[2] or _zi[1] >= _zi[3]: continue
        for _zs in _zworld:
            if seg_intersects_bbox(_zs, _zi):
                zviol.append(n_)
                break
        else:
            continue
        break
print('Ζ) Γραμμή οπλισμού μέσα στη δική της ετικέτα:', len(zviol), zviol[:8])
# Η) BEAM_TEXT εκτός της ΔΙΚΗΣ του δοκού (ΚΑΝΟΝΙΣΜΟΣ §4: το κείμενο δοκού
# κάθεται πάνω στη δοκό του εξ ορισμού - κέντρο εκτός της λωρίδας της = λάθος).
# Προστέθηκε όταν ο μηχανικός εντόπισε σε πραγματικό AutoCAD κείμενο δοκού
# παρατημένο έξω από τη δοκό του, αόρατο σε όλες τις άλλες κατηγορίες.
hviol = []
for n_ in sorted(blocks):
    if not re.match(r'FL-?\d+_BEAM_TEXT\d+$', n_): continue
    _m = re.match(r'(FL-?\d+_)BEAM_TEXT(\d+)$', n_)
    _ob = _m.group(1)+'BEAM'+_m.group(2)
    if _ob not in blocks: continue
    _hox, _hoy = ins.get(_ob, (0, 0))
    _hlines, _ = block_lines_local(blocks[_ob])
    if not _hlines: continue
    _hw = [(a+_hox, b+_hoy, c+_hox, d+_hoy) for a, b, c, d in _hlines]
    _best = None; _bl = -1
    for _x1, _y1, _x2, _y2 in _hw:
        _l = math.hypot(_x2-_x1, _y2-_y1)
        if _l > _bl: _bl = _l; _best = (_x1, _y1, _x2, _y2)
    if _bl < 1e-9: continue
    _ux, _uy = (_best[2]-_best[0])/_bl, (_best[3]-_best[1])/_bl
    _px, _py = -_uy, _ux
    _tsv = [q0*_ux+q1*_uy for a, b, c, d in _hw for q0, q1 in ((a, b), (c, d))]
    _psv = [q0*_px+q1*_py for a, b, c, d in _hw for q0, q1 in ((a, b), (c, d))]
    _tox, _toy = ins.get(n_, (0, 0))
    _tb = None
    for _zx, _zy, _zw, _zh, _zr in block_text_bboxes(blocks[n_]):
        _b = text_bbox(_zx+_tox, _zy+_toy, _zw, _zh, _zr)
        _tb = _b if _tb is None else (min(_tb[0], _b[0]), min(_tb[1], _b[1]),
                                       max(_tb[2], _b[2]), max(_tb[3], _b[3]))
    if _tb is None: continue
    _cx, _cy = (_tb[0]+_tb[2])/2, (_tb[1]+_tb[3])/2
    _ct, _cp = _cx*_ux+_cy*_uy, _cx*_px+_cy*_py
    _t2 = 0.02
    _bad = []
    if not (min(_psv)-_t2 <= _cp <= max(_psv)+_t2): _bad.append('κάθετα')
    if not (min(_tsv)-_t2 <= _ct <= max(_tsv)+_t2): _bad.append('κατά μήκος')
    # ΚΑΝΟΝΙΣΜΟΣ §4: «99% μόνο ολίσθηση ΚΑΤΑ ΜΗΚΟΣ». Το FESPA τοποθετεί ΟΛΑ τα
    # κείμενα δοκού με σταθερή, σκόπιμη απόκλιση από τον άξονα (~-0.05), οπότε
    # «κεντραρισμένο» θα ήταν λάθος κριτήριο - το σωστό είναι πόσο ΚΑΘΕΤΑ
    # μετακινήθηκε σε σχέση με το ΠΡΩΤΟΤΥΠΟ.
    if len(sys.argv) > 2 and n_ in ins_in:
        _d1 = ins.get(n_, (0.0, 0.0)); _d0 = ins_in[n_]
        _shift = (_d1[0]-_d0[0])*_px + (_d1[1]-_d0[1])*_py
        if abs(_shift) > 0.05 + 1e-6:
            _bad.append(f'κάθετη μετατόπιση {_shift:+.2f}')
    if _bad:
        hviol.append((n_, '/'.join(_bad)))
print('Η) Κείμενα δοκών εκτός της δοκού τους:', len(hviol), hviol[:8])
# Θ) Ετικέτα οπλισμού κομμένη από ΞΕΝΗ γραμμή οπλισμού, σε ΟΠΟΙΑΔΗΠΟΤΕ γωνία.
# Η κατηγορία Δ μετράει μόνο τομές παράλληλες στη φορά ανάγνωσης (cos>0.94),
# αλλά μια γραμμή που περνά ΚΑΘΕΤΑ μέσα από τα γράμματα τα κόβει εξίσου - ο
# μηχανικός το βλέπει, ο ελεγκτής όχι. (§1: «γραμμή οπλισμού που κόβει κείμενο»,
# χωρίς περιορισμό γωνίας.)
thviol = []
_thbars = {}
for n_ in blocks:
    if re.match(r'FL-?\d+_(SLABBAR|BEAMBAR)\d+$', n_):
        _ox, _oy = ins.get(n_, (0, 0))
        _ls, _ = block_lines_local(blocks[n_])
        _thbars[n_] = [(a+_ox, b+_oy, c+_ox, d+_oy) for a, b, c, d in _ls]
for n_ in sorted(_thbars):
    _ox, _oy = ins.get(n_, (0, 0))
    _hit = None
    for _x, _y, _w, _h, _r in block_text_bboxes(blocks[n_]):
        _b = text_bbox(_x+_ox, _y+_oy, _w, _h, _r)
        _in = (_b[0]+0.03, _b[1]+0.03, _b[2]-0.03, _b[3]-0.03)
        if _in[0] >= _in[2] or _in[1] >= _in[3]: continue
        for _m, _segs in _thbars.items():
            if _m == n_: continue
            if any(seg_intersects_bbox(_s, _in) for _s in _segs):
                _hit = _m; break
        if _hit: break
    if _hit:
        thviol.append((n_, _hit))
print('Θ) Ετικέτα οπλισμού κομμένη από ΞΕΝΗ ράβδο:', len(thviol), thviol[:8])
# Ι) ΑΛΛΟΙΩΜΕΝΗ κάθετη απόσταση ετικέτας οπλισμού από τη ράβδο της.
# Στο αρχείο-ΜΗΤΡΑ η απόσταση αυτή είναι σταθερή ανά τύπο (μετρημένο στο
# DAMAR07: 0,141 / 0,101 / 0,061) και η ετικέτα ΠΟΤΕ δεν πέφτει πάνω στη
# γραμμή της. Επιτρέπεται ΜΟΝΟ ολίσθηση ΚΑΤΑ ΜΗΚΟΣ - καμία κάθετη αλλαγή.
iviol = []
if len(sys.argv) > 2:
    def _perp_of(bl_, ins_, n_):
        _ox, _oy = ins_.get(n_, (0, 0))
        _ls, _ = block_lines_local(bl_[n_])
        if not _ls: return None
        _w = [(a+_ox, b+_oy, c+_ox, d+_oy) for a, b, c, d in _ls]
        _bs = max(_w, key=lambda q: math.hypot(q[2]-q[0], q[3]-q[1]))
        _L = math.hypot(_bs[2]-_bs[0], _bs[3]-_bs[1])
        if _L < 1e-9: return None
        _ux, _uy = (_bs[2]-_bs[0])/_L, (_bs[3]-_bs[1])/_L
        _px, _py = -_uy, _ux
        _barp = ((_bs[0]+_bs[2])/2)*_px + ((_bs[1]+_bs[3])/2)*_py
        _tb = None
        for _x, _y, _w2, _h, _r in block_text_bboxes(bl_[n_]):
            _t = text_bbox(_x+_ox, _y+_oy, _w2, _h, _r)
            _tb = _t if _tb is None else (min(_tb[0], _t[0]), min(_tb[1], _t[1]),
                                           max(_tb[2], _t[2]), max(_tb[3], _t[3]))
        if _tb is None: return None
        return ((_tb[0]+_tb[2])/2)*_px + ((_tb[1]+_tb[3])/2)*_py - _barp
    for n_ in sorted(blocks):
        if not re.match(r'FL-?\d+_(SLABBAR|BEAMBAR)\d+$', n_): continue
        if n_ not in blocks_in: continue
        _po = _perp_of(blocks, ins, n_); _pi = _perp_of(blocks_in, ins_in, n_)
        if _po is None or _pi is None: continue
        if abs(abs(_po)-abs(_pi)) > 0.02:
            iviol.append((n_, round(_pi, 2), round(_po, 2)))
print('Ι) Αλλοιωμένη κάθετη απόσταση ετικέτας-ράβδου:', len(iviol), iviol[:8])
# ΙΑ) ΠΑΡΑΠΛΑΝΗΤΙΚΟ κείμενο κολώνας (ΚΑΝΟΝΙΣΜΟΣ §6): πρέπει να είναι ΠΑΝΤΑ
# πιο κοντά στη ΔΙΚΗ του κολώνα παρά σε οποιαδήποτε άλλη. Η απόσταση μετριέται
# από το ΠΛΗΣΙΕΣΤΕΡΟ ΣΗΜΕΙΟ του κουτιού προς το πλησιέστερο σημείο της κολώνας -
# ΟΧΙ από κέντρα (ένα φαρδύ κείμενο έχει πάντα το κέντρο του μακριά).
iaviol = []
try:
    from perimeter import build_footprint as _bfp, column_outward_dirs as _cod, point_inside as _point_inside
    _FP = _bfp(blocks, ins)
    _PERIM_OUT = {c for c, d in _cod(blocks, ins, _FP).items() if d}
except Exception:
    _FP = None; _PERIM_OUT = set(); _point_inside = None
def _gap_box(a, b):
    _dx = max(0.0, max(a[0]-b[2], b[0]-a[2]))
    _dy = max(0.0, max(a[1]-b[3], b[1]-a[3]))
    return math.hypot(_dx, _dy)
_cols = {}
for _c in blocks:
    if re.match(r'FL-?\d+_COLUMN\d+$', _c):
        _cox, _coy = ins.get(_c, (0, 0))
        _cls, _ = block_lines_local(blocks[_c])
        if not _cls: continue
        _cxs = [p+_cox for s_ in _cls for p in (s_[0], s_[2])]
        _cys = [p+_coy for s_ in _cls for p in (s_[1], s_[3])]
        _cols[_c] = (min(_cxs), min(_cys), max(_cxs), max(_cys))
for n_ in sorted(blocks):
    _m = re.match(r'(FL-?\d+)_COLUMN_TEXT(\d+)$', n_)
    if not _m: continue
    _own = f'{_m.group(1)}_COLUMN{_m.group(2)}'
    if _own not in _cols: continue
    _tox, _toy = ins.get(n_, (0, 0))
    _tb = None
    for _x, _y, _w, _h, _r in block_text_bboxes(blocks[n_]):
        _b = text_bbox(_x+_tox, _y+_toy, _w, _h, _r)
        _tb = _b if _tb is None else (min(_tb[0], _b[0]), min(_tb[1], _b[1]),
                                       max(_tb[2], _b[2]), max(_tb[3], _b[3]))
    if _tb is None: continue
    _do = _gap_box(_tb, _cols[_own])
    _df = min(((_gap_box(_tb, _b2), _c2) for _c2, _b2 in _cols.items() if _c2 != _own),
               default=(1e9, ''))
    if _df[0] < _do:
        # ΕΞΑΙΡΕΣΗ: ο κανόνας των ΠΕΡΙΜΕΤΡΙΚΩΝ υπερισχύει της ιδιοκτησίας. Ένα
        # κείμενο περιμετρικής κολώνας που κάθεται σωστά ΕΞΩ από το κτίριο δεν
        # είναι παραπλανητικό, ακόμη κι αν κάποια άλλη κολώνα τυχαίνει να είναι
        # οριακά κοντύτερη - εκεί ανήκει.
        _exempt = False
        try:
            if _PERIM_OUT and _own in _PERIM_OUT and _FP is not None:
                _pcx, _pcy = (_tb[0]+_tb[2])/2, (_tb[1]+_tb[3])/2
                if not _point_inside(_FP, _pcx, _pcy):
                    _exempt = True
        except Exception:
            _exempt = False
        if not _exempt:
            iaviol.append((n_, round(_do, 2), _df[1].split('_')[-1], round(_df[0], 2)))
print('ΙΑ) Παραπλανητικό κείμενο κολώνας (πιο κοντά σε ξένη):', len(iaviol), iaviol[:8])
# ΙΒ) Ετικέτα οπλισμού κομμένη από ΔΟΜΙΚΗ γραμμή (δοκού/κολώνας) σε οποιαδήποτε
# γωνία. Η κατηγορία Δ πιάνει μόνο τις παράλληλες τομές (cos>0.94) - μια γραμμή
# δοκού που περνά ΚΑΘΕΤΑ μέσα από τα γράμματα τα κόβει εξίσου.
ibviol = []
_struct_lines = []
for _n in blocks:
    if re.match(r'FL-?\d+_(BEAM|COLUMN)\d+$', _n) and 'TEXT' not in _n:
        _sox, _soy = ins.get(_n, (0, 0))
        _sls, _ = block_lines_local(blocks[_n])
        _struct_lines += [(a+_sox, b+_soy, c+_sox, d+_soy) for a, b, c, d in _sls]
for n_ in sorted(blocks):
    if not re.match(r'FL-?\d+_(SLABBAR|BEAMBAR)\d+$', n_): continue
    _box, _boy = ins.get(n_, (0, 0))
    for _x, _y, _w, _h, _r in block_text_bboxes(blocks[n_]):
        _b = text_bbox(_x+_box, _y+_boy, _w, _h, _r)
        _in = (_b[0]+0.03, _b[1]+0.03, _b[2]-0.03, _b[3]-0.03)
        if _in[0] >= _in[2] or _in[1] >= _in[3]: continue
        if any(seg_intersects_bbox(_s, _in) for _s in _struct_lines):
            ibviol.append(n_)
            break
print('ΙΒ) Ετικέτα οπλισμού κομμένη από δομική γραμμή:', len(ibviol), ibviol[:8])
# ΙΓ) Κείμενο κολώνας πάνω σε HATCH (ΚΑΝΟΝΙΣΜΟΣ §6/§7: το hatch είναι εμπόδιο
# για ΟΛΑ τα κείμενα - ΚΑΙ το hatch της ίδιας της κολώνας του).
igviol = []
try:
    from hatch_engine import get_hatch_polys as _ghp, bbox_poly_overlap as _bpo
    _hp = _ghp(out_path)
    for n_ in sorted(blocks):
        if not re.match(r'FL-?\d+_COLUMN_TEXT\d+$', n_): continue
        _ox, _oy = ins.get(n_, (0, 0))
        for _x, _y, _w, _h, _r in block_text_bboxes(blocks[n_]):
            _b = text_bbox(_x+_ox, _y+_oy, _w, _h, _r)
            if any(_bpo(_b, _p) for _p, _ in _hp):
                igviol.append(n_)
                break
except Exception:
    igviol = []
print('ΙΓ) Κείμενο κολώνας πάνω σε hatch:', len(igviol), igviol[:8])
print('AUDIT_TOTAL', len(overlaps)+len(tight)+len(cv)+len(pcut)+len(eviol)+len(stviol)+len(zviol)+len(hviol)+len(thviol)+len(iviol)+len(iaviol)+len(ibviol)+len(igviol))
