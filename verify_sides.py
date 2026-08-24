import re, math, pickle
from engine import load_all, block_lines_local
from compute_beambar3 import compute_beambar_offsets
insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
ins, blocks = load_all('karaisk05_OR0.dxf')

# Πλευρά κάθε BEAMBAR ως προς τη δοκό της: ΦΥΣΙΚΗ θέση αρχείου εισόδου vs ΤΕΛΙΚΗ
_, debug = compute_beambar_offsets('karaisk05_OR0.dxf')

def side_of(name, dx, dy, beam):
    bm_lines,_ = block_lines_local(blocks[beam])
    bbest=None;bbl=-1
    for x1,y1,x2,y2 in bm_lines:
        l=math.hypot(x2-x1,y2-y1)
        if l>bbl:bbl=l;bbest=(x1,y1,x2,y2)
    bux,buy=(bbest[2]-bbest[0])/bbl,(bbest[3]-bbest[1])/bbl
    nx,ny=-buy,bux
    pv=[x*nx+y*ny for x1,y1,x2,y2 in bm_lines for x,y in [(x1,y1),(x2,y2)]]
    ref=sum(pv)/len(pv)
    ml,_ = block_lines_local(blocks[name])
    mb=None;mbl=-1
    for x1,y1,x2,y2 in ml:
        l=math.hypot(x2-x1,y2-y1)
        if l>mbl:mbl=l;mb=(x1,y1,x2,y2)
    d=((mb[0]+mb[2])/2+dx)*nx+((mb[1]+mb[3])/2+dy)*ny-ref
    return 1 if d>0 else -1

flips=[]; checked=0
for n in blocks:
    if not re.match(r'FL0_BEAMBAR\d+$',n): continue
    m=re.search(r'beam=(\S+)', debug.get(n,'') or '')
    if not m or m.group(1) not in blocks: continue
    dx,dy = insert_final.get(n,(0,0))
    if dx<=-49: continue
    nx0,ny0 = ins.get(n,(0,0))
    s_native = side_of(n, nx0, ny0, m.group(1))
    s_final  = side_of(n, dx, dy, m.group(1))
    checked+=1
    if s_native != s_final:
        flips.append((n, m.group(1)))
print(f'Έλεγχος πλευράς Άνω/Κάτω σε {checked} beambar (φυσική θέση εισόδου vs τελική):')
print('ΑΛΛΑΓΕΣ ΠΛΕΥΡΑΣ:', flips if flips else 'ΚΑΜΙΑ')
for n in ('FL0_BEAMBAR1','FL0_BEAMBAR4','FL0_BEAMBAR5'):
    m=re.search(r'beam=(\S+)', debug.get(n,'') or '')
    if not m: print(f'  {n}: (χωρίς αντιστοιχισμένη δοκό)'); continue
    dx,dy=insert_final.get(n,(0,0)); nx0,ny0=ins.get(n,(0,0))
    sn=side_of(n,nx0,ny0,m.group(1)); sf=side_of(n,dx,dy,m.group(1))
    lbl={1:'ΠΑΝΩ',-1:'ΚΑΤΩ'}
    print(f'  {n}: {m.group(1)}  φυσική={lbl[sn]}  τελική={lbl[sf]}  {"OK" if sn==sf else "!!!"}')
