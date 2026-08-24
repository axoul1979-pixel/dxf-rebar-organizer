import re, math, pickle
from engine import load_all, block_lines_local
from compute_column_text import block_text_bboxes, union_bbox
from compute_beambar3 import compute_beambar_offsets
insert_final, text_local_final = pickle.load(open('offs.pkl','rb'))
ins, blocks = load_all('karaisk05_OR0.dxf')
_, debug = compute_beambar_offsets('karaisk05_OR0.dxf')

def spine(nm):
    L,_ = block_lines_local(blocks[nm]); b=None;bl=-1
    for x1,y1,x2,y2 in L:
        l=math.hypot(x2-x1,y2-y1)
        if l>bl:bl=l;b=(x1,y1,x2,y2)
    return b,bl,L

bt_centers=[]
for t in blocks:
    if re.match(r'FL0_BEAM_TEXT\d+$',t):
        bl = block_text_bboxes(blocks[t])
        if bl:
            u=union_bbox(bl); bt_centers.append((t,(u[0]+u[2])/2,(u[1]+u[3])/2))

flips_axis=[]; flips_text=[]; n_bars=0; n_textchecks=0
for n in blocks:
    if not re.match(r'FL0_BEAMBAR\d+$',n): continue
    dx,dy = insert_final.get(n,(0,0))
    if dx<=-49: continue
    nx0,ny0 = ins.get(n,(0,0))
    mb,mbl,mylines = spine(n)
    if mb is None or mbl<1e-9: continue
    n_bars+=1
    ux,uy=(mb[2]-mb[0])/mbl,(mb[3]-mb[1])/mbl
    pnx,pny=-uy,ux
    mid=lambda a,b:(((mb[0]+mb[2])/2+a)*pnx+((mb[1]+mb[3])/2+b)*pny)
    # (α) ως προς τον άξονα της αντιστοιχισμένης δοκού
    m=re.search(r'beam=(\S+)', debug.get(n,'') or '')
    if m and m.group(1) in blocks:
        bb,bbl,BL = spine(m.group(1))
        bux,buy=(bb[2]-bb[0])/bbl,(bb[3]-bb[1])/bbl
        bnx,bny=-buy,bux
        ref=sum(x*bnx+y*bny for x1,y1,x2,y2 in BL for x,y in [(x1,y1),(x2,y2)])/(2*len(BL))
        bmid=lambda a,b:(((mb[0]+mb[2])/2+a)*bnx+((mb[1]+mb[3])/2+b)*bny)
        if (bmid(nx0,ny0)-ref)*(bmid(dx,dy)-ref) < 0: flips_axis.append((n,m.group(1)))
    # (β) ως προς κάθε γειτονικό beam_text (κατά μήκος επικάλυψη στη φυσική θέση)
    pts=[(p[0]+nx0,p[1]+ny0) for l_ in mylines for p in [(l_[0],l_[1]),(l_[2],l_[3])]]
    ts=[p[0]*ux+p[1]*uy for p in pts]; lo,hi=min(ts),max(ts)
    for t,cx,cy in bt_centers:
        tal=cx*ux+cy*uy
        if tal<lo-0.3 or tal>hi+0.3: continue
        tp=cx*pnx+cy*pny
        d0=mid(nx0,ny0)-tp; d1=mid(dx,dy)-tp
        if abs(d0)<1e-6: continue
        n_textchecks+=1
        if d0*d1<0: flips_text.append((n,t))
print(f'Έλεγχος {n_bars} ενεργών beambar, {n_textchecks} έλεγχοι πλευράς έναντι beam_text:')
print('  Αλλαγές πλευράς ως προς άξονα δοκού:', flips_axis or 'ΚΑΜΙΑ')
print('  Αλλαγές πλευράς ως προς beam_text  :', flips_text or 'ΚΑΜΙΑ')
for n in ('FL0_BEAMBAR1','FL0_BEAMBAR4','FL0_BEAMBAR5','FL0_BEAMBAR38'):
    dx,dy=insert_final.get(n,(0,0))
    print(f'  {n}: offset=({dx:+.2f},{dy:+.2f})')
