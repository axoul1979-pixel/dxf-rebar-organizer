import re

def patch_slab_marker_geometry(input_path, output_path, marker_deltas):
    """marker_deltas: dict blockname -> (dx,dy) - moves the CIRCLE and slab_center LINE
    entities inside that BLOCK by the same delta already applied to its text, so the
    circle/crosshair and its label always move together as one rigid group."""
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]

    start = None; end = None
    for i in range(0, len(lines)-1):
        if lines[i].strip()=='2' and lines[i+1].strip()=='BLOCKS':
            start = i; break
    for i in range(start, len(lines)-1):
        if lines[i].strip()=='0' and lines[i+1].strip()=='ENDSEC':
            end = i; break

    i = start
    n_patched = 0
    cur_block = None
    while i < end:
        if lines[i].strip()=='0' and lines[i+1].strip()=='BLOCK':
            j = i+2
            name = None
            while j < end-1 and lines[j].strip() != '0':
                if lines[j].strip()=='2' and name is None:
                    name = lines[j+1].strip()
                j += 2
            cur_block = name
            i = j
            continue
        if lines[i].strip()=='0' and lines[i+1].strip()=='ENDBLK':
            cur_block = None
            i += 2
            continue
        if lines[i].strip()=='0' and lines[i+1].strip() in ('LINE','CIRCLE') and cur_block in marker_deltas:
            dx, dy = marker_deltas[cur_block]
            j = i+2
            layer = None
            coord_idxs = []  # (code, line_index) pairs for 10/20/11/21
            while j < end-1 and lines[j].strip() != '0':
                code = lines[j].strip()
                if code == '8' and layer is None:
                    layer = lines[j+1].strip()
                if code in ('10','20','11','21'):
                    coord_idxs.append((code, j+1))
                j += 2
            if layer == 'slab_center':
                for code, idx in coord_idxs:
                    cur = float(lines[idx])
                    if code in ('10','11'):
                        lines[idx] = repr(cur + dx)
                    elif code in ('20','21'):
                        lines[idx] = repr(cur + dy)
                n_patched += 1
            i = j
            continue
        i += 1

    sep = '\r\n' if uses_crlf else '\n'
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(sep.join(lines))
    return n_patched


def swap_marker_name_height(input_path, output_path):
    """ΚΑΝΟΝΑΣ ΑΠΟ ΤΟ MANUAL_MODEL του μηχανικού (καθολικό μοτίβο 10/11 δεικτών):
    μέσα σε κάθε δείκτη πλάκας, το όνομα «Π n» μπαίνει στην ΠΑΝΩ γραμμή και το
    «h=...» στην ΚΑΤΩ. Αν βρεθούν ανάποδα (Π κάτω από h=), ανταλλάσσονται οι
    θέσεις των δύο MTEXT (10/20). Εφαρμόζεται ΜΟΝΟ όταν ο δείκτης έχει ακριβώς
    2 MTEXT στο layer slab_center - σύνθετοι δείκτες μένουν ανέπαφοι."""
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]

    start = None; end = None
    for i in range(0, len(lines)-1):
        if lines[i].strip()=='2' and lines[i+1].strip()=='BLOCKS':
            start = i; break
    for i in range(start, len(lines)-1):
        if lines[i].strip()=='0' and lines[i+1].strip()=='ENDSEC':
            end = i; break

    i = start
    cur_block = None
    block_marks = {}   # block -> list of dicts {x_idx,y_idx,x,y,is_h}
    while i < end:
        s0, s1 = lines[i].strip(), lines[i+1].strip() if i+1 < len(lines) else ''
        if s0=='0' and s1=='BLOCK':
            j = i+2; name=None
            while j < end-1 and lines[j].strip()!='0':
                if lines[j].strip()=='2' and name is None:
                    name = lines[j+1].strip()
                j += 2
            cur_block = name; i = j; continue
        if s0=='0' and s1=='ENDBLK':
            cur_block=None; i+=2; continue
        if s0=='0' and s1=='MTEXT' and cur_block and re.match(r'FL-?\d+_SLAB\d+$', cur_block):
            j=i+2; layer=None; xi=yi=None; xv=yv=None; content=''
            while j < end-1 and lines[j].strip()!='0':
                code=lines[j].strip()
                if code=='8': layer=lines[j+1].strip()
                elif code=='10': xi=j+1; xv=float(lines[j+1])
                elif code=='20': yi=j+1; yv=float(lines[j+1])
                elif code in ('1','3'): content += lines[j+1]
                j+=2
            if layer=='slab_center' and xi is not None and yi is not None:
                block_marks.setdefault(cur_block, []).append(
                    dict(xi=xi, yi=yi, x=xv, y=yv, is_h=('h=' in content)))
            i=j; continue
        i+=1

    n_sw = 0
    for bn, marks in block_marks.items():
        if len(marks)!=2: continue
        hs=[m for m in marks if m['is_h']]; ps=[m for m in marks if not m['is_h']]
        if len(hs)!=1 or len(ps)!=1: continue
        h, p = hs[0], ps[0]
        if p['y'] < h['y'] - 1e-9:   # Π κάτω από h= -> ανταλλαγή
            lines[p['xi']], lines[h['xi']] = lines[h['xi']], lines[p['xi']]
            lines[p['yi']], lines[h['yi']] = lines[h['yi']], lines[p['yi']]
            n_sw += 1
    sep = '\r\n' if uses_crlf else '\n'
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(sep.join(lines))
    return n_sw


def collapse_bar_geometry(path, names):
    """ΕΦΕΔΡΕΙΑ ΦΟΥΡΚΕΤΑΣ: μέσα στα BLOCKS των `names`, προβάλλει ΟΛΕΣ τις LINE
    πάνω στον άξονα της μακρύτερης (κορμός) - το σχέδιο γίνεται απλή γραμμή,
    ίδιο κείμενο, ίδιες οντότητες."""
    import math
    with open(path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    L = [l[:-1] if l.endswith('\r') else l for l in raw.split('\n')]
    i = 0; cur = None; n_p = 0
    # 1ο πέρασμα: μάζεψε γραμμές ανά block για spine
    segs = {n: [] for n in names}
    idxs = {n: [] for n in names}
    while i < len(L)-1:
        if L[i].strip()=='0' and L[i+1].strip()=='BLOCK':
            cur = None
        if L[i].strip()=='2' and cur is None and i>0 and L[i-1].strip() in ('BLOCK','') :
            pass
        if L[i].strip()=='2' and L[i+1].strip() in names and cur is None:
            cur = L[i+1].strip()
        if L[i].strip()=='0' and L[i+1].strip()=='ENDBLK':
            cur = None
        if cur and L[i].strip()=='0' and L[i+1].strip()=='LINE':
            j = i+2; v = {}
            while j < len(L)-1 and L[j].strip() != '0':
                try: v[int(L[j].strip())] = j+1
                except: pass
                j += 2
            if all(k in v for k in (10,20,11,21)):
                x1=float(L[v[10]]); y1=float(L[v[20]]); x2=float(L[v[11]]); y2=float(L[v[21]])
                segs[cur].append((x1,y1,x2,y2)); idxs[cur].append(v)
            i = j; continue
        i += 1
    for n in names:
        if not segs[n]:
            continue
        mb=None; mbl=-1.0
        for x1,y1,x2,y2 in segs[n]:
            l2=math.hypot(x2-x1,y2-y1)
            if l2>mbl: mbl=l2; mb=(x1,y1,x2,y2)
        ux,uy=(mb[2]-mb[0])/mbl,(mb[3]-mb[1])/mbl
        nx,ny=-uy,ux; bx,by=mb[0],mb[1]
        for (x1,y1,x2,y2),v in zip(segs[n], idxs[n]):
            d1=(x1-bx)*nx+(y1-by)*ny; d2=(x2-bx)*nx+(y2-by)*ny
            L[v[10]]=f'{x1-d1*nx}'; L[v[20]]=f'{y1-d1*ny}'
            L[v[11]]=f'{x2-d2*nx}'; L[v[21]]=f'{y2-d2*ny}'
            n_p += 1
    out = ('\r\n' if uses_crlf else '\n').join(L)
    with open(path, 'w', encoding='latin-1', newline='') as f:
        f.write(out)
    return n_p
