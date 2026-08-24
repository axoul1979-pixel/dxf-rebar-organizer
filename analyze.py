import re
from parse_dxf import load_lines, parse_pairs

def get_all_blocks(path):
    lines = load_lines(path)
    pairs = parse_pairs(lines)
    start=None; end=None
    for idx,(c,v) in enumerate(pairs):
        if c==2 and v=='BLOCKS':
            start=idx
        if c==0 and v=='ENDSEC' and start is not None and end is None and idx>start:
            end=idx; break
    blocks = pairs[start:end]
    result={}
    i=0
    while i < len(blocks):
        c,v = blocks[i]
        if c==0 and v=='BLOCK':
            j=i+1
            name=None
            while j<len(blocks) and blocks[j][0]!=0:
                if blocks[j][0]==2 and name is None:
                    name=blocks[j][1]
                j+=1
            k=j
            while k<len(blocks) and not(blocks[k][0]==0 and blocks[k][1]=='ENDBLK'):
                k+=1
            result[name]=blocks[i:k]
            i=k+1
        else:
            i+=1
    return result

def entities_from_pairs(pairlist):
    cur=[]; ents=[]
    for c,v in pairlist:
        if c==0:
            if cur: ents.append(cur)
            cur=[(c,v)]
        else:
            cur.append((c,v))
    if cur: ents.append(cur)
    return ents

def to_dict(ent):
    d={}
    for c,v in ent:
        d.setdefault(c,[]).append(v)
    return d

def get_points(ent_dict):
    pts=[]
    if 10 in ent_dict and 20 in ent_dict:
        pts.append((float(ent_dict[10][0]), float(ent_dict[20][0])))
    if 11 in ent_dict and 21 in ent_dict:
        pts.append((float(ent_dict[11][0]), float(ent_dict[21][0])))
    return pts

if __name__ == '__main__':
    bin_ = get_all_blocks('/mnt/user-data/uploads/input.dxf')
    bout = get_all_blocks('/mnt/user-data/uploads/output.dxf')
    diffs = open('diffs.txt').read().split('\n')

    results = {}
    for name in diffs:
        ein = entities_from_pairs(bin_[name])
        eout = entities_from_pairs(bout[name])
        pts_in=[]; pts_out=[]
        for a,b in zip(ein,eout):
            da = to_dict(a); db = to_dict(b)
            pin = get_points(da); pout = get_points(db)
            pts_in.extend(pin); pts_out.extend(pout)
        if len(pts_in)!=len(pts_out) or not pts_in:
            results[name] = None
            continue
        dxs = [b[0]-a[0] for a,b in zip(pts_in,pts_out)]
        dys = [b[1]-a[1] for a,b in zip(pts_in,pts_out)]
        results[name] = (dxs,dys,pts_in,pts_out)
    
    import statistics
    cats = {}
    for name,res in results.items():
        if res is None:
            print(name, 'POINT COUNT MISMATCH (possible rotation/reorder)')
            continue
        dxs,dys,pin,pout = res
        cat = re.sub(r'\d+$','', re.sub(r'^FL\d+_','',name))
        cats.setdefault(cat,[]).append((name, statistics.mean(dxs), statistics.mean(dys), max(dxs)-min(dxs), max(dys)-min(dys)))
    
    for cat, items in cats.items():
        print('===', cat, len(items))
        for it in items[:6]:
            print('  ', it)
