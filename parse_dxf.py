import re, sys, json

def load_lines(path):
    with open(path, encoding='latin-1') as f:
        content = f.read()
    lines = content.split('\n')
    lines = [l.rstrip('\r') for l in lines]
    return lines

def parse_pairs(lines):
    # yield (code:int, value:str)
    i = 0
    pairs = []
    while i < len(lines)-1:
        code = lines[i].strip()
        val = lines[i+1]
        try:
            code = int(code)
        except:
            i += 1
            continue
        pairs.append((code, val))
        i += 2
    return pairs

def get_entities_section(pairs):
    # find ENTITIES section
    start = None
    end = None
    for idx,(c,v) in enumerate(pairs):
        if c==2 and v=='ENTITIES':
            start = idx
        if c==0 and v=='ENDSEC' and start is not None and end is None and idx>start:
            end = idx
            break
    return pairs[start:end]

def split_entities(pairs):
    entities = []
    cur = []
    for c,v in pairs:
        if c==0:
            if cur:
                entities.append(cur)
            cur = [(c,v)]
        else:
            cur.append((c,v))
    if cur:
        entities.append(cur)
    return entities

def entity_to_dict(ent):
    etype = ent[0][1]
    d = {'type': etype, 'codes': {}}
    for c,v in ent[1:]:
        d['codes'].setdefault(c, []).append(v)
    return d

def summarize(path):
    lines = load_lines(path)
    pairs = parse_pairs(lines)
    sec = get_entities_section(pairs)
    ents = split_entities(sec)
    ents = [entity_to_dict(e) for e in ents if e[0][1] not in ('SECTION',)]
    return ents

if __name__ == '__main__':
    path = sys.argv[1]
    ents = summarize(path)
    from collections import Counter
    cnt = Counter(e['type'] for e in ents)
    print(cnt)
