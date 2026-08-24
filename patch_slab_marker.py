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
