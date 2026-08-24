import re

def patch_dxf(input_path, output_path, offsets):
    """offsets: dict blockname -> (dx,dy) to ADD to the INSERT's 10/20 codes."""
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]

    # find ENTITIES section bounds
    start = None; end = None
    for i in range(0, len(lines)-1, 1):
        if lines[i].strip()=='2' and lines[i+1].strip()=='ENTITIES':
            start = i
            break
    for i in range(start, len(lines)-1):
        if lines[i].strip()=='0' and lines[i+1].strip()=='ENDSEC':
            end = i
            break

    i = start
    n_patched = 0
    while i < end:
        if lines[i].strip()=='0' and lines[i+1].strip()=='INSERT':
            # scan this entity until next code-0
            j = i+2
            name = None
            code10_idx = None
            code20_idx = None
            while j < end-1 and lines[j].strip() != '0':
                code = lines[j].strip()
                if code == '2' and name is None:
                    name = lines[j+1].strip()
                if code == '10':
                    code10_idx = j+1
                if code == '20':
                    code20_idx = j+1
                j += 2
            if name in offsets and code10_idx is not None and code20_idx is not None:
                dx,dy = offsets[name]
                cur_x = float(lines[code10_idx])
                cur_y = float(lines[code20_idx])
                lines[code10_idx] = repr(cur_x + dx)
                lines[code20_idx] = repr(cur_y + dy)
                n_patched += 1
            i = j
        else:
            i += 1

    sep = '\r\n' if uses_crlf else '\n'
    out = sep.join(lines)
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(out)
    return n_patched


def patch_block_mtext(input_path, output_path, block_layer_offsets):
    """block_layer_offsets: dict blockname -> (dx,dy,set_of_layers) - adds dx,dy to the 10/20
    of every MTEXT inside that BLOCK definition whose layer (code 8) is in the given set.
    Operates on file already written by patch_dxf (chain calls), or on the raw input."""
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]

    start = None; end = None
    for i in range(0, len(lines)-1):
        if lines[i].strip()=='2' and lines[i+1].strip()=='BLOCKS':
            start = i
            break
    for i in range(start, len(lines)-1):
        if lines[i].strip()=='0' and lines[i+1].strip()=='ENDSEC':
            end = i
            break

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
        if lines[i].strip()=='0' and lines[i+1].strip()=='MTEXT' and cur_block in block_layer_offsets:
            dx, dy, layers = block_layer_offsets[cur_block]
            j = i+2
            layer = None
            code10_idx = None
            code20_idx = None
            while j < end-1 and lines[j].strip() != '0':
                code = lines[j].strip()
                if code == '8' and layer is None:
                    layer = lines[j+1].strip()
                if code == '10' and code10_idx is None:
                    code10_idx = j+1
                if code == '20' and code20_idx is None:
                    code20_idx = j+1
                j += 2
            if layer in layers and code10_idx is not None and code20_idx is not None:
                cur_x = float(lines[code10_idx])
                cur_y = float(lines[code20_idx])
                lines[code10_idx] = repr(cur_x + dx)
                lines[code20_idx] = repr(cur_y + dy)
                n_patched += 1
            i = j
            continue
        i += 1

    sep = '\r\n' if uses_crlf else '\n'
    out = sep.join(lines)
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(out)
    return n_patched

if __name__ == '__main__':
    from compute_beambar2 import compute_beambar_offsets
    res, debug = compute_beambar_offsets('/mnt/user-data/uploads/input.dxf')
    n = patch_dxf('/mnt/user-data/uploads/input.dxf', '/home/claude/work/my_output.dxf', res)
    print('patched', n, 'inserts')
