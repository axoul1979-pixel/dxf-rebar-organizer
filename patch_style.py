def patch_style(input_path, output_path, slab_poly_color=4, hatch_scale=0.02, orig_hatch_scale=0.1,
                 slab_center_color=None):
    """slab_poly_color: ACI color index (4=cyan). hatch_scale: new pattern scale - also
    rescales the 45/46 line-offset vectors proportionally (these encode the ACTUAL line
    spacing in world units; group 41 alone is a multiplier that doesn't change rendering
    unless the underlying offsets are rescaled too). slab_center_color: if set, also patch
    that layer's color the same way as slab_poly."""
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]

    scale_ratio = hatch_scale / orig_hatch_scale
    n_color = 0
    n_scale = 0
    n_offset = 0
    i = 0
    while i < len(lines) - 1:
        code = lines[i].strip()
        if code == '0' and lines[i+1].strip() == 'HATCH':
            j = i + 2
            while j < len(lines) - 1 and lines[j].strip() != '0':
                c = lines[j].strip()
                if c == '41':
                    lines[j+1] = repr(float(hatch_scale))
                    n_scale += 1
                elif c in ('45','46'):
                    cur = float(lines[j+1])
                    lines[j+1] = repr(cur * scale_ratio)
                    n_offset += 1
                j += 2
            i = j
            continue
        i += 1

    sep = '\r\n' if uses_crlf else '\n'
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(sep.join(lines))
    return n_color, n_scale, n_offset
