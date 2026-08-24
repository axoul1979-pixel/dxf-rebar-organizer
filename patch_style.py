def patch_layer_colors(input_path, output_path, color_map=None):
    """Set the default color (group code 62) for a set of named layers, directly in the
    LAYER table. color_map defaults to the standard palette used throughout this project
    (chosen for visibility on a black AutoCAD background):
        slab_poly              -> 4 (cyan)
        slab_center             -> 2 (yellow)
        beam_prefix_name_beton  -> 2 (yellow)
        slab_name                -> 2 (yellow)
        beambar_name             -> 2 (yellow)
        slabbar_name             -> 2 (yellow)
        slabbar_line             -> 6 (magenta)
        slab_prefix_name         -> 5 (blue)
    Missing layers in a given file are silently skipped (not every file has every layer)."""
    if color_map is None:
        color_map = {
            'slab_poly': 4,
            'slab_center': 2,
            'beam_prefix_name_beton': 2,
            'slab_name': 2,
            'beambar_name': 2,
            'slabbar_name': 2,
            'slabbar_line': 6,
            'slab_prefix_name': 5,
        }
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]
    patched = {}
    i = 0
    while i < len(lines) - 1:
        if lines[i].strip() == '2' and lines[i+1].strip() in color_map:
            layer_name = lines[i+1].strip()
            j = i + 2
            while j < len(lines) - 1 and lines[j].strip() != '0':
                if lines[j].strip() == '62':
                    lines[j+1] = str(color_map[layer_name])
                    patched[layer_name] = True
                    break
                j += 2
        i += 1
    sep = '\r\n' if uses_crlf else '\n'
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(sep.join(lines))
    return patched

def patch_hatch_scale_by_layer(input_path, output_path, layer_name, new_scale, orig_scale=0.02):
    """Rescale HATCH entities ONLY on a specific layer (unlike patch_style's hatch_scale
    which applies globally to every HATCH). Rescales both the pattern-scale (group 41)
    and the 45/46 line-offset vectors proportionally, same logic as patch_style."""
    with open(input_path, 'r', encoding='latin-1', newline='') as f:
        raw = f.read()
    uses_crlf = '\r\n' in raw
    lines = raw.split('\n')
    lines = [l[:-1] if l.endswith('\r') else l for l in lines]

    scale_ratio = new_scale / orig_scale
    n_scale = 0
    i = 0
    while i < len(lines) - 1:
        code = lines[i].strip()
        if code == '0' and lines[i+1].strip() == 'HATCH':
            j = i + 2
            layer = None
            while j < len(lines) - 1 and lines[j].strip() != '0':
                if lines[j].strip() == '8' and layer is None:
                    layer = lines[j+1].strip()
                j += 2
            if layer == layer_name:
                j = i + 2
                while j < len(lines) - 1 and lines[j].strip() != '0':
                    c = lines[j].strip()
                    if c == '41':
                        lines[j+1] = repr(float(new_scale))
                        n_scale += 1
                    elif c in ('45','46'):
                        cur = float(lines[j+1])
                        lines[j+1] = repr(cur * scale_ratio)
                    j += 2
            i = j
            continue
        i += 1

    sep = '\r\n' if uses_crlf else '\n'
    with open(output_path, 'w', encoding='latin-1', newline='') as f:
        f.write(sep.join(lines))
    return n_scale

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
