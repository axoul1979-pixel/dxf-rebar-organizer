import sys
sys.dont_write_bytecode = True
import re, time
from pipeline_v11 import process_all
from beambar_engine import get_inserts
from patcher import patch_dxf, patch_block_mtext
from patch_slab_marker import patch_slab_marker_geometry
from patch_style import patch_layer_colors, patch_style
from analyze import get_all_blocks, entities_from_pairs, to_dict

t0=time.time()
input_path = 'karaisk05_OR0.dxf'
insert_final, text_local_final = process_all(input_path, is_training_file=False)

ins_native = dict((n,(x,y)) for n,x,y in get_inserts(input_path))
fixed = dict(insert_final)
for name in list(fixed.keys()):
    if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', name):
        nx, ny = ins_native.get(name, (0.0,0.0))
        dx, dy = fixed[name]
        fixed[name] = (dx - nx, dy - ny)

patch_dxf(input_path, 'stage1.dxf', fixed)
blocks = get_all_blocks(input_path)
blo = {}
for name,(dx,dy) in text_local_final.items():
    ents = entities_from_pairs(blocks[name])
    layers = {to_dict(e).get(8,[''])[0] for e in ents if e[0][1]=='MTEXT'}
    blo[name] = (dx,dy,layers)
patch_block_mtext('stage1.dxf','stage2.dxf', blo)
md = {n:(dx,dy) for n,(dx,dy) in text_local_final.items() if re.match(r'FL-?\d+_SLAB\d+$',n)}
patch_slab_marker_geometry('stage2.dxf','stage3.dxf', md)
patch_layer_colors('stage3.dxf','stage4.dxf')
patch_style('stage4.dxf','karaisk05_OR0_tidied.dxf', hatch_scale=0.02, orig_hatch_scale=0.1)
print('DONE in %.1fs'%(time.time()-t0))
