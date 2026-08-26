import sys
sys.dont_write_bytecode = True
import re, sys, time
from pipeline_v11 import process_all
from beambar_engine import get_inserts
from patcher import patch_dxf, patch_block_mtext
from patch_slab_marker import patch_slab_marker_geometry, swap_marker_name_height, collapse_bar_geometry
import pipeline_v11 as _P
from patch_style import patch_layer_colors, patch_style
from analyze import get_all_blocks, entities_from_pairs, to_dict
import pickle

input_path = sys.argv[1]; out_path = sys.argv[2]
t0=time.time()
insert_final, text_local_final = process_all(input_path, is_training_file=False)
pickle.dump((insert_final, text_local_final), open(sys.argv[3],'wb'))

ins_native = dict((n,(x,y)) for n,x,y in get_inserts(input_path))
fixed = dict(insert_final)
for name in list(fixed.keys()):
    if re.match(r'FL-?\d+_COLUMN_TEXT\d+$', name):
        nx, ny = ins_native.get(name, (0.0,0.0))
        dx, dy = fixed[name]
        fixed[name] = (dx - nx, dy - ny)

patch_dxf(input_path, '_s1.dxf', fixed)
blocks = get_all_blocks(input_path)
blo = {}
for name,(dx,dy) in text_local_final.items():
    ents = entities_from_pairs(blocks[name])
    layers = {to_dict(e).get(8,[''])[0] for e in ents if e[0][1]=='MTEXT'}
    blo[name] = (dx,dy,layers)
patch_block_mtext('_s1.dxf','_s2.dxf', blo)
md = {n:(dx,dy) for n,(dx,dy) in text_local_final.items() if re.match(r'FL-?\d+_SLAB\d+$',n)}
patch_slab_marker_geometry('_s2.dxf','_s3.dxf', md)
nsw = swap_marker_name_height('_s3.dxf','_s3.dxf')
print(f'MARKER FORMAT: {nsw} δείκτες σε μορφή «Π πάνω / h= κάτω»')
patch_layer_colors('_s3.dxf','_s4.dxf')
patch_style('_s4.dxf', out_path, hatch_scale=0.02, orig_hatch_scale=0.1)
if _P.P2_REPLACE:
    _np2 = collapse_bar_geometry(out_path, set(_P.P2_REPLACE), targets=dict(_P.P2_REPLACE))
    print('P2 REPLACE (πρόβολοι):', sorted(_P.P2_REPLACE), f'({_np2} γραμμές)')
if _P.COLLAPSE_BARS:
    _nc = collapse_bar_geometry(out_path, set(_P.COLLAPSE_BARS))
    print('HAIRPIN COLLAPSE: %s (%d γραμμές στην ευθεία)' % (sorted(_P.COLLAPSE_BARS), _nc))
else:
    print('HAIRPIN COLLAPSE: κανένα')
print('DONE %.1fs'%(time.time()-t0))
