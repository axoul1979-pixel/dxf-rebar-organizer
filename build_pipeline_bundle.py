# -*- coding: utf-8 -*-
"""Παράγει ΞΑΝΑ τον ενσωματωμένο κώδικα Python από τα pipeline/*.py.

    python3 build/build_pipeline_bundle.py

Γράφει:
  * pipeline/autotidy_all_in_one.py   (μονοαρχειακή έκδοση)
  * pipeline/autotidy_single_file.py  (πανομοιότυπο αντίγραφο)
  * src/base_index.html               (ενημερώνει το const PIPELINE_B64)

ΓΙΑΤΙ ΥΠΑΡΧΕΙ: το PIPELINE_B64 ήταν καρφωμένο μέσα στο base_index.html και ΔΕΝ
ξαναχτιζόταν από το build_index.py. Έτσι ο browser μπορούσε να τρέχει ΠΑΛΙΟ
κώδικα ενώ τα .py του πακέτου ήταν ενημερωμένα - και τα αποτελέσματα της
εφαρμογής απέκλιναν από αυτά της γραμμής εντολών. Πλέον το build_index.py
καλεί ΠΑΝΤΑ αυτό το script πρώτο, οπότε η απόκλιση δεν μπορεί να ξανασυμβεί.
"""
import base64
import pathlib
import re
import zlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
PIPE = ROOT / 'pipeline'
SRC = ROOT / 'src'

# ΣΕΙΡΑ ΦΟΡΤΩΣΗΣ - ΑΠΑΡΑΒΑΤΗ: το slab_region πρέπει να προηγείται του parse_dxf.
ORDER = ['slab_region', 'parse_dxf', 'analyze', 'engine', 'beambar_engine',
         'hatch_engine', 'compute_column_text', 'compute_beamtext_slabmarker',
         'compute_beambar3', 'compute_slabbar3', 'perimeter', 'patch_style',
         'patch_slab_marker', 'patcher', 'pipeline_v11', 'global_repair']
SCRIPTS = ['run_any', 'audit_output', 'verify_all']

HEADER = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DXF Rebar Auto-Tidy - ΜΟΝΟΑΡΧΕΙΑΚΗ έκδοση (όλα τα modules ενσωματωμένα).
Για περιβάλλοντα όπου δεν βολεύει φάκελος με πολλά αρχεία (Pyodide/browser,
online runners). Πανομοιότυπος κώδικας με το κανονικό πακέτο.

ΠΑΡΑΓΕΤΑΙ ΑΥΤΟΜΑΤΑ από το build/build_pipeline_bundle.py - ΜΗΝ το πειράζεις
με το χέρι.

Χρήση:
    python autotidy_all_in_one.py tidy   ΕΙΣΟΔΟΣ.dxf ΕΞΟΔΟΣ.dxf [offsets.pkl]
    python autotidy_all_in_one.py audit  ΕΞΟΔΟΣ.dxf ΕΙΣΟΔΟΣ.dxf
    python autotidy_all_in_one.py verify ΕΙΣΟΔΟΣ.dxf offsets.pkl
"""
import sys, os, types, base64, zlib, ast as _ast
sys.dont_write_bytecode = True

_SRC = _ast.literal_eval(zlib.decompress(base64.b64decode(
'''

TAIL = '''
_ORDER = %r

_LOADED = set()

def _load_modules():
    # Εκτέλεση σε "σταθερό σημείο": όποιο module αποτύχει επειδή δεν έχει
    # εκτελεστεί ακόμη η εξάρτησή του, ξαναδοκιμάζεται στον επόμενο γύρο.
    pending = [m for m in _ORDER if m not in _LOADED]
    while pending:
        progressed = False
        errors = {}
        for _m in list(pending):
            mod = types.ModuleType(_m)
            mod.__file__ = _m + ".py"
            sys.modules[_m] = mod
            try:
                exec(compile(_SRC[_m], _m + ".py", "exec"), mod.__dict__)
            except (ImportError, AttributeError) as e:
                del sys.modules[_m]
                errors[_m] = e
                continue
            _LOADED.add(_m)
            pending.remove(_m)
            progressed = True
        if not progressed:
            raise ImportError("Αδιέξοδο εξαρτήσεων: %%r" %% errors)

def _run_script(name, argv):
    _load_modules()
    old = sys.argv
    sys.argv = [name] + list(argv)
    try:
        g = {"__name__": "__main__", "__file__": name + ".py"}
        exec(compile(_SRC["@" + name], name + ".py", "exec"), g)
    finally:
        sys.argv = old

def tidy(src, dst, pkl="offsets.pkl"):
    """Τακτοποίηση: παράγει το DXF εξόδου + pickle μετατοπίσεων."""
    _run_script("run_any", [src, dst, pkl])

def audit(out_dxf, in_dxf):
    """ΑΝΕΞΑΡΤΗΤΟΣ έλεγχος στο τελικό DXF - απαιτείται AUDIT_TOTAL 0."""
    _run_script("audit_output", [out_dxf, in_dxf])

def verify(in_dxf, pkl):
    """Επαλήθευση απαράβατων κανόνων (πλευρές/πλάκες/κατά-μήκος)."""
    _run_script("verify_all", [in_dxf, pkl])

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("tidy", "audit", "verify"):
        print(__doc__)
        sys.exit(1)
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == "tidy":
        tidy(*args)
    elif cmd == "audit":
        audit(*args)
    else:
        verify(*args)
''' % (ORDER,)


def _wrap(b64, indent, width=110):
    out = []
    for i in range(0, len(b64), width):
        out.append('%s"%s"' % (indent, b64[i:i + width]))
    return '\n'.join(out)


def build_monolith():
    src = {}
    for m in ORDER:
        src[m] = (PIPE / (m + '.py')).read_text(encoding='utf-8')
    for s in SCRIPTS:
        src['@' + s] = (PIPE / (s + '.py')).read_text(encoding='utf-8')
    blob = base64.b64encode(zlib.compress(repr(src).encode('utf-8'), 9)).decode('ascii')
    text = HEADER + _wrap(blob, '    ') + '\n)).decode("utf-8"))\n' + TAIL
    (PIPE / 'autotidy_all_in_one.py').write_text(text, encoding='utf-8')
    (PIPE / 'autotidy_single_file.py').write_text(text, encoding='utf-8')
    return text


def embed_in_base_index(mono_text):
    html = (SRC / 'base_index.html').read_text(encoding='utf-8')
    b64 = base64.b64encode(mono_text.encode('utf-8')).decode('ascii')
    body = '\n'.join('    "%s" +' % b64[i:i + 110] for i in range(0, len(b64), 110))
    body = body.rstrip(' +')
    new = 'const PIPELINE_B64 =\n' + body + ';'
    m = re.search(r'const PIPELINE_B64 =.*?;\n', html, re.S)
    if not m:
        raise SystemExit('Δεν βρέθηκε το const PIPELINE_B64 στο src/base_index.html')
    html = html[:m.start()] + new + '\n' + html[m.end():]
    (SRC / 'base_index.html').write_text(html, encoding='utf-8')
    return len(b64)


if __name__ == '__main__':
    mono = build_monolith()
    n = embed_in_base_index(mono)
    print('bundle: %d modules + %d scripts | monolith %d bytes | b64 %d chars'
          % (len(ORDER), len(SCRIPTS), len(mono), n))
