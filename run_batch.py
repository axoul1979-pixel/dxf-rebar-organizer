import sys
sys.dont_write_bytecode = True
"""Μαζική τακτοποίηση: ΟΛΑ τα DXF ενός φακέλου με ΜΙΑ εντολή.

Χρήση:
    python run_batch.py ΦΑΚΕΛΟΣ_ΕΙΣΟΔΟΥ [ΦΑΚΕΛΟΣ_ΕΞΟΔΟΥ]

- Επεξεργάζεται κάθε *.dxf του φακέλου εισόδου (παραλείπει όσα τελειώνουν
  σε _tidied.dxf ώστε να μην ξαναπεράσει έτοιμα αποτελέσματα).
- Για κάθε αρχείο παράγει <όνομα>_tidied.dxf + <όνομα>_offsets.pkl και
  τρέχει τον ΑΝΕΞΑΡΤΗΤΟ έλεγχο (audit) πάνω στο αποτέλεσμα.
- Στο τέλος τυπώνει συγκεντρωτικό πίνακα: αρχείο, χρόνος, AUDIT_TOTAL.
  Παραδοτέο θεωρείται ΜΟΝΟ ό,τι έχει AUDIT_TOTAL 0 - τα υπόλοιπα
  αναφέρονται ρητά για έλεγχο.

Παράδειγμα (Windows):
    python run_batch.py C:\\Users\\TASOS\\Desktop\\ktirio_A
"""
import os, glob, time, subprocess

os.environ.setdefault('PYTHONHASHSEED', '0')

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    src_dir = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else src_dir
    os.makedirs(out_dir, exist_ok=True)
    here = os.path.dirname(os.path.abspath(__file__))

    files = sorted(f for f in glob.glob(os.path.join(src_dir, '*.dxf'))
                   if not f.lower().endswith('_tidied.dxf'))
    if not files:
        print('Δεν βρέθηκαν .dxf στον φάκελο:', src_dir)
        sys.exit(1)
    print(f'Βρέθηκαν {len(files)} αρχεία. Ξεκινάω…\n')

    results = []
    for i, src in enumerate(files, 1):
        base = os.path.splitext(os.path.basename(src))[0]
        dst = os.path.join(out_dir, base + '_tidied.dxf')
        pkl = os.path.join(out_dir, base + '_offsets.pkl')
        print(f'[{i}/{len(files)}] {base}.dxf …', flush=True)
        t0 = time.time()
        r1 = subprocess.run([sys.executable, os.path.join(here, 'run_any.py'), src, dst, pkl],
                            capture_output=True, text=True)
        dt = time.time() - t0
        if r1.returncode != 0 or not os.path.exists(dst):
            print(f'   ΣΦΑΛΜΑ ΕΚΤΕΛΕΣΗΣ ({dt:.0f}s):')
            print('   ' + (r1.stderr or r1.stdout).strip().splitlines()[-1] if (r1.stderr or r1.stdout) else '')
            results.append((base, dt, 'ΣΦΑΛΜΑ'))
            continue
        r2 = subprocess.run([sys.executable, os.path.join(here, 'audit_output.py'), dst, src],
                            capture_output=True, text=True)
        total = '?'
        for line in (r2.stdout or '').splitlines():
            if line.startswith('AUDIT_TOTAL'):
                total = line.split()[-1]
        print(f'   ΟΚ σε {dt:.0f}s - AUDIT_TOTAL {total}')
        results.append((base, dt, total))

    print('\n===== ΣΥΓΚΕΝΤΡΩΤΙΚΑ =====')
    total_t = sum(dt for _, dt, _ in results)
    clean = sum(1 for _, _, t in results if t == '0')
    for base, dt, t in results:
        mark = '✓' if t == '0' else ('✗ ' + str(t))
        print(f'  {base:32s} {dt:6.0f}s   {mark}')
    print(f'\nΣύνολο: {len(results)} αρχεία σε {total_t/60:.1f} λεπτά - '
          f'{clean} καθαρά (AUDIT_TOTAL 0), {len(results)-clean} για έλεγχο.')

if __name__ == '__main__':
    main()
