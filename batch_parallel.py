#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ΜΑΖΙΚΗ ΤΑΚΤΟΠΟΙΗΣΗ - ΠΑΡΑΛΛΗΛΑ ΣΕ ΟΛΟΥΣ ΤΟΥΣ ΠΥΡΗΝΕΣ

Γιατί παράλληλα ΑΝΑ ΑΡΧΕΙΟ: κάθε DXF είναι εντελώς ανεξάρτητο από τα άλλα.
Μέσα σε ΕΝΑ αρχείο ο παραλληλισμός δεν βοηθά (η βελτιστοποίηση είναι
σειριακή διαδικασία με κοινή κατάσταση), αλλά 5 αρχεία σε 8 πυρήνες
τρέχουν σχεδόν στον χρόνο του ΕΝΟΣ. Για 4 στάθμες + λεπτομέρειες
υποστυλωμάτων αυτό είναι η ουσιαστική επιτάχυνση.

ΧΡΗΣΗ
-----
    python batch_parallel.py ΦΑΚΕΛΟΣ
    python batch_parallel.py ΦΑΚΕΛΟΣ ΦΑΚΕΛΟΣ_ΕΞΟΔΟΥ
    python batch_parallel.py a.dxf b.dxf c.dxf -o ΕΞΟΔΟΣ

ΕΠΙΛΟΓΕΣ
--------
    -o ΦΑΚΕΛΟΣ      φάκελος εξόδου (default: ίδιος με την είσοδο)
    -j N            πλήθος ταυτόχρονων εργασιών (default: πυρήνες - 1)
    -t ΔΕΥΤ         χρονικό όριο βελτιστοποίησης ΑΝΑ αρχείο
                    (default: χωρίς όριο. π.χ. -t 300)
    --no-audit      παράλειψη του ελέγχου (πιο γρήγορο)

ΠΑΡΑΔΕΙΓΜΑ (Windows)
    python batch_parallel.py C:\\ktirio -j 6 -t 600

ΤΙ ΠΑΡΑΓΕΙ
    <όνομα>_tidy.dxf     το τακτοποιημένο σχέδιο
    <όνομα>_offsets.pkl  οι μετατοπίσεις
    <όνομα>_audit.txt    η πλήρης αναφορά ελέγχου
    _SUMMARY.txt         συγκεντρωτικός πίνακας με AUDIT_TOTAL ανά αρχείο
"""
import sys, os, glob, time, subprocess, re

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))


def _audit_total(text):
    """Τραβά το AUDIT_TOTAL από την έξοδο του ελέγχου."""
    old = new = None
    for line in text.splitlines():
        m = re.match(r'AUDIT_TOTAL_OLD_SCOPE\s+(\d+)', line)
        if m:
            old = int(m.group(1)); continue
        m = re.match(r'AUDIT_TOTAL\s+(\d+)', line)
        if m:
            new = int(m.group(1))
    return new, old


def process_one(job):
    """Τρέχει ΕΝΑ αρχείο. Εκτελείται σε ξεχωριστή διεργασία."""
    src, out_dir, budget, do_audit = job
    base = os.path.splitext(os.path.basename(src))[0]
    out_dxf = os.path.join(out_dir, base + '_tidy.dxf')
    out_pkl = os.path.join(out_dir, base + '_offsets.pkl')
    out_aud = os.path.join(out_dir, base + '_audit.txt')

    env = dict(os.environ)
    env['PYTHONHASHSEED'] = '0'
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    if budget:
        env['AUTOTIDY_OPT_SECONDS'] = str(budget)

    t0 = time.time()
    try:
        r = subprocess.run(
            [sys.executable, '-u', os.path.join(HERE, 'run_any.py'),
             src, out_dxf, out_pkl],
            capture_output=True, text=True, env=env, cwd=HERE)
    except Exception as exc:                       # pragma: no cover
        return (base, 'ΣΦΑΛΜΑ', None, None, time.time()-t0, str(exc)[:200])

    if r.returncode != 0 or not os.path.exists(out_dxf):
        tail = (r.stdout or '')[-300:] + (r.stderr or '')[-300:]
        return (base, 'ΑΠΕΤΥΧΕ', None, None, time.time()-t0, tail.strip()[:300])

    new = old = None
    if do_audit:
        try:
            a = subprocess.run(
                [sys.executable, '-u', os.path.join(HERE, 'audit_output.py'),
                 out_dxf, src],
                capture_output=True, text=True, env=env, cwd=HERE)
            with open(out_aud, 'w', encoding='utf-8') as fh:
                fh.write(a.stdout)
            new, old = _audit_total(a.stdout)
        except Exception:
            pass

    return (base, 'ΟΚ', new, old, time.time()-t0, '')


def main():
    args = [a for a in sys.argv[1:]]
    if not args or args[0] in ('-h', '--help'):
        print(__doc__); sys.exit(0)

    out_dir = None
    workers = None
    budget = None
    do_audit = True
    inputs = []

    i = 0
    while i < len(args):
        a = args[i]
        if a == '-o' and i+1 < len(args):
            out_dir = args[i+1]; i += 2
        elif a == '-j' and i+1 < len(args):
            workers = int(args[i+1]); i += 2
        elif a == '-t' and i+1 < len(args):
            budget = float(args[i+1]); i += 2
        elif a == '--no-audit':
            do_audit = False; i += 1
        else:
            inputs.append(a); i += 1

    files = []
    for item in inputs:
        if os.path.isdir(item):
            files += sorted(glob.glob(os.path.join(item, '*.dxf')))
            if out_dir is None:
                out_dir = item
        else:
            files.append(item)
            if out_dir is None:
                out_dir = os.path.dirname(os.path.abspath(item))

    # μη ξαναπεράσεις ήδη τακτοποιημένα
    files = [f for f in files if not f.lower().endswith(('_tidy.dxf', '_tidied.dxf'))]
    if not files:
        print('Δεν βρέθηκε κανένα .dxf.'); sys.exit(1)

    os.makedirs(out_dir, exist_ok=True)

    try:
        import multiprocessing as mp
        ncpu = mp.cpu_count()
    except Exception:
        mp, ncpu = None, 1
    if workers is None:
        workers = max(1, min(len(files), ncpu - 1 if ncpu > 1 else 1))

    print(f'Αρχεία: {len(files)}   Πυρήνες: {ncpu}   Ταυτόχρονα: {workers}'
          + (f'   Όριο/αρχείο: {budget:.0f}s' if budget else '   Χωρίς χρονικό όριο'))
    for f in files:
        print('   -', os.path.basename(f))
    print()

    jobs = [(f, out_dir, budget, do_audit) for f in files]
    t0 = time.time()
    results = []

    if mp is not None and workers > 1:
        with mp.Pool(processes=workers) as pool:
            for res in pool.imap_unordered(process_one, jobs):
                results.append(res)
                nm, st, new, old, el, err = res
                extra = f'AUDIT {old}' if old is not None else st
                print(f'  [{len(results)}/{len(files)}] {nm}: {extra}  ({el:.0f}s)')
                if err:
                    print(f'        {err}')
    else:
        for job in jobs:
            res = process_one(job)
            results.append(res)
            nm, st, new, old, el, err = res
            print(f'  [{len(results)}/{len(files)}] {nm}: '
                  f'{old if old is not None else st}  ({el:.0f}s)')

    total = time.time() - t0
    results.sort(key=lambda r: r[0])

    lines = []
    lines.append('=' * 66)
    lines.append(f'{"ΑΡΧΕΙΟ":<30}{"ΚΑΤΑΣΤ.":<10}{"AUDIT":>8}{"ΠΛΗΡΕΣ":>9}{"ΧΡΟΝΟΣ":>9}')
    lines.append('=' * 66)
    okc = 0
    for nm, st, new, old, el, err in results:
        if st == 'ΟΚ':
            okc += 1
        lines.append(f'{nm[:29]:<30}{st:<10}'
                     f'{("-" if old is None else old):>8}'
                     f'{("-" if new is None else new):>9}'
                     f'{el:>8.0f}s')
    lines.append('=' * 66)
    lines.append(f'Επιτυχή: {okc}/{len(results)}   Συνολικός χρόνος: {total:.0f}s')
    if workers > 1:
        seq = sum(r[4] for r in results)
        lines.append(f'Σειριακά θα έκανε: {seq:.0f}s  ->  κέρδος {seq/max(total,1):.1f}x')
    # ---------------- ΕΝΟΠΟΙΗΣΗ ΣΕ ΕΝΑ DXF ----------------
    # Το ζητούμενο δεν είναι μόνο χωριστά _tidy.dxf αλλά ΕΝΑ ενοποιημένο
    # σχέδιο με όλες τις στάθμες (και τις λεπτομέρειες υποστυλωμάτων)
    # τοποθετημένες τη μία κάτω από την άλλη - ό,τι κάνει ήδη ο browser.
    # Η ενοποίηση ζει στο src/merge_dxf.js, οπότε χρειάζεται Node.js.
    merged_path = os.path.join(out_dir, '_ENOPOIIMENO.dxf')
    tidied = [os.path.join(out_dir, os.path.splitext(os.path.basename(f))[0] + '_tidy.dxf')
              for f in files]
    tidied = [p for p in tidied if os.path.exists(p)]
    merge_note = ''
    if len(tidied) >= 1:
        cli = os.path.abspath(os.path.join(HERE, '..', 'src', 'merge_cli.js'))
        if not os.path.exists(cli):
            merge_note = 'ΕΝΟΠΟΙΗΣΗ: δεν βρέθηκε src/merge_cli.js'
        else:
            try:
                m = subprocess.run(['node', cli, merged_path] + tidied,
                                   capture_output=True, text=True)
                if m.returncode == 0 and os.path.exists(merged_path):
                    merge_note = f'ΕΝΟΠΟΙΗΜΕΝΟ: {merged_path}'
                    print('\n' + (m.stdout or '').strip())
                else:
                    merge_note = ('ΕΝΟΠΟΙΗΣΗ ΑΠΕΤΥΧΕ: '
                                  + ((m.stderr or m.stdout or '').strip()[:200]))
            except FileNotFoundError:
                merge_note = ('ΕΝΟΠΟΙΗΣΗ: δεν βρέθηκε το Node.js. '
                              'Εγκατέστησέ το από nodejs.org και ξανατρέξε, '
                              'ή χρησιμοποίησε το index.html στον browser.')
            except Exception as exc:
                merge_note = f'ΕΝΟΠΟΙΗΣΗ ΑΠΕΤΥΧΕ: {str(exc)[:200]}'
    if merge_note:
        lines.append(merge_note)

    out = '\n'.join(lines)
    print('\n' + out)
    with open(os.path.join(out_dir, '_SUMMARY.txt'), 'w', encoding='utf-8') as fh:
        fh.write(out + '\n')
    print(f'\nΑποτελέσματα στον φάκελο: {out_dir}')


if __name__ == '__main__':
    main()
