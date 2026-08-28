# -*- coding: utf-8 -*-
"""Παράγει το index.html (αυτόνομη εφαρμογή) από τα src/.

    python3 build/build_index.py

Ενσωματώνει το src/merge_dxf.js μέσα στο src/app.js και τα δύο μέσα στο
src/base_index.html, μαζί με τις προσθήκες UI (κανόνες, λίστα σταθμών,
κεφαλίδα). Το αποτέλεσμα γράφεται στο index.html της ρίζας.
"""
import pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC, OUT = ROOT / 'src', ROOT / 'index.html'

src  = (SRC / 'base_index.html').read_text(encoding='utf-8')
tail = (SRC / 'app.js').read_text(encoding='utf-8').replace(
       '/*__MERGE_MODULE__*/', (SRC / 'merge_dxf.js').read_text(encoding='utf-8'))

i = src.index('const $ = id => document.getElementById(id);'); j = src.index('</script>', i)
src = src[:i] + tail + '\n' + src[j:]

src = src.replace('<input type="file" id="file" accept=".dxf" hidden>',
                  '<input type="file" id="file" accept=".dxf" multiple hidden>')
src = src.replace('<h2>Ρίξε εδώ το DXF</h2>', '<h2>Ρίξε εδώ ΟΛΑ τα DXF</h2>')
src = src.replace('<p>ή πάτησε για επιλογή αρχείου · ξυλότυπος ASCII (blocks FLn_*)</p>',
                  '<p>ένα αρχείο ανά στάθμη · παράγεται ΕΝΑ ενοποιημένο DXF<br>ξυλότυποι &amp; λεπτομέρειες υποστυλωμάτων μαζί</p>')
src = src.replace('>Επιλογή αρχείου…</button>', '>Επιλογή αρχείων…</button>')

# ---- η πινακίδα προειδοποίησης: 3 αριθμημένοι κανόνες ----
old_note = '''      ⚠ <b>Προσοχή!</b> Τα DXF που εξάγεις από το FESPA πρέπει να είναι με Layers
      <b>«Με χρώματα»</b> — <u>όχι</u> με Layers <b>«Με πένες»</b>.'''
new_note = '''      <div class="wn-title">⚠ <b>Προσοχή!</b></div>
      <ol class="rules">
        <li>Η εξαγωγή αρχείων από το FESPA πρέπει να είναι με Layers
            <b>«Με χρώματα»</b> — <u>όχι</u> με Layers <b>«Με πένες»</b>.</li>
        <li>Τα αρχεία DXF <b>ξυλοτύπων</b> να είναι της μορφής
            <code><i class="box"></i><span class="ph">preffix</span>_or<i class="box"></i></code></li>
        <li>Οι <b>λεπτομέρειες υποστυλωμάτων</b> να είναι της μορφής
            <code><i class="box"></i><span class="ph">preffix</span>_Λεπτ_Υποστ_or<i class="box"></i></code></li>
        <li><b>Αφαίρεση σταυρονήματος</b> «οδηγός»/«κατασκευή» <span class="then">&amp; μετά export</span></li>
        <li>Στην εξαγωγή, καρτέλα <b>«Αρχιτεκτονικά ανά διαφανές»</b>: επιλογή <b>«Όχι»</b>
            <span class="then">&amp; μετά export</span></li>
        <li><b>Αφαίρεση snap</b> «έλξεις» <span class="then">&amp; μετά export</span></li>
      </ol>'''
assert old_note in src
src = src.replace(old_note, new_note)

# ---- λίστα σταθμών στην πινακίδα ----
old_row = '<div class="tb-row"><div class="tb-k">Αρχειο</div><div class="tb-v" id="tb-file">—</div></div>'
src = src.replace(old_row, old_row +
  '\n      <div class="tb-row"><div class="tb-k">Σταθμες</div><div class="tb-v">'
  '<div id="flist" class="flist"><div class="fl-empty">κανένα αρχείο</div></div>'
  '<button type="button" id="clear" class="mini">καθαρισμός λίστας</button></div></div>')
src = src.replace('<button>⬇ Κατέβασμα DXF</button>', '<button>⬇ Κατέβασμα ΕΝΟΠΟΙΗΜΕΝΟΥ DXF</button>')


# ---- κεφαλίδα: πινακίδα τίτλου σχεδίου ----
header = '''    <header class="titleblock" style="grid-column:1/-1;">
      <svg class="mark" viewBox="0 0 96 96" aria-hidden="true">
        <rect x="7" y="7" width="82" height="82" rx="2" class="sec"/>
        <rect x="17" y="17" width="62" height="62" rx="7" class="stir"/>
        <g class="bars">
          <circle cx="22" cy="22" r="4.6"/><circle cx="48" cy="20" r="4.6"/><circle cx="74" cy="22" r="4.6"/>
          <circle cx="20" cy="48" r="4.6"/><circle cx="76" cy="48" r="4.6"/>
          <circle cx="22" cy="74" r="4.6"/><circle cx="48" cy="76" r="4.6"/><circle cx="74" cy="74" r="4.6"/>
        </g>
        <path class="lead" d="M74 22 L92 6"/>
      </svg>
      <div class="tt">
        <h1>DXF <em>REBAR</em> AUTO-TIDY</h1>
        <div class="dim"><span>Αυτόματη τακτοποίηση κειμένων οπλισμού σε ξυλοτύπους</span></div>
      </div>
      <dl class="meta">
        <div><dt>Εκδοση</dt><dd>A3-v9</dd></div>
        <div><dt>Εκτελεση</dt><dd>Τοπικα</dd></div>
      </dl>
    </header>

'''
anchor = '    <div class="warn-note" style="grid-column:1/-1;">'
assert anchor in src
src = src.replace(anchor, header + anchor)
src = src.replace('<div class="tb-title">DXF <span class="k">R</span>EBAR AUTO-TIDY <span class="num">Π</span>ΙΝΑΚΙΔΑ</div>',
                  '<div class="tb-title"><span class="num">Π</span>ΙΝΑΚΙΔΑ <span class="k">Ε</span>ΡΓΟΥ</div>')

# ---- CSS ----
css = '''
  /* --- ΚΕΦΑΛΙΔΑ: πινακίδα τίτλου σχεδίου --- */
  .titleblock{ display:grid; grid-template-columns:auto 1fr auto; align-items:center; gap:18px;
               padding:14px 18px; margin-bottom:14px;
               border:1px solid var(--frame); border-left:4px solid var(--y);
               background:linear-gradient(180deg,#1b1e2233,#14161900); }
  .titleblock .mark{ width:56px; height:56px; flex:none; }
  .titleblock .mark .sec{ fill:none; stroke:var(--cy); stroke-width:2.6; }
  .titleblock .mark .stir{ fill:none; stroke:var(--frame); stroke-width:1.5; }
  .titleblock .mark .bars circle{ fill:var(--y); }
  .titleblock .mark .lead{ fill:none; stroke:var(--frame); stroke-width:1.2; }
  .titleblock h1{ margin:0; font:700 clamp(1.5rem,3.4vw,2.35rem) 'Saira Condensed';
                  letter-spacing:.2em; text-transform:uppercase; color:var(--frame); line-height:1; }
  .titleblock h1 em{ font-style:normal; color:var(--y); }
  /* γραμμή διάστασης: το υπότιτλο κάθεται μέσα στο κενό της, όπως σε ξυλότυπο */
  .titleblock .dim{ display:flex; align-items:center; gap:10px; margin-top:9px; }
  .titleblock .dim::before, .titleblock .dim::after{
        content:""; flex:1; height:9px; border-bottom:1px solid var(--hair); }
  .titleblock .dim::before{ border-left:1px solid var(--frame); }
  .titleblock .dim::after{ border-right:1px solid var(--frame); }
  .titleblock .dim span{ font:500 .74rem 'Saira Condensed'; letter-spacing:.15em;
                         text-transform:uppercase; color:var(--dim); white-space:nowrap; }
  .titleblock .meta{ display:flex; gap:0; margin:0; border:1px solid var(--hair); }
  .titleblock .meta > div{ padding:6px 12px; border-right:1px solid var(--hair); }
  .titleblock .meta > div:last-child{ border-right:0; }
  .titleblock .meta dt{ font:700 .6rem 'Saira Condensed'; letter-spacing:.16em;
                        text-transform:uppercase; color:var(--dim); }
  .titleblock .meta dd{ margin:2px 0 0; font:700 .84rem 'Saira Condensed';
                        letter-spacing:.1em; color:var(--cy); }
  @media (max-width:900px){
    .titleblock{ grid-template-columns:auto 1fr; gap:12px; }
    .titleblock .meta{ grid-column:1/-1; }
    .titleblock .dim span{ white-space:normal; }
  }

  /* --- λίστα σταθμών --- */
  .flist{ max-height:210px; overflow:auto; border:1px solid rgba(255,255,255,.10); border-radius:6px; }
  .fl-empty{ padding:6px 8px; opacity:.55; }
  .fl-row{ display:grid; grid-template-columns:1fr 1.25fr auto auto; gap:8px; align-items:center;
           padding:4px 8px; border-bottom:1px solid rgba(255,255,255,.06); font-size:12.5px; }
  .fl-row:last-child{ border-bottom:0; }
  .fl-n{ overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .fl-l{ opacity:.65; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .fl-s{ opacity:.85; white-space:nowrap; } .fl-s.ok{ color:var(--ok); } .fl-s.bad{ color:var(--bad); }
  .fl-d a{ text-decoration:none; font-size:15px; opacity:.85; }
  button.mini{ width:auto; margin-top:8px; padding:4px 10px; font-size:11.5px;
               background:transparent; border:1px solid rgba(255,255,255,.22); color:inherit; }
  /* --- κανόνες ονοματοδοσίας --- */
  .warn-note .wn-title{ display:block; font-size:1rem; letter-spacing:.06em; color:#ff8a4c;
                        padding-bottom:7px; margin-bottom:9px;
                        border-bottom:1px solid #b5451f66; }
  .warn-note .rules{ margin:0; padding:0 0 0 12px; list-style:none; counter-reset:r;
                     text-align:left; display:inline-block; vertical-align:top;
                     border-left:2px solid #b5451f88; }
  .warn-note .rules li{ counter-increment:r; margin:4px 0; padding-left:22px; position:relative; }
  .warn-note .rules li::before{ content:counter(r) ")"; position:absolute; left:0; color:#ff8a4c; font-weight:700; }
  .warn-note code{ font-family:ui-monospace,Consolas,monospace; font-size:.92em;
                   background:#00000038; padding:1px 6px; border-radius:4px; letter-spacing:.02em; }
  .warn-note code .ph{ color:#7fe3a0; font-weight:700; }
  .warn-note .then{ opacity:.7; font-style:italic; white-space:nowrap; }
  /* χωριστά κενά κουτάκια, ίδιο μέγεθος, πριν και μετά */
  .warn-note code .box{ display:inline-block; width:.9em; height:1.05em; vertical-align:-.2em;
                        margin:0 2px; border:1.5px solid #7fe3a0; border-radius:3px;
                        background:#7fe3a01a; }
  /* --- η υπογραφή να μην ακουμπά τον πάτο --- */
  .credit{ margin-bottom:34px; }
'''
k = src.index('</style>'); src = src[:k] + css + src[k:]

src = src.replace('Ο μέσος χρόνος παραγωγής τακτοποιημένου σχεδίου μίας στάθμης διαρκεί <b>μερικά λεπτά</b>.',
 'Ο μέσος χρόνος τακτοποίησης μίας στάθμης είναι <b>μερικά λεπτά</b> και οι στάθμες τρέχουν η μία μετά την άλλη. '
 'Τα αρχεία λεπτομερειών είναι ακαριαία (δεν περνούν από τακτοποίηση οπλισμού).')

OUT.write_text(src, encoding='utf-8')
i2 = src.index('<script>') + 8; j2 = src.index('</script>', i2)
(ROOT / 'build' / '_extracted.js').write_text(src[i2:j2], encoding='utf-8')
print('index', len(src), '| rules:', 'class="rules"' in src, '| credit margin:', '.credit{ margin-bottom:34px; }' in src)
