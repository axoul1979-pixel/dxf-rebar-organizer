/* ============================================================================
   Τρέχει ΟΛΗ την επεξεργασία σε έναν πραγματικό φάκελο έργου και ενοποιεί.

     node tests/run_project.js <φάκελος> [<αρχείο εξόδου.dxf>]

   ΠΡΟΣΟΧΗ: εδώ ΔΕΝ τρέχει η τακτοποίηση οπλισμού (είναι Python/Pyodide και
   θέλει λεπτά ανά στάθμη). Ελέγχεται ό,τι κάνει το JS: διάταξη διατομών,
   περιμετρικές διαστάσεις, αντίγραφο σκελετού και ενοποίηση σταθμών.
   ========================================================================== */
const fs = require('fs'), path = require('path');
const M = require('../src/merge_dxf.js');
const strict = require('./strict_check.js');
const clearance = require('./check_clearance.js');

const dir = process.argv[2];
if (!dir) { console.error('Χρήση: node tests/run_project.js <φάκελος με τα DXF> [έξοδος.dxf]'); process.exit(2); }
const outFile = process.argv[3] || path.join(dir, 'ΕΝΟΠΟΙΗΜΕΝΟ.dxf');

const files = fs.readdirSync(dir).filter(n => /\.dxf$/i.test(n))
  .map(n => ({ n: n, lv: M.parseLevelName(n) })).filter(x => x.lv);
if (!files.length) { console.error('Κανένα αρχείο …_orN.dxf στον φάκελο.'); process.exit(1); }
files.sort((a, b) => (a.lv.level - b.lv.level) || (a.lv.kind === 'plan' ? -1 : 1));

const items = [], srcs = [];
console.log('=== ΞΥΛΟΤΥΠΟΙ ===');
files.filter(x => x.lv.kind === 'plan').forEach(x => {
  const t = fs.readFileSync(path.join(dir, x.n), 'latin1'); srcs.push(t);
  const d = M.dimensionPlan(t);
  const c = M.copyStructure(d.added ? d.text : t);
  console.log('  ' + x.n.padEnd(30) +
    (d.added ? 'διαστάσεις +' + d.added + ' (' + d.kind + ' ' + d.columns + ')' : '— χωρίς διαστάσεις') +
    ' | αντίγραφο ' + (c.copied - c.lines) + ' blocks + ' + c.lines + ' γραμμές, κενό ' + c.gap + 'μ' +
    (d.added ? ' | ασφάλεια ' + clearance(c.text, true).toFixed(3) : ''));
  items.push({ name: x.n, text: c.text });
});
console.log('=== ΛΕΠΤΟΜΕΡΕΙΕΣ ΥΠΟΣΤΥΛΩΜΑΤΩΝ ===');
files.filter(x => x.lv.kind === 'detail').forEach(x => {
  const t = fs.readFileSync(path.join(dir, x.n), 'latin1'); srcs.push(t);
  const r = M.layoutSections(t);
  console.log('  ' + x.n.padEnd(30) + r.groups.length + ' διατομές | ' + r.rows + ' σειρές | ' +
    r.width.toFixed(2) + ' × ' + r.height.toFixed(2) + ' | scale ' + r.baseScale + '→' + r.finalScale);
  items.push({ name: x.n, text: r.text });
});

const t0 = Date.now();
const merged = M.mergeLevels(items);
console.log('\n=== ΕΝΟΠΟΙΗΣΗ (' + ((Date.now() - t0) / 1000).toFixed(1) + 's) ===');
merged.report.forEach(r => console.log('  or' + String(r.level).padStart(2) + ' ' +
  (r.kind === 'detail' ? 'ΛΕΠΤ' : 'ΞΥΛΟ') + ' pick(' + r.pick[0].toFixed(2) + ', ' + r.pick[1].toFixed(2) +
  ') → (' + r.target[0] + ', ' + r.target[1] + ')  ' + r.title));
merged.warnings.forEach(w => console.log('  ⚠ ' + w));
console.log('  οντότητες: ' + JSON.stringify(merged.counts));
fs.writeFileSync(outFile, merged.text, 'latin1');
console.log('  → ' + outFile);

console.log('\n=== ΑΥΣΤΗΡΟΣ ΕΛΕΓΧΟΣ ΔΟΜΗΣ ===');
const bad = strict(merged.text, srcs);
console.log(bad <= 1 ? '  ΟΚ (το 1 πρόβλημα, αν εμφανιστεί, είναι προϋπάρχουσα κρεμασμένη αναφορά των πηγαίων)'
                     : '  ΠΡΟΣΟΧΗ: ' + bad + ' προβλήματα');
process.exit(bad <= 1 ? 0 : 1);
