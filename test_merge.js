const FIX=require('path').join(__dirname,'fixtures');
const fs = require('fs');
const M = require('../src/merge_dxf.js');

const dir = FIX+'/r12_mtext';
const names = fs.readdirSync(dir).filter(n => n.endsWith('.dxf')).sort();
const items = names.map(n => ({ name: n, text: fs.readFileSync(dir + '/' + n, 'latin1') }));

const res = M.mergeLevels(items);
fs.writeFileSync(FIX+'/r12_mtext/MERGED.dxf', res.text, 'latin1');

console.log('--- REPORT ---');
res.report.forEach(r => console.log(
  r.file.padEnd(28), 'or' + String(r.level).padStart(2),
  '| pick(' + r.pick[0].toFixed(2) + ',' + r.pick[1].toFixed(2) + ')',
  '-> (' + r.target[0].toFixed(1) + ',' + r.target[1].toFixed(1) + ')',
  '| d(' + r.delta[0].toFixed(2) + ',' + r.delta[1].toFixed(2) + ')',
  '|', r.title));
console.log('--- WARNINGS ---'); res.warnings.forEach(w => console.log(' ! ' + w));
console.log('--- COUNTS ---', JSON.stringify(res.counts), 'ext', JSON.stringify(res.extents));

/* ---------- ΕΠΑΛΗΘΕΥΣΗ ---------- */
const merged = M._analyze(res.text);
let fail = 0;
const chk = (cond, msg) => { console.log((cond ? '  ok   ' : '  FAIL ') + msg); if (!cond) fail++; };

// 1. τα STRUCT blocks κάθε στάθμης πρέπει να έχουν max x = 0 και min y = (n+2)*50
const per = {};
merged.blocks.forEach(b => {
  const m = b.name.match(/^FL(-?\d+)_(COLUMN|BEAM|SLAB|FREENODE)\d*$/);
  if (!m || b.name.indexOf('TEXT') >= 0) return;
  const lvl = parseInt(m[1], 10);
  const pos = (merged.inserts[b.name] || [[0, 0]]);
  const box = per[lvl] = per[lvl] || { maxx: -1e18, miny: 1e18 };
  b.body.forEach(ch => {
    if (ch[0].v.trim() !== 'LINE') return;
    const g = c => parseFloat(ch.find(p => p.c === c).v);
    pos.forEach(([ox, oy]) => {
      [[g(10) + ox, g(20) + oy], [g(11) + ox, g(21) + oy]].forEach(([x, y]) => {
        if (x > box.maxx) box.maxx = x;
        if (y < box.miny) box.miny = y;
      });
    });
  });
});
Object.keys(per).map(Number).sort((a, b) => a - b).forEach(lvl => {
  const b = per[lvl], ty = (lvl + 2) * 50;
  chk(Math.abs(b.maxx - 0) < 1e-6, `or${lvl}: pick x -> 0 (βρέθηκε ${b.maxx.toFixed(6)})`);
  chk(Math.abs(b.miny - ty) < 1e-6, `or${lvl}: pick y -> ${ty} (βρέθηκε ${b.miny.toFixed(6)})`);
});

// 2. πλήθος οντοτήτων = άθροισμα εισόδων + τίτλοι
const sumIn = items.reduce((s, it) => s + M._analyze(it.text).ents.length, 0);
chk(merged.ents.length === sumIn + items.length,
  `οντότητες ENTITIES: ${merged.ents.length} = ${sumIn} + ${items.length} τίτλοι`);

// 3. πλήθος blocks: 5 ανά στάθμη, το ARROWHEAD κοινό (1 φορά) + 1 μετονομασμένο
const bn = merged.blocks.map(b => b.name);
chk(new Set(bn).size === bn.length, 'κανένα διπλό όνομα block στο τελικό (' + bn.length + ')');
chk(bn.filter(n => n.indexOf('ARROWHEAD') === 0).length === 2,
  'ARROWHEAD: 1 κοινό + 1 μετονομασμένο -> ' + bn.filter(n => n.indexOf('ARROWHEAD') === 0));

// 4. το INSERT της στάθμης 2 δείχνει στο μετονομασμένο block
const ins2 = merged.ents.filter(c => c[0].v.trim() === 'INSERT')
  .map(c => c.find(p => p.c === 2).v.trim()).filter(n => n.indexOf('ARROWHEAD') === 0);
chk(ins2.some(n => n !== 'ARROWHEAD'), 'INSERT στάθμης 2 -> ' + JSON.stringify(ins2));

// 5. layers: ένωση + LEVEL_TITLE
const layers = merged.tables.find(t => t.type === 'LAYER').entries.map(e => e.name);
chk(layers.indexOf('LEVEL_TITLE') >= 0, 'υπάρχει layer LEVEL_TITLE');
chk(layers.filter(n => /^L-?\d+_ONLY$/.test(n)).length === 5, 'ενώθηκαν τα ανά-στάθμη layers: ' + layers.join(','));
chk(new Set(layers).size === layers.length, 'κανένα διπλό layer');

// 6. τίτλοι
const titles = merged.ents.filter(c => c[0].v.trim() === 'MTEXT' && (c.find(p => p.c === 8) || {}).v === 'LEVEL_TITLE');
chk(titles.length === 5, 'βρέθηκαν ' + titles.length + ' τίτλοι');
console.log('  τίτλοι:', titles.map(t => t.find(p => p.c === 1).v.replace(/\\U\+([0-9A-F]{4})/g,
  (_, h) => String.fromCharCode(parseInt(h, 16)))).join(' | '));

// 7. τα MTEXT των ράβδων μετακινήθηκαν σωστά (10/20 ναι, 11/21 όχι)
chk(!merged.ents.some(c => c[0].v.trim() === 'MTEXT' && c.some(p => p.c === 11)), 'MTEXT: κανένα 11 (ok - δεν προστέθηκε)');

// 8. section order + EOF
const secOrder = res.text.match(/SECTION\n\s*2\n(\w+)/g);
chk(/EOF\s*$/.test(res.text.trim()), 'τελειώνει σε EOF');
console.log('\n' + (fail ? 'ΑΠΕΤΥΧΑΝ ' + fail : 'ΟΛΑ ΤΑ TESTS ΠΕΡΑΣΑΝ'));
process.exit(fail ? 1 : 0);
