const FIX=require('path').join(__dirname,'fixtures');
/* Τρέχει ΤΟΝ ΙΔΙΟ κώδικα της σελίδας με stubs (χωρίς browser/Pyodide):
   ουρά αρχείων -> "τακτοποίηση" (ο worker επιστρέφει το input) -> ένωση. */
const fs = require('fs'), vm = require('vm');

const els = {};
function el(id) {
  if (els[id]) return els[id];
  const e = {
    id, textContent: '', innerHTML: '', className: '', title: '', href: '', download: '',
    disabled: false, style: {}, children: [], _h: {},
    addEventListener(t, f) { (this._h[t] = this._h[t] || []).push(f); },
    fire(t, ev) { (this._h[t] || []).forEach(f => f(ev || { preventDefault() {}, stopPropagation() {} })); },
    appendChild(c) { this.children.push(c); },
    classList: { add() {}, remove() {}, toggle() {} }
  };
  e.parentElement = e;
  return (els[id] = e);
}
const document = {
  getElementById: el,
  createElement: () => ({ style: {}, classList: { add() {} } }),
  addEventListener() {}
};
const blobs = [];
class Blob { constructor(parts) { this.parts = parts; blobs.push(this); } }
const URL = { createObjectURL: b => 'blob:' + blobs.indexOf(b) };

let workerInstance = null;
class Worker {
  constructor() { workerInstance = this; setTimeout(() => this.onmessage({ data: { type: 'ready' } }), 0); }
  postMessage(m) {
    if (m.type === 'init') return;
    if (m.type === 'run') {
      // ο ψεύτικος pipeline: επιστρέφει το input αυτούσιο + λέει AUDIT_TOTAL 0
      setTimeout(() => {
        this.onmessage({ data: { type: 'phase', i: 1 } });
        this.onmessage({ data: { type: 'log', s: 'AUDIT_TOTAL 0' } });
        this.onmessage({ data: { type: 'done', out: m.buf } });
      }, 0);
    }
  }
}
const self = {};
const ctx = { document, Blob, URL, Worker, self, setTimeout, setInterval, clearInterval, console, Date, Math, JSON, Uint8Array, Promise, String, Number, Object, Array, Error, parseInt, parseFloat, isNaN, isFinite, TextDecoder, atob: s => Buffer.from(s, 'base64').toString('binary') };
ctx.window = ctx; ctx.globalThis = ctx; ctx.self = ctx;
ctx.addEventListener = function(){};

const page = fs.readFileSync(require('path').join(__dirname,'..','build','_extracted.js'),'utf8');
vm.createContext(ctx);
vm.runInContext(page, ctx, { filename: 'index.html:script' });

/* --- ψεύτικα αρχεία από τον φάκελο δοκιμών --- */
const dir = FIX+'/r12_mtext';
const files = fs.readdirSync(dir).filter(n => /_or-?\d+_tidied\.dxf$/.test(n)).sort().reverse(); // ανάποδη σειρά επίτηδες
const fake = files.map(n => {
  const buf = fs.readFileSync(dir + '/' + n);
  return { name: n.replace('_tidied', ''), arrayBuffer: async () => new Uint8Array(buf).buffer };
});

(async () => {
  await new Promise(r => setTimeout(r, 10));            // περίμενε το "ready"
  console.log('status μετά το boot:', el('stext').innerHTML);

  el('file').fire('change', { target: { files: fake, value: '' } });
  console.log('tb-file:', el('tb-file').textContent);
  console.log('σειρά λίστας (ταξινομημένη κατά στάθμη):');
  console.log(el('flist').innerHTML.replace(/<[^>]+>/g, '|').replace(/\|+/g, ' | ').trim());
  console.log('run disabled?', el('run').disabled);

  el('run').fire('click');
  await new Promise(r => setTimeout(r, 400));

  const dl = el('dl');
  console.log('\nκουμπί λήψης:', dl.download, '| ορατό:', dl.style.display);
  console.log('τελικό status:', el('stext').innerHTML);
  console.log('audit row:', el('audit').textContent);

  const merged = blobs[blobs.length - 1];
  const bytes = merged.parts[0];
  const text = Buffer.from(bytes).toString('latin1');
  fs.writeFileSync(FIX+'/r12_mtext/APP_MERGED.dxf', text, 'latin1');
  const M = require('../src/merge_dxf.js');
  const a = M._analyze(text);
  console.log('\nΕΝΟΠΟΙΗΜΕΝΟ: entities', a.ents.length, '| blocks', a.blocks.length,
              '| titles', a.ents.filter(c => (c.find(p => p.c === 8) || {}).v === 'LEVEL_TITLE').length);
  const logTxt = el('log').textContent;
  console.log('\n--- ΤΕΛΕΥΤΑΙΕΣ ΓΡΑΜΜΕΣ LOG ---\n' + logTxt.split('\n').slice(-14).join('\n'));
})();
