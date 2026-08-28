const $ = id => document.getElementById(id);
const log = s => { $("log").textContent += s + "\n"; $("log").scrollTop = 1e9; };
let workerReady = false, t0 = 0, tick = null;
let queue = [];            // {name, file, level, prefix, status, out, audit}
let running = false, sawZero = false, pending = null;

function phase(i, state) { const el = $("ph" + i); if (el) el.className = "ph " + state; }
function busy(on, text) {
  const st = document.getElementById("timer") && document.getElementById("timer").parentElement;
  if (st) st.className = "tb-v" + (on ? " running" : "");
  if (text !== undefined) $("stext").innerHTML = text;
  if (on) { t0 = Date.now(); clearInterval(tick);
    tick = setInterval(() => { $("timer").textContent = ((Date.now()-t0)/1000).toFixed(0) + " s"; }, 500); }
  else clearInterval(tick);
}

/* ==================================================================== */
/*  ΕΝΟΠΟΙΗΤΗΣ ΣΤΑΘΜΩΝ - self.DXFMERGE                                  */
/* ==================================================================== */
/*__MERGE_MODULE__*/

/* ==================================================================== */
const workerCode = `
  const PYODIDE_URL = "https://cdn.jsdelivr.net/pyodide/v0.26.4/full/";
  postMessage({type:"log", s:"[boot] Κατέβασμα Pyodide runtime…"});
  importScripts(PYODIDE_URL + "pyodide.js");
  postMessage({type:"log", s:"[boot] Το runtime κατέβηκε, εκκίνηση…"});
  let pyodide = null;
  const _b64d = s => new TextDecoder("utf-8").decode(Uint8Array.from(atob(s), c => c.charCodeAt(0)));
  self.onmessage = async ev => {
    const m = ev.data;
    try {
      if (m.type === "init") {
        // ΚΡΙΣΙΜΟ: σε Worker από Blob το Pyodide δεν μπορεί να συμπεράνει
        // τη διαδρομή των αρχείων του - χωρίς ρητό indexURL κρεμούσε για πάντα.
        pyodide = await loadPyodide({ indexURL: PYODIDE_URL });
        postMessage({type:"log", s:"[boot] Python έτοιμη, φόρτωση pipeline…"});
        pyodide.setStdout({ batched: s => postMessage({type:"log", s}) });
        pyodide.setStderr({ batched: s => postMessage({type:"log", s}) });
        const bin = Uint8Array.from(atob(m.b64), c => c.charCodeAt(0));
        pyodide.FS.writeFile("pipeline_v11.py", bin);
        postMessage({type:"ready"});
      } else if (m.type === "run") {
        pyodide.FS.writeFile("input.dxf", new Uint8Array(m.buf));
        postMessage({type:"phase", i:1});
        await pyodide.runPythonAsync(_b64d("aW1wb3J0IHN5cwpzeXMubW9kdWxlcy5wb3AoInBpcGVsaW5lX3YxMSIsIE5vbmUpCmltcG9ydCBwaXBlbGluZV92MTEgYXMgUAo="));
        await pyodide.runPythonAsync(_b64d("UC50aWR5KCJpbnB1dC5keGYiLCJvdXRwdXQuZHhmIiwib2Zmc2V0cy5wa2wiKQ=="));
        await pyodide.runPythonAsync(_b64d("cHJpbnQoIi0tLSDOkc6dzpXOns6RzqHOpM6XzqTOn86jIM6VzpvOlc6TzqfOn86jIC0tLSIpClAuYXVkaXQoIm91dHB1dC5keGYiLCJpbnB1dC5keGYiKQ=="));
        const out = pyodide.FS.readFile("output.dxf");
        postMessage({type:"done", out: out.buffer}, [out.buffer]);
      }
    } catch (e) {
      postMessage({type:"error", s: String(e)});
    }
  };
`;
const worker = new Worker(URL.createObjectURL(new Blob([workerCode], {type:"application/javascript"})));

worker.onerror = e => {
  busy(false, '<span class="bad">Σφάλμα στο περιβάλλον εκτέλεσης — δες το log.</span>');
  log("[worker error] " + (e.message || e.type) + (e.filename ? " @ " + e.filename + ":" + e.lineno : ""));
  if (pending) { const p = pending; pending = null; p.rej(new Error("worker error")); }
};
worker.onmessageerror = e => log("[worker messageerror] " + e);

worker.onmessage = ev => {
  const m = ev.data;
  if (m.type === "log") { log(m.s); if (m.s.includes("AUDIT_TOTAL 0")) sawZero = true; }
  else if (m.type === "ready") {
    workerReady = true; phase(0, "done");
    busy(false, "Έτοιμο. " + (queue.length ? "Πάτησε Εκτέλεση." : "Ρίξε ένα ή περισσότερα DXF."));
    maybeEnable();
  }
  else if (m.type === "phase") { phase(m.i-1, "done"); phase(m.i, "on"); }
  else if (m.type === "done") { const p = pending; pending = null; if (p) p.res(new Uint8Array(m.out)); }
  else if (m.type === "error") { log(m.s); const p = pending; pending = null; if (p) p.rej(new Error(m.s)); }
};

busy(true, "Φόρτωση περιβάλλοντος Python… (10-30 δευτερόλεπτα την πρώτη φορά)");
phase(0, "on");
worker.postMessage({type:"init", b64: PIPELINE_B64});

/* ---------------- είσοδος αρχείων ---------------- */
const drop = $("drop");
drop.addEventListener("click", () => $("file").click());
$("pick").addEventListener("click", ev => { ev.stopPropagation(); $("file").click(); });
["dragover","dragenter"].forEach(e => drop.addEventListener(e, ev => { ev.preventDefault(); drop.classList.add("hover"); }));
["dragleave","drop"].forEach(e => drop.addEventListener(e, ev => { ev.preventDefault(); drop.classList.remove("hover"); }));
drop.addEventListener("drop", ev => takeFiles(ev.dataTransfer.files));
$("file").addEventListener("change", ev => { takeFiles(ev.target.files); ev.target.value = ""; });
$("clear").addEventListener("click", ev => { ev.stopPropagation(); queue = []; renderList(); maybeEnable(); });

const LEVEL_Y = n => (n + 2) * 50;

function takeFiles(list) {
  if (running) { log("Τρέχει ήδη μια παρτίδα — περίμενε να τελειώσει."); return; }
  let added = 0, skipped = [];
  for (const f of list || []) {
    if (!/\.dxf$/i.test(f.name)) { skipped.push(f.name); continue; }
    if (queue.some(q => q.name === f.name)) continue;
    const lv = DXFMERGE.parseLevelName(f.name);
    queue.push({ name: f.name, file: f, level: lv ? lv.level : null,
                 prefix: lv ? lv.prefix : null, kind: lv ? lv.kind : 'plan',
                 status: "αναμονή", out: null, audit: null });
    added++;
  }
  queue.sort((a, b) => ((a.level === null ? 999 : a.level) - (b.level === null ? 999 : b.level))
                       || (a.kind === 'plan' ? -1 : 1));
  if (skipped.length) log("Παραλείφθηκαν (όχι .dxf): " + skipped.join(", "));
  const noLevel = queue.filter(q => q.level === null);
  if (noLevel.length) log("⚠ Δεν αναγνωρίστηκε στάθμη (χρειάζεται όνομα …_orN.dxf): " +
                          noLevel.map(q => q.name).join(", ") + " — θα τακτοποιηθούν αλλά ΔΕΝ θα μπουν στο ενοποιημένο.");
  drop.classList.toggle("hasfile", queue.length > 0);
  renderList();
  if (workerReady && !running) busy(false, queue.length ? "Έτοιμο — " + queue.length + " αρχεία. Πάτησε Εκτέλεση." : "Ρίξε ένα ή περισσότερα DXF.");
  maybeEnable();
}

function levelLabel(q) {
  if (q.level === null) return "—";
  const levels = queue.filter(x => x.level !== null && x.kind === 'plan').map(x => x.level);
  if (q.kind === 'detail')
    return "or" + q.level + " · ΛΕΠΤΟΜΕΡΕΙΕΣ ΥΠΟΣΤ. → (5, " + LEVEL_Y(q.level) + ")";
  return "or" + q.level + " · y=" + LEVEL_Y(q.level) + " · " +
         DXFMERGE.levelTitle(q.level, levels.length ? levels : [q.level]);
}

function renderList() {
  const box = $("flist");
  if (!queue.length) { box.innerHTML = '<div class="fl-empty">κανένα αρχείο</div>'; $("tb-file").textContent = "—"; return; }
  $("tb-file").textContent = queue.length + " αρχεία" + (queue[0].prefix ? " · " + queue[0].prefix : "");
  box.innerHTML = queue.map((q, i) => {
    const cls = q.status.indexOf("ΟΚ") === 0 ? "ok" : (q.status.indexOf("ΣΦΑΛΜΑ") === 0 || q.status.indexOf("ΠΡΟΣΟΧΗ") === 0 ? "bad" : "");
    return '<div class="fl-row"><div class="fl-n">' + q.name + '</div>' +
           '<div class="fl-l">' + levelLabel(q) + '</div>' +
           '<div class="fl-s ' + cls + '" id="fs' + i + '">' + q.status + '</div>' +
           '<div class="fl-d" id="fd' + i + '"></div></div>';
  }).join("");
  queue.forEach((q, i) => {
    if (!q.out) return;
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([q.out], { type: "application/dxf" }));
    a.download = q.name.replace(/\.dxf$/i, "") + "_tidied.dxf";
    a.textContent = "⬇";
    a.title = "Κατέβασμα τακτοποιημένου";
    $("fd" + i).appendChild(a);
  });
}

function maybeEnable() { $("run").disabled = !(workerReady && queue.length && !running); }

/* ---------------- εκτέλεση παρτίδας ---------------- */
function runOne(buf) {
  return new Promise((res, rej) => { pending = { res, rej }; worker.postMessage({ type: "run", buf }, [buf]); });
}

$("run").addEventListener("click", runAll);

async function runAll() {
  if (running || !queue.length) return;
  running = true; maybeEnable();
  $("dl").style.display = "none";
  $("audit").className = ""; $("audit").textContent = "σε εξέλιξη";
  const T0 = Date.now();
  let okCount = 0;

  for (let i = 0; i < queue.length; i++) {
    const q = queue[i];
    phase(1, "on"); phase(2, ""); phase(3, "");
    q.status = "τρέχει…"; renderList();
    busy(true, "[" + (i+1) + "/" + queue.length + "] " + q.name + " — τακτοποίηση… (τα μεγάλα σχέδια θέλουν λεπτά)");
    log("\n=== [" + (i+1) + "/" + queue.length + "] ΕΚΤΕΛΕΣΗ: " + q.name + " ===");
    sawZero = false;
    try {
      const buf = await q.file.arrayBuffer();
      if (q.kind === 'detail') {
        /* ΛΕΠΤΟΜΕΡΕΙΕΣ: καμία τακτοποίηση οπλισμού - μόνο διάταξη διατομών */
        busy(true, "[" + (i+1) + "/" + queue.length + "] " + q.name + " — διάταξη διατομών…");
        const res = DXFMERGE.layoutSections(DXFMERGE.bytesToStr(new Uint8Array(buf)));
        q.out = DXFMERGE.strToBytes(res.text);
        q.status = "ΟΚ · " + res.rows + " σειρές";
        okCount++;
        log("  Διατομές: " + res.groups.length + " σύνολα, " + res.rows + " σειρές, φύλλο " +
            res.width.toFixed(1) + " × " + res.height.toFixed(1) + " (×" + DXFMERGE.CFG.SEC_SCALE + ")");
        log("  Σειρά ανάγνωσης: " + res.order.filter(Boolean).join(", "));
        res.warnings.forEach(w => log("  ⚠ " + w));
        phase(1, "done"); phase(2, "done"); phase(3, "done");
      } else {
        const out = await runOne(buf);
        q.audit = sawZero;
        /* περιμετρικές διαστάσεις: ΜΕΤΑ την τακτοποίηση και τον έλεγχο,
           ώστε το AUDIT_TOTAL να αφορά μόνο τα κείμενα οπλισμού */
        let dxf = out;
        try {
          const dm = DXFMERGE.dimensionPlan(DXFMERGE.bytesToStr(out));
          if (dm.added) {
            dxf = DXFMERGE.strToBytes(dm.text);
            const sd = Object.keys(dm.sides).map(k => k + ":" + dm.sides[k].segs).join(" ");
            log("  Διαστάσεις: " + dm.added + " οντότητες σε 4 πλευρές (τμήματα " + sd + ")");
          }
          dm.warnings.forEach(w => log("  ⚠ " + w));
        } catch (e) { log("  ⚠ Οι διαστάσεις δεν μπήκαν: " + (e.message || e)); }
        /* αντίγραφο σκελετού (μόνο COLUMN & BEAM) 100μ αριστερά - ΜΕΤΑ τις
           διαστάσεις, ώστε να μην μετρήσει στην απόσταση ασφαλείας */
        try {
          const cp = DXFMERGE.copyStructure(DXFMERGE.bytesToStr(dxf));
          if (cp.copied) {
            dxf = DXFMERGE.strToBytes(cp.text);
            log("  Αντίγραφο σκελετού σε κενό " + cp.gap + "μ αριστερά (μετατόπιση " +
                cp.dx.toFixed(2) + "): " + (cp.copied - cp.lines) +
                " blocks COLUMN & BEAM + " + cp.lines + " γραμμές slab_poly");
          }
          cp.warnings.forEach(w => log("  ⚠ " + w));
        } catch (e) { log("  ⚠ Το αντίγραφο σκελετού δεν μπήκε: " + (e.message || e)); }
        q.out = dxf;
        q.status = sawZero ? "ΟΚ · audit 0" : "ΠΡΟΣΟΧΗ · audit ≠ 0";
        if (sawZero) okCount++;
        phase(3, "done");
      }
    } catch (e) {
      q.status = "ΣΦΑΛΜΑ"; q.out = null;
      log("✗ " + q.name + ": " + (e.message || e));
    }
    renderList();
  }

  const dt = ((Date.now() - T0) / 60000).toFixed(1);
  log("\n===== ΣΥΓΚΕΝΤΡΩΤΙΚΑ =====");
  queue.forEach(q => log("  " + q.name.padEnd(34) + (q.status.indexOf("ΟΚ") === 0 ? "✓ " + q.status : "✗ " + q.status)));
  log("Σύνολο: " + queue.length + " αρχεία σε " + dt + " λεπτά — " + okCount + " καθαρά (AUDIT_TOTAL 0).");

  $("audit").className = okCount === queue.length ? "ok" : "bad";
  const nPlan = queue.filter(q => q.kind !== 'detail').length;
  $("audit").textContent = okCount + "/" + queue.length + " ΟΚ" + (nPlan < queue.length ? " (οι λεπτ. δεν ελέγχονται με audit)" : "");

  mergeAll();
  running = false; maybeEnable();
}

/* ---------------- ένωση σε ΕΝΑ DXF ---------------- */
function mergeAll() {
  const items = queue.filter(q => q.out && q.level !== null);
  if (!items.length) {
    busy(false, '<span class="bad">Κανένα αρχείο δεν ολοκληρώθηκε — δες το log.</span>');
    return;
  }
  log("\n===== ΕΝΟΠΟΙΗΣΗ ΣΤΑΘΜΩΝ =====");
  let res;
  try {
    res = DXFMERGE.mergeLevels(items.map(q => ({ name: q.name, kind: q.kind, text: DXFMERGE.bytesToStr(q.out) })));
  } catch (e) {
    log("✗ Η ένωση απέτυχε: " + (e.message || e));
    busy(false, '<span class="bad">Τα επιμέρους αρχεία είναι έτοιμα, αλλά η ένωση απέτυχε — δες το log.</span>');
    return;
  }
  res.report.forEach(r => log(
    "  or" + String(r.level).padStart(2) + " " + (r.kind === 'detail' ? "ΛΕΠΤ " : "ΞΥΛΟ ") + r.title.padEnd(34) +
    " pick(" + r.pick[0].toFixed(2) + ", " + r.pick[1].toFixed(2) + ")" +
    " → (" + r.target[0].toFixed(1) + ", " + r.target[1].toFixed(1) + ")" +
    "  μετατόπιση (" + r.delta[0].toFixed(2) + ", " + r.delta[1].toFixed(2) + ")"));
  res.warnings.forEach(w => log("  ⚠ " + w));
  log("  Οντότητες: " + res.counts.entitiesOut + " (από " + res.counts.entitiesIn + ") + " +
      res.counts.titles + " τίτλοι στάθμης.");
  if (res.counts.entitiesIn !== res.counts.entitiesOut) log("  ⚠ ΔΙΑΦΟΡΑ ΣΤΟ ΠΛΗΘΟΣ ΟΝΤΟΤΗΤΩΝ — έλεγξέ το.");

  const prefix = (items[0].prefix || "ktirio");
  const blob = new Blob([DXFMERGE.strToBytes(res.text)], { type: "application/dxf" });
  const a = $("dl");
  a.href = URL.createObjectURL(blob);
  a.download = prefix + "_ΕΝΟΠΟΙΗΜΕΝΟ.dxf";
  a.style.display = "inline-block";
  const okAll = queue.every(q => q.status.indexOf("ΟΚ") === 0);
  busy(false, (okAll ? '<span class="ok">Όλα καθαρά (AUDIT_TOTAL 0). ' : '<span class="bad">Ολοκληρώθηκε με επιφυλάξεις — δες το log. ')
    + 'Ενοποιημένο DXF: ' + res.counts.levels + ' στάθμες ανά ' + DXFMERGE.CFG.STEP_Y + ' μονάδες'
    + (res.counts.details ? ' + ' + res.counts.details + ' φύλλο/α λεπτομερειών' : '') + '.</span>');
}

/* ΚΡΙΣΙΜΟ: αν το αρχείο πέσει ΕΚΤΟΣ ζώνης, ο browser ΕΓΚΑΤΑΛΕΙΠΕΙ τη σελίδα
   και ανοίγει το DXF ως κείμενο. Πλέον ΟΛΗ η σελίδα δέχεται τη ρίψη. */
["dragover","drop"].forEach(e => window.addEventListener(e, ev => ev.preventDefault()));
window.addEventListener("drop", ev => { if (ev.dataTransfer) takeFiles(ev.dataTransfer.files); });

renderList();
