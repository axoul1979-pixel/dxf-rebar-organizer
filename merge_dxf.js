/* =====================================================================
   ΕΝΟΠΟΙΗΣΗ ΣΤΑΘΜΩΝ — merge_dxf.js
   Παίρνει τα ΤΑΚΤΟΠΟΙΗΜΕΝΑ DXF (ένα ανά στάθμη) και τα ενώνει σε ΕΝΑ DXF,
   με κάθε κάτοψη μεταφερμένη ώστε το pick point της (max x, min y του
   φέροντος σκελετού) να πέσει στο (BASE_X, (n+2)*STEP_Y).

   Καθαρή JS, καμία εξάρτηση. Δουλεύει σε κείμενο latin-1 (byte-προς-byte).
   ===================================================================== */
(function (global) {
'use strict';

var CFG = {
  STEP_Y: 50.0,                 // κατακόρυφο βήμα σταθμών
  BASE_X: 0.0,                  // x του pick point κάθε στάθμης
  BASE_LEVEL: -2,               // η στάθμη που πάει στο y = 0
  TITLE: true,                  // να γραφτεί τίτλος στάθμης;
  TITLE_H: 0.60,                // ύψος κειμένου τίτλου
  TITLE_DY: -2.00,              // πόσο κάτω από τη βάση της κάτοψης
  TITLE_LAYER: 'LEVEL_TITLE',   // layer τίτλων (ASCII - ασφαλές για R12)
  TITLE_COLOR: 7,
  STRUCT_RE: /^FL-?\d+_(COLUMN|BEAM|SLAB|FREENODE)\d*$/,
  /* --- διατομές υποστυλωμάτων (_Λεπτ_Υποστ_) --- */
  DETAIL_RE: /\u039b\u03b5\u03c0\u03c4[_ ]?\u03a5\u03c0\u03bf\u03c3\u03c4/i,   // «Λεπτ_Υποστ» στο όνομα αρχείου
  SEC_RE: /^FL-?\d+_SECTION\d+$/,
  DETAIL_BASE_X: 5.0,           // pick point (min x, min y) των λεπτομερειών
  SEC_SCALE: 2.5,               // ΤΕΛΙΚΟ insert scale των διατομών (απόλυτο)
  SEC_NAME_GAP: 0.20,           // κενό ονόματος από το περιεχόμενο (πριν το scale)
  SEC_GX: 1.5, SEC_GY: 1.5,     // κενά μεταξύ διατομών (τελικές μονάδες)
  /* --- περιμετρικές διαστάσεις ξυλοτύπου --- */
  DIM_LAYER: 'DIM_PERIMETER',
  DIM_COLOR: 2,                 // κίτρινο (ACI 2)
  DIM_TEXT_H: 0.15,
  DIM_SAFE: 0.25,               // απόσταση ασφαλείας από ΚΑΘΕ στοιχείο του σχεδίου
  /* --- αντίγραφο σκελετού δίπλα στην κάτοψη --- */
  DUP_RE: /^FL-?\d+_(COLUMN|BEAM)\d+$/,   // ΜΟΝΟ κολώνες & δοκοί (όχι κείμενα)
  DUP_GAP: 100.0,               // ΚΕΝΟ 100μ ανάμεσα στο αντίγραφο και την κάτοψη
  DUP_DY: 0.0,
  DUP_LINE_LAYERS: ['slab_poly'],  // + μεμονωμένες γραμμές αυτών των layers
  DIM_TICK: 0.09,               // μισό μήκος ακίδας 45°
  DIM_EXT_GAP: 0.10,            // κενό παρειάς → γραμμή προέκτασης
  DIM_EXT_OVER: 0.13,           // προεξοχή πέρα από τη γραμμή διάστασης
  DIM_TEXT_GAP: 0.05,
  DIM_MIN_SEG: 0.02,            // κάτω από αυτό δεν γράφεται διάσταση
  SEC_LAYER_COLORS: {           // επιβολή χρώματος layer (ACI) στις λεπτομέρειες
    'ironlabels_conline': 5,    //   5 = μπλε
    'section_iron': 6           //   6 = magenta
  },
  SEC_MAX_ROWS: 4,              // ΠΟΤΕ πάνω από 4 σειρές
  SEC_MAX_H: 26.0,              // ΑΠΑΡΑΒΑΤΟ: το ΥΨΟΣ της ζώνης. Μέσα σε αυτό,
                                //   στόχος είναι το ΕΛΑΧΙΣΤΟ ΠΛΑΤΟΣ.
  MAX_NAME: 31                  // όριο μήκους ονόματος block σε R12
};

/* ---------- bytes <-> latin-1 string (1:1, χωρίς αλλοίωση) ---------- */
function bytesToStr(u8) {
  var s = '', CH = 0x8000;
  for (var i = 0; i < u8.length; i += CH) {
    s += String.fromCharCode.apply(null, u8.subarray(i, i + CH));
  }
  return s;
}
function strToBytes(s) {
  var u8 = new Uint8Array(s.length);
  for (var i = 0; i < s.length; i++) u8[i] = s.charCodeAt(i) & 0xff;
  return u8;
}

/* ---------- ονόματα σταθμών ---------- */
function parseLevelName(filename) {
  var base = String(filename).replace(/\.dxf$/i, '');
  base = base.replace(/_tidied$/i, '');
  var m = base.match(/^(.*?)_or_?(-?\d+)$/i);
  if (!m) return null;
  var detail = CFG.DETAIL_RE.test(m[1]);
  return {
    prefix: m[1].replace(CFG.DETAIL_RE, '').replace(/[_\s]+$/, ''),
    level: parseInt(m[2], 10),
    kind: detail ? 'detail' : 'plan'
  };
}

function levelTitle(n, levels) {
  var has = function (k) { return levels.indexOf(k) >= 0; };
  var top = Math.max.apply(null, levels);
  if (n === -2) return 'ΘΕΜΕΛΙΩΣΗ';
  if (n === -1) return has(-2) ? 'ΟΡΟΦΗ ΥΠΟΓΕΙΟΥ' : 'ΘΕΜΕΛΙΩΣΗ';
  if (n === 0) return 'ΟΡΟΦΗ ΙΣΟΓΕΙΟΥ';
  if (n === top) return 'ΟΡΟΦΗ ΑΝΩΤΑΤΗΣ ΣΤΑΘΜΗΣ/ΑΠΟΛ.ΚΛΙΜ.';
  return 'ΟΡΟΦΗ ' + n + 'ΟΥ ΟΡΟΦΟΥ';
}

/* Κείμενο τίτλου στην ΙΔΙΑ κωδικοσελίδα με το υπόλοιπο σχέδιο.
   Τα FESPA DXF είναι ANSI_1253 και γράφουν τα ελληνικά ως raw bytes cp1253·
   αν η κωδικοσελίδα είναι άλλη/άγνωστη, πέφτουμε σε escapes \\U+03A6. */
var CP1253_MAP = "20AC80201A82019283201E8420268520208620218720308920398B201891201992201C93201D94202295201396201497212299203A9B00A0A00385A10386A200A3A300A4A400A5A500A6A600A7A700A8A800A9A900ABAB00ACAC00ADAD00AEAE2015AF00B0B000B1B100B2B200B3B30384B400B5B500B6B600B7B70388B80389B9038ABA00BBBB038CBC00BDBD038EBE038FBF0390C00391C10392C20393C30394C40395C50396C60397C70398C80399C9039ACA039BCB039CCC039DCD039ECE039FCF03A0D003A1D103A3D303A4D403A5D503A6D603A7D703A8D803A9D903AADA03ABDB03ACDC03ADDD03AEDE03AFDF03B0E003B1E103B2E203B3E303B4E403B5E503B6E603B7E703B8E803B9E903BAEA03BBEB03BCEC03BDED03BEEE03BFEF03C0F003C1F103C2F203C3F303C4F403C5F503C6F603C7F703C8F803C9F903CAFA03CBFB03CCFC03CDFD03CEFE";
var CP1253 = (function () {
  var m = {};
  for (var i = 0; i < CP1253_MAP.length; i += 6)
    m[parseInt(CP1253_MAP.substr(i, 4), 16)] = parseInt(CP1253_MAP.substr(i + 4, 2), 16);
  return m;
})();
function dxfEsc(s) {
  var out = '';
  for (var i = 0; i < s.length; i++) {
    var c = s.charCodeAt(i);
    out += (c < 128) ? s[i] : '\\U+' + ('000' + c.toString(16).toUpperCase()).slice(-4);
  }
  return out;
}
function dxfText(s, codepage) {
  if (codepage === 'ANSI_1253') {
    var out = '';
    for (var i = 0; i < s.length; i++) {
      var c = s.charCodeAt(i);
      if (c < 128) { out += s[i]; continue; }
      if (CP1253[c] === undefined) return dxfEsc(s);      // ένας άγνωστος -> όλο σε escapes
      out += String.fromCharCode(CP1253[c]);
    }
    return out;
  }
  return dxfEsc(s);
}

/* προσάρτηση στη θέση της - το `a = a.concat(b)` μέσα σε βρόχο αντιγράφει
   ολόκληρο τον πίνακα κάθε φορά (τετραγωνικό κόστος σε μεγάλα έργα) */
function push_(dst, src) {
  for (var i = 0; i < src.length; i++) dst.push(src[i]);
  return dst;
}

/* ---------- ζεύγη DXF ---------- */
function parsePairs(text) {
  var raw = text.split('\n'), P = [], i = 0;
  while (i < raw.length - 1) {
    var cr = raw[i]; if (cr.charCodeAt(cr.length - 1) === 13) cr = cr.slice(0, -1);
    var c = parseInt(cr.trim(), 10);
    if (isNaN(c)) { i++; continue; }
    var v = raw[i + 1]; if (v.charCodeAt(v.length - 1) === 13) v = v.slice(0, -1);
    P.push({ c: c, cr: cr, v: v });
    i += 2;
  }
  return P;
}
function mk(c, v) { return { c: c, cr: ('  ' + c).slice(-Math.max(3, String(c).length)), v: String(v) }; }
function ser(P, eol) {
  var out = [];
  for (var i = 0; i < P.length; i++) { out.push(P[i].cr); out.push(P[i].v); }
  return out.join(eol) + eol;
}
function num(n) {
  var s = n.toFixed(9).replace(/0+$/, '');
  if (s.charAt(s.length - 1) === '.') s += '0';
  return s;
}

function splitSections(P) {
  var sec = {}, order = [];
  for (var i = 0; i < P.length; i++) {
    if (P[i].c === 0 && P[i].v.trim() === 'SECTION') {
      var name = (P[i + 1] && P[i + 1].c === 2) ? P[i + 1].v.trim() : '';
      var j = i + 2;
      while (j < P.length && !(P[j].c === 0 && P[j].v.trim() === 'ENDSEC')) j++;
      sec[name] = P.slice(i + 2, j);
      order.push(name);
      i = j;
    }
  }
  sec.__order = order;
  return sec;
}

function chunkByZero(P) {
  var out = [], cur = null;
  for (var i = 0; i < P.length; i++) {
    if (P[i].c === 0) { if (cur) out.push(cur); cur = [P[i]]; }
    else if (cur) cur.push(P[i]);
  }
  if (cur) out.push(cur);
  return out;
}
function val(chunk, code) {
  for (var i = 1; i < chunk.length; i++) if (chunk[i].c === code) return chunk[i].v.trim();
  return null;
}

/* ---------- BLOCKS ---------- */
function parseBlocks(P) {
  if (!P) return [];
  var chunks = chunkByZero(P), blocks = [], cur = null;
  for (var i = 0; i < chunks.length; i++) {
    var t = chunks[i][0].v.trim();
    if (t === 'BLOCK') { cur = { head: chunks[i], body: [], end: null }; blocks.push(cur); }
    else if (t === 'ENDBLK') { if (cur) { cur.end = chunks[i]; cur = null; } }
    else if (cur) cur.body.push(chunks[i]);
  }
  for (var k = 0; k < blocks.length; k++) {
    var b = blocks[k];
    b.name = val(b.head, 2) || '';
    b.bx = parseFloat(val(b.head, 10) || '0') || 0;
    b.by = parseFloat(val(b.head, 20) || '0') || 0;
    if (!b.end) b.end = [mk(0, 'ENDBLK')];
  }
  return blocks;
}
function blockPairs(b) {
  var out = b.head.slice();
  for (var i = 0; i < b.body.length; i++) push_(out, b.body[i]);
  return out.concat(b.end);
}
function hashChunks(list) {
  var s = '', h = 5381;
  for (var i = 0; i < list.length; i++)
    for (var j = 0; j < list[i].length; j++) s += list[i][j].c + '\u0001' + list[i][j].v + '\u0002';
  for (var k = 0; k < s.length; k++) { h = ((h << 5) + h + s.charCodeAt(k)) | 0; }
  return h + ':' + s.length;
}

/* ---------- TABLES ---------- */
function parseTables(P) {
  if (!P) return [];
  var chunks = chunkByZero(P), tabs = [], cur = null;
  for (var i = 0; i < chunks.length; i++) {
    var t = chunks[i][0].v.trim();
    if (t === 'TABLE') { cur = { type: val(chunks[i], 2) || '', head: chunks[i], entries: [] }; tabs.push(cur); }
    else if (t === 'ENDTAB') cur = null;
    else if (cur) cur.entries.push({ name: (val(chunks[i], 2) || ''), chunk: chunks[i] });
  }
  return tabs;
}

/* ---------- μετατόπιση οντοτήτων ---------- */
/* ποιοι κωδικοί x (10..18) είναι ΘΕΣΕΙΣ - οι υπόλοιποι είναι διανύσματα */
var XALL = [10, 11, 12, 13, 14, 15, 16, 17, 18];
var XONLY10 = [10];
var POS_CODES = {
  MTEXT: XONLY10,      // 11/21 = διάνυσμα κατεύθυνσης, ΔΕΝ μετακινείται
  ELLIPSE: XONLY10,    // 11/21 = ημιάξονας ως προς το κέντρο
  XLINE: XONLY10, RAY: XONLY10,
  LWPOLYLINE: XONLY10, HATCH: XONLY10, LEADER: XONLY10, IMAGE: XONLY10,
  POLYLINE: []         // 10/20/30 τυπικά 0 - δεν αγγίζονται
};
function translateChunk(chunk, dx, dy) {
  var type = chunk[0].v.trim();
  var allowed = POS_CODES.hasOwnProperty(type) ? POS_CODES[type] : XALL;
  if (!allowed.length) return chunk;
  var out = chunk.slice();
  for (var i = 1; i < out.length; i++) {
    var c = out[i].c;
    if (allowed.indexOf(c) >= 0) {
      var x = parseFloat(out[i].v);
      if (!isNaN(x)) out[i] = mk(c, num(x + dx));
    } else if (c >= 20 && c <= 28 && allowed.indexOf(c - 10) >= 0) {
      var y = parseFloat(out[i].v);
      if (!isNaN(y)) out[i] = mk(c, num(y + dy));
    }
  }
  return out;
}

/* ---------- γεωμετρία: κόσμος ---------- */
function linesOfBlock(b) {
  var L = [];
  for (var i = 0; i < b.body.length; i++) {
    var ch = b.body[i];
    if (ch[0].v.trim() !== 'LINE') continue;
    var x1 = parseFloat(val(ch, 10)), y1 = parseFloat(val(ch, 20));
    var x2 = parseFloat(val(ch, 11)), y2 = parseFloat(val(ch, 21));
    if (isNaN(x1) || isNaN(y1) || isNaN(x2) || isNaN(y2)) continue;
    L.push([x1, y1, x2, y2]);
  }
  return L;
}

function analyzeFile(text) {
  var P = parsePairs(text);
  var sec = splitSections(P);
  var f = {
    pairs: P, sec: sec,
    eol: text.indexOf('\r\n') >= 0 ? '\r\n' : '\n',
    blocks: parseBlocks(sec.BLOCKS),
    tables: parseTables(sec.TABLES),
    ents: chunkByZero(sec.ENTITIES || []),
    header: sec.HEADER || []
  };
  f.blockByName = {};
  for (var i = 0; i < f.blocks.length; i++) f.blockByName[f.blocks[i].name] = f.blocks[i];

  /* θέσεις INSERT (top-level) */
  f.inserts = {};
  for (var j = 0; j < f.ents.length; j++) {
    var ch = f.ents[j];
    if (ch[0].v.trim() !== 'INSERT') continue;
    var nm = val(ch, 2);
    var x = parseFloat(val(ch, 10) || '0'), y = parseFloat(val(ch, 20) || '0');
    var sx = parseFloat(val(ch, 41) || '1'), sy = parseFloat(val(ch, 42) || '1');
    if (!nm) continue;
    (f.inserts[nm] = f.inserts[nm] || []).push([isNaN(x) ? 0 : x, isNaN(y) ? 0 : y,
                                               isNaN(sx) || !sx ? 1 : sx, isNaN(sy) || !sy ? 1 : sy]);
  }

  /* bbox: φέρων σκελετός (για pick point) + ΟΛΑ (για extents) */
  var s = { minx: Infinity, miny: Infinity, maxx: -Infinity, maxy: -Infinity };
  var a = { minx: Infinity, miny: Infinity, maxx: -Infinity, maxy: -Infinity };
  var sc = { minx: Infinity, miny: Infinity, maxx: -Infinity, maxy: -Infinity };
  function put(box, x, y) {
    if (x < box.minx) box.minx = x; if (x > box.maxx) box.maxx = x;
    if (y < box.miny) box.miny = y; if (y > box.maxy) box.maxy = y;
  }
  for (var b = 0; b < f.blocks.length; b++) {
    var blk = f.blocks[b];
    var pos = f.inserts[blk.name] || [[0, 0]];
    var isStruct = CFG.STRUCT_RE.test(blk.name) && blk.name.indexOf('TEXT') < 0;
    var isSec = CFG.SEC_RE.test(blk.name);
    var lines = linesOfBlock(blk);
    for (var p = 0; p < pos.length; p++) {
      var px = pos[p][0], py = pos[p][1], kx = pos[p][2] || 1, ky = pos[p][3] || 1;
      var TX = function (v) { return px + kx * (v - blk.bx); };
      var TY = function (v) { return py + ky * (v - blk.by); };
      for (var l = 0; l < lines.length; l++) {
        var q = lines[l];
        var x1 = TX(q[0]), y1 = TY(q[1]), x2 = TX(q[2]), y2 = TY(q[3]);
        put(a, x1, y1); put(a, x2, y2);
        if (isStruct) { put(s, x1, y1); put(s, x2, y2); }
        if (isSec) { put(sc, x1, y1); put(sc, x2, y2); }
      }
    }
  }
  for (var e = 0; e < f.ents.length; e++) {
    var c2 = f.ents[e], t = c2[0].v.trim();
    if (t === 'LINE') {
      put(a, parseFloat(val(c2, 10)), parseFloat(val(c2, 20)));
      put(a, parseFloat(val(c2, 11)), parseFloat(val(c2, 21)));
      if (CFG.STRUCT_RE.test('') === false) { /* top-level lines: μόνο στα extents */ }
    } else if (t === 'MTEXT' || t === 'TEXT' || t === 'CIRCLE' || t === 'ARC') {
      /* τα INSERT ΔΕΝ μπαίνουν στα όρια: το σημείο εισαγωγής μπορεί να είναι
         πολύ έξω από τη γεωμετρία (η γεωμετρία μετριέται ήδη μέσω των blocks) */
      var xx = parseFloat(val(c2, 10)), yy = parseFloat(val(c2, 20));
      if (!isNaN(xx) && !isNaN(yy)) put(a, xx, yy);
    }
  }
  f.structBox = isFinite(s.minx) ? s : null;
  f.secBox = isFinite(sc.minx) ? sc : null;
  f.allBox = isFinite(a.minx) ? a : null;
  f.codepage = (function () {
    for (var i9 = 0; i9 + 1 < f.header.length; i9++)
      if (f.header[i9].c === 9 && f.header[i9].v.trim() === '$DWGCODEPAGE') return f.header[i9 + 1].v.trim().toUpperCase();
    return '';
  })();
  f.acadver = (function () {
    for (var i2 = 0; i2 + 1 < f.header.length; i2++)
      if (f.header[i2].c === 9 && f.header[i2].v.trim() === '$ACADVER') return f.header[i2 + 1].v.trim();
    return '';
  })();

  /* ---- handles (R13+): εδώ τα αρχεία έχουν ΜΟΝΟ κωδικό 5, χωρίς owner refs ---- */
  f.maxHandle = 0;
  var hx = function (s) { var n = parseInt(String(s).trim(), 16); return isNaN(n) ? 0 : n; };
  for (var q1 = 0; q1 < f.pairs.length; q1++) {
    if (f.pairs[q1].c === 5 || f.pairs[q1].c === 105) {
      var hv = hx(f.pairs[q1].v);
      if (hv > f.maxHandle) f.maxHandle = hv;
    }
  }
  f.hasHandles = f.maxHandle > 0;
  f.tableHead = {};                       // τύπος πίνακα -> handle κεφαλίδας
  for (var q2 = 0; q2 < f.tables.length; q2++) {
    var th = val(f.tables[q2].head, 5);
    if (th) f.tableHead[f.tables[q2].type] = th;
  }
  f.plotStyle = (function () {
    for (var t = 0; t < f.tables.length; t++) {
      if (f.tables[t].type !== 'LAYER') continue;
      for (var e2 = 0; e2 < f.tables[t].entries.length; e2++) {
        var v2 = val(f.tables[t].entries[e2].chunk, 390);
        if (v2) return v2;
      }
    }
    return null;
  })();
  f.objHandles = {};                      // handles της OBJECTS (για 390/340/350)
  var objs = sec.OBJECTS || [];
  for (var q3 = 0; q3 < objs.length; q3++) if (objs[q3].c === 5) f.objHandles[objs[q3].v.trim().toUpperCase()] = 1;
  return f;
}

/* =====================================================================
   ΔΙΑΤΑΞΗ ΔΙΑΤΟΜΩΝ ΥΠΟΣΤΥΛΩΜΑΤΩΝ (_Λεπτ_Υποστ_)
   1) ομαδοποίηση: κάθε διατομή με τα σίδερα/συνδετήρες/ετικέτες/όνομά της
   2) το όνομα έρχεται ακριβώς πάνω από το set (ποτέ πάνω σε ironlabels)
   3) ομοιοθεσία ×2.5 μέσω insert scale - ΚΑΜΙΑ αλλαγή στη γεωμετρία των blocks
   4) στοίχιση σε 3 ή 4 σειρές (όποιο βγαίνει πιο τετράγωνο), με σειρά
      ανάγνωσης το όνομα Κ1, Κ2, … αριστερά→δεξιά
   ===================================================================== */
function _secAnalyze(f) {
  var ins = {}, box = {}, pts = {};
  for (var e = 0; e < f.ents.length; e++) {
    var ch = f.ents[e];
    if (ch[0].v.trim() !== 'INSERT') continue;
    var gv = function (k) { for (var i = 1; i < ch.length; i++) if (ch[i].c === k) return parseFloat(ch[i].v); return null; };
    ins[val(ch, 2)] = { x: gv(10) || 0, y: gv(20) || 0, sx: gv(41) || 1, sy: gv(42) || 1 };
  }
  for (var b = 0; b < f.blocks.length; b++) {
    var blk = f.blocks[b], i2 = ins[blk.name];
    if (!i2) continue;
    var B = { mnx: 1e18, mny: 1e18, mxx: -1e18, mxy: -1e18 }, P = [];
    for (var q = 0; q < blk.body.length; q++) {
      var c2 = blk.body[q], typ = c2[0].v.trim();
      for (var k = 1; k < c2.length - 1; k++) {
        var cd = c2[k].c;
        if (cd < 10 || cd > 13 || c2[k + 1].c !== cd + 10) continue;
        var x = parseFloat(c2[k].v), y = parseFloat(c2[k + 1].v);
        if (isNaN(x) || isNaN(y)) continue;
        var X = i2.x + i2.sx * (x - blk.bx), Y = i2.y + i2.sy * (y - blk.by);
        P.push([X, Y, typ]);
        if (X < B.mnx) B.mnx = X; if (X > B.mxx) B.mxx = X;
        if (Y < B.mny) B.mny = Y; if (Y > B.mxy) B.mxy = Y;
      }
    }
    if (B.mnx < 1e17) { box[blk.name] = B; pts[blk.name] = P; }
  }
  return { ins: ins, box: box, pts: pts };
}
function _bdist(B, x, y) {
  var dx = Math.max(B.mnx - x, 0, x - B.mxx), dy = Math.max(B.mny - y, 0, y - B.mxy);
  return Math.sqrt(dx * dx + dy * dy);
}
function _bin(B, x, y, t) { return x >= B.mnx - t && x <= B.mxx + t && y >= B.mny - t && y <= B.mxy + t; }
function _bgrow(A, B) {
  return { mnx: Math.min(A.mnx, B.mnx), mny: Math.min(A.mny, B.mny), mxx: Math.max(A.mxx, B.mxx), mxy: Math.max(A.mxy, B.mxy) };
}
var CP1253_REV = (function () {
  var m = {};
  for (var i = 0; i < CP1253_MAP.length; i += 6)
    m[parseInt(CP1253_MAP.substr(i + 4, 2), 16)] = parseInt(CP1253_MAP.substr(i, 4), 16);
  return m;
})();
/* ονόματα διατομών: άλλα αρχεία τα γράφουν με escapes \\U+039A, άλλα ως raw
   bytes cp1253 - διαβάζουμε και τα δύο, αλλιώς το «Κ1» φαίνεται ως «Ê1» */
function _decEsc(s, codepage) {
  var t = String(s).replace(/\\U\+([0-9A-Fa-f]{4})/g, function (_m, h) { return String.fromCharCode(parseInt(h, 16)); });
  if (codepage !== 'ANSI_1253') return t;
  var out = '';
  for (var i = 0; i < t.length; i++) {
    var c = t.charCodeAt(i);
    out += (c > 127 && CP1253_REV[c]) ? String.fromCharCode(CP1253_REV[c]) : t[i];
  }
  return out;
}

function layoutSections(text, cfgOverride) {
  var cfg = {}; for (var k0 in CFG) cfg[k0] = CFG[k0];
  if (cfgOverride) for (var k1 in cfgOverride) cfg[k1] = cfgOverride[k1];
  var f = analyzeFile(text), S = _secAnalyze(f);
  var pre = 'FL0_';
  for (var pi = 0; pi < f.blocks.length; pi++) {
    var pm = f.blocks[pi].name.match(/^(FL-?\d+_)SECTION\d+$/);
    if (pm) { pre = pm[1]; break; }
  }
  var esc = function (s2) { return s2.replace(/[-\/\\^$*+?.()|[\]{}]/g, '\\$&'); };
  var numOf = function (n) { var m = n.match(/(\d+)$/); return m ? +m[1] : -1; };
  var anchors = [];
  for (var a = 0; a < f.blocks.length; a++)
    if (cfg.SEC_RE.test(f.blocks[a].name) && S.box[f.blocks[a].name]) anchors.push(f.blocks[a].name);
  if (!anchors.length) throw new Error('Δεν βρέθηκαν διατομές (blocks FLn_SECTIONn) — είναι όντως αρχείο λεπτομερειών;');

  var G = {}, warn = [];
  anchors.forEach(function (n) {
    var B = S.box[n];
    G[n] = { key: n, members: [n], content: { mnx: B.mnx, mny: B.mny, mxx: B.mxx, mxy: B.mxy }, texts: [] };
  });
  var nearest = function (x, y) {
    var best = null, bd = 1e18;
    anchors.forEach(function (m) { var d = _bdist(S.box[m], x, y); if (d < bd) { bd = d; best = m; } });
    return best;
  };
  var names = Object.keys(S.box);

  ['SECTION_IRON', 'SECTION_STIRRUP'].forEach(function (cat) {
    var re = new RegExp('^' + esc(pre) + cat + '\\d+$');
    names.filter(function (n) { return re.test(n); }).forEach(function (n) {
      var B = S.box[n], cx = (B.mnx + B.mxx) / 2, cy = (B.mny + B.mxy) / 2;
      var hit = anchors.filter(function (m) { return _bin(S.box[m], cx, cy, 0.05); });
      var g = G[hit.length === 1 ? hit[0] : nearest(cx, cy)];
      g.members.push(n); g.content = _bgrow(g.content, B);
    });
  });

  var reIdx = new RegExp('^' + esc(pre) + 'SECTION_INDEX\\d+$');
  names.filter(function (n) { return reIdx.test(n); }).forEach(function (n) {
    var hit = {}, lp = S.pts[n].filter(function (p) { return p[2] === 'LINE'; });
    lp.forEach(function (p) { anchors.forEach(function (m) { if (_bin(S.box[m], p[0], p[1], 0.02)) hit[m] = 1; }); });
    var hs = Object.keys(hit), key;
    if (hs.length === 1) key = hs[0];
    else {
      var B2 = S.box[n];
      var c = lp[0] || [(B2.mnx + B2.mxx) / 2, (B2.mny + B2.mxy) / 2];
      key = nearest(c[0], c[1]);
    }
    G[key].members.push(n); G[key].content = _bgrow(G[key].content, S.box[n]);
  });

  var reTxt = new RegExp('^' + esc(pre) + 'SECTION_TEXT\\d+$');
  names.filter(function (n) { return reTxt.test(n); }).forEach(function (n) {
    var own = pre + 'SECTION' + numOf(n), B = S.box[n];
    var key = G[own] ? own : nearest((B.mnx + B.mxx) / 2, (B.mny + B.mxy) / 2);
    if (!G[own]) warn.push('Το ' + own.replace(pre, '') + ' είναι κενό (χωρίς περίγραμμα) — το όνομά του εντάχθηκε στο ' + key.replace(pre, '') + '.');
    G[key].members.push(n); G[key].texts.push(n);
  });

  /* κανόνας 2 */
  var moves = {};
  Object.keys(G).forEach(function (key) {
    var g = G[key];
    g.box = { mnx: g.content.mnx, mny: g.content.mny, mxx: g.content.mxx, mxy: g.content.mxy };
    /* Τα ονόματα του set μπαίνουν όσο πιο χαμηλά γίνεται πάνω από το
       περιεχόμενο. Ανεβαίνουν ΜΟΝΟ αν πέφτουν πάνω σε άλλο όνομα του ίδιου
       set (πολλά ονόματα σε ενιαίο περίγραμμα) - όχι κατά κανόνα. */
    var base = g.box.mxy + cfg.SEC_NAME_GAP, placed = [];
    var order2 = g.texts.slice().sort(function (p, q) { return S.box[p].mnx - S.box[q].mnx; });
    order2.forEach(function (t) {
      var T = S.box[t], y = base, moved = true, guard = 0;
      while (moved && guard++ < 50) {
        moved = false;
        for (var z = 0; z < placed.length; z++) {
          var Q = placed[z];
          var ovX = T.mnx < Q.mxx + cfg.SEC_NAME_GAP && T.mxx > Q.mnx - cfg.SEC_NAME_GAP;
          var ovY = y < Q.mxy + cfg.SEC_NAME_GAP && (y + (T.mxy - T.mny)) > Q.mny;
          if (ovX && ovY) { y = Q.mxy + cfg.SEC_NAME_GAP; moved = true; }
        }
      }
      var dy = y - T.mny;
      moves[t] = [0, dy];
      placed.push({ mnx: T.mnx, mxx: T.mxx, mny: T.mny + dy, mxy: T.mxy + dy });
      g.box = _bgrow(g.box, { mnx: T.mnx, mxx: T.mxx, mny: T.mny + dy, mxy: T.mxy + dy });
    });
  });

  var label = function (key) {
    var t = G[key].texts[0]; if (!t) return '';
    for (var i = 0; i < f.blocks.length; i++) {
      if (f.blocks[i].name !== t) continue;
      var body = f.blocks[i].body;
      for (var j = 0; j < body.length; j++) {
        if (body[j][0].v.trim() !== 'MTEXT') continue;
        if ((val(body[j], 8) || '') !== 'section_prefix_name') continue;
        return _decEsc(val(body[j], 1) || '', f.codepage);
      }
    }
    return '';
  };
  var keys = Object.keys(G);
  keys.sort(function (x, y) {
    var mx = label(x).match(/(\d+)/), my = label(y).match(/(\d+)/);
    return (mx ? +mx[1] : 9999) - (my ? +my[1] : 9999);
  });

  /* ΤΕΛΙΚΟ scale = cfg.SEC_SCALE (απόλυτο). Τα INSERT του FESPA έρχονται με
     δική τους κλίμακα (εδώ 2), οπότε ο συντελεστής ομοιοθεσίας είναι
     SEC_SCALE / (η επικρατούσα αρχική κλίμακα) - ίδιος για ΟΛΑ τα blocks,
     ώστε να μην αλλοιωθούν οι μεταξύ τους αναλογίες. */
  var tally = {};
  Object.keys(S.ins).forEach(function (n) {
    var v = Math.abs(S.ins[n].sx) || 1; tally[v] = (tally[v] || 0) + 1;
  });
  var baseScale = 1, bestN = -1;
  Object.keys(tally).forEach(function (v) { if (tally[v] > bestN) { bestN = tally[v]; baseScale = parseFloat(v); } });
  var K = cfg.SEC_SCALE / (baseScale || 1);
  var items = keys.map(function (key) {
    return { k: key, w: (G[key].box.mxx - G[key].box.mnx) * K, h: (G[key].box.mxy - G[key].box.mny) * K };
  });
  /* ΣΤΟΙΧΙΣΗ - ακριβής βελτιστοποίηση σε ΟΛΕΣ τις κατατμήσεις.
     Η σειρά Κ1→Κν διατηρείται, άρα κάθε σειρά είναι συνεχόμενο κομμάτι.
     Κανόνας (με αυτή τη σειρά προτεραιότητας):
       1. η ζώνη ΔΕΝ ξεπερνά ΠΟΤΕ το SEC_MAX_H·
       2. από όσες λύσεις σπαταλούν έως SEC_AREA_TOL× το εμβαδόν της
          πυκνότερης δυνατής, κρατάμε τη ΜΙΚΡΟΤΕΡΗ ζώνη·
       3. σε ισοπαλία, το στενότερο φύλλο.
     Το βήμα 2 είναι που εμποδίζει τη «λύση-λωρίδα»: μία σειρά δίνει πάντα
     την ελάχιστη ζώνη, αλλά με τεράστιο πλάτος και σπατάλη χώρου. */
  var n = items.length;
  function rowW(i, j) { var w = -cfg.SEC_GX; for (var t = i; t < j; t++) w += items[t].w + cfg.SEC_GX; return w; }
  function rowH(i, j) { var h = 0; for (var t = i; t < j; t++) if (items[t].h > h) h = items[t].h; return h; }

  var cands = [];
  (function enumerate(start, cuts) {
    if (cuts.length > cfg.SEC_MAX_ROWS - 1) return;
    if (start === n) {
      var parts = [], p = 0;
      cuts.concat([n]).forEach(function (c) { parts.push([p, c]); p = c; });
      var W = 0, H = -cfg.SEC_GY;
      parts.forEach(function (q) { var w = rowW(q[0], q[1]); if (w > W) W = w; H += rowH(q[0], q[1]) + cfg.SEC_GY; });
      cands.push({ R: parts.length, W: W, H: H, area: W * H, rows: parts });
      return;
    }
    for (var c2 = start + 1; c2 <= n; c2++) {
      if (c2 < n) enumerate(c2, cuts.concat([c2]));
      else enumerate(n, cuts);
    }
  })(0, []);

  /* Ζώνη ύψους SEC_MAX_H (απαράβατο). Μέσα σε αυτήν, από ΟΛΕΣ τις κατατμήσεις
     κρατάμε αυτή με το ΕΛΑΧΙΣΤΟ ΠΛΑΤΟΣ - και σε ισοπαλία τη χαμηλότερη. */
  var fits = cands.filter(function (x) { return x.H <= cfg.SEC_MAX_H + 1e-9; });
  var best, warnFit = null;
  if (!fits.length) {
    best = cands.reduce(function (a2, b2) { return b2.H < a2.H ? b2 : a2; });
    warnFit = 'Δεν χωράει σε ζώνη ύψους ' + cfg.SEC_MAX_H + ': το ελάχιστο εφικτό ύψος είναι ' +
              best.H.toFixed(1) + ' (η ψηλότερη διατομή από μόνη της είναι ' +
              Math.max.apply(null, items.map(function (x) { return x.h; })).toFixed(1) + ').';
  } else {
    fits.sort(function (x, y) { return (x.W - y.W) || (x.H - y.H); });
    best = fits[0];
  }

  var place = {}, yy = best.H;
  best.rows.forEach(function (p) {
    var rh = rowH(p[0], p[1]), xx = 0;
    for (var t3 = p[0]; t3 < p[1]; t3++) { place[items[t3].k] = [xx, yy - rh]; xx += items[t3].w + cfg.SEC_GX; }
    yy -= rh + cfg.SEC_GY;
  });
  if (warnFit) warn.push(warnFit);

  var newIns = {};
  keys.forEach(function (key) {
    var g = G[key], p = place[key], B = g.box;
    g.members.forEach(function (m) {
      var i3 = S.ins[m]; if (!i3) return;
      var mv = moves[m] || [0, 0];
      newIns[m] = { x: p[0] + K * (i3.x + mv[0] - B.mnx), y: p[1] + K * (i3.y + mv[1] - B.mny), sx: i3.sx * K, sy: i3.sy * K };
    });
  });

  var P = f.pairs, a0 = -1, b0 = -1;
  for (var w = 0; w < P.length; w++) if (P[w].c === 2 && P[w].v.trim() === 'ENTITIES') { a0 = w + 1; break; }
  for (var w2 = a0; w2 < P.length; w2++) if (P[w2].c === 0 && P[w2].v.trim() === 'ENDSEC') { b0 = w2; break; }
  var flat = [], nMoved = 0, nScaled = 0;
  chunkByZero(P.slice(a0, b0)).forEach(function (ch) {
    if (ch[0].v.trim() !== 'INSERT') { push_(flat, ch); return; }
    var n = newIns[val(ch, 2)];
    if (!n) { push_(flat, ch); return; }
    nMoved++;
    var has41 = false, at = -1;
    var out2 = ch.map(function (p, ix) {
      if (p.c === 10) return mk(10, num(n.x));
      if (p.c === 20) { if (at < ix) at = ix; return mk(20, num(n.y)); }
      if (p.c === 30) { at = ix; return p; }
      if (p.c === 41) { has41 = true; return mk(41, num(n.sx)); }
      if (p.c === 42) { has41 = true; return mk(42, num(n.sy)); }
      if (p.c === 43) { has41 = true; return mk(43, num(n.sx)); }
      return p;
    });
    /* ΚΡΙΣΙΜΟ: αν το INSERT δεν έχει καθόλου κλίμακα (πολλά DXF την
       παραλείπουν όταν είναι 1), ΠΡΕΠΕΙ να γραφτεί - αλλιώς οι θέσεις
       μεγεθύνονται και η γεωμετρία μένει 1:1. Μπαίνει στην κανονική
       της θέση, αμέσως μετά τα 10/20/30. */
    if (!has41 && at >= 0) {
      var ins2 = [];
      if (!ch.some(function (p) { return p.c === 30; })) ins2.push(mk(30, '0.0'));
      ins2.push(mk(41, num(n.sx)), mk(42, num(n.sy)), mk(43, num(n.sx)));
      out2 = out2.slice(0, at + 1).concat(ins2, out2.slice(at + 1));
      nScaled++;
    }
    push_(flat, out2);
  });
  var NP = P.slice(0, a0).concat(flat, P.slice(b0));

  /* χρώματα layer: αλλάζει ΜΟΝΟ ο κωδικός 62 της εγγραφής LAYER - καμία
     οντότητα αυτών των layers δεν έχει δικό της χρώμα, οπότε ισχύει παντού */
  var recolored = [];
  for (var z1 = 0; z1 < NP.length - 1; z1++) {
    if (!(NP[z1].c === 0 && NP[z1].v.trim() === 'LAYER')) continue;
    var nm2 = null, at62 = -1;
    for (var z2 = z1 + 1; z2 < NP.length && NP[z2].c !== 0; z2++) {
      if (NP[z2].c === 2 && nm2 === null) nm2 = NP[z2].v.trim();
      if (NP[z2].c === 62) at62 = z2;
    }
    if (nm2 && at62 >= 0 && cfg.SEC_LAYER_COLORS.hasOwnProperty(nm2)) {
      NP[at62] = mk(62, cfg.SEC_LAYER_COLORS[nm2]);
      recolored.push(nm2 + '→' + cfg.SEC_LAYER_COLORS[nm2]);
    }
  }

  return {
    text: ser(NP, f.eol), rows: best.R, width: best.W, height: best.H,
    baseScale: baseScale, finalScale: cfg.SEC_SCALE, factor: K,
    order: keys.map(label), moved: nMoved, scaleAdded: nScaled, warnings: warn, recolored: recolored,
    groups: keys.map(function (key) {
      return { key: key.replace(pre, ''), label: label(key), members: G[key].members.length,
               w: (G[key].box.mxx - G[key].box.mnx) * K, h: (G[key].box.mxy - G[key].box.mny) * K };
    })
  };
}

/* Πλάτος κειμένου - ίδιο μοντέλο με το beambar_engine.py του pipeline,
   βαθμονομημένο σε πραγματικές επικαλύψεις. */
function _charW(ch) {
  if (' .,\''.indexOf(ch) >= 0) return 0.38;
  if ('iIl|'.indexOf(ch) >= 0) return 0.45;
  if ((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || (ch >= '\u0386' && ch <= '\u03AB')) return 0.94;
  return 0.83;
}
function _stripFmt(raw) {
  var s = String(raw).replace(/\\U\+([0-9A-Fa-f]{4})/g, function (_m, h) { return String.fromCharCode(parseInt(h, 16)); });
  s = s.replace(/\\[A-Za-z](\{[^}]*\}|[^\\;]*;)?/g, '');
  return s.replace(/\\P/g, ' ').replace(/[{}]/g, '');
}
function _textW(content, h) {
  var s = _stripFmt(content), w = 0;
  for (var i = 0; i < s.length; i++) w += _charW(s[i]);
  return Math.max(w * h, h * 0.5) + 0.25 * h;
}
/* πλαίσιο MTEXT σε πραγματικές συντεταγμένες, με σημείο αγκύρωσης & στροφή */
function _textBox(x, y, h, rot, attach, content, put) {
  var w = _textW(content, h), hh = h * 1.25, a = attach || 1;
  var x0 = (a % 3 === 1) ? 0 : (a % 3 === 2 ? -w / 2 : -w);
  var y1 = (a <= 3) ? -hh : (a <= 6 ? -hh / 2 : 0);
  var co = Math.cos(rot * Math.PI / 180), si = Math.sin(rot * Math.PI / 180);
  var pts = [[x0, y1], [x0 + w, y1], [x0 + w, y1 + hh], [x0, y1 + hh]];
  for (var i = 0; i < 4; i++) put(x + pts[i][0] * co - pts[i][1] * si, y + pts[i][0] * si + pts[i][1] * co);
}

/* =====================================================================
   ΠΕΡΙΜΕΤΡΙΚΕΣ ΔΙΑΣΤΑΣΕΙΣ ΞΥΛΟΤΥΠΟΥ (preffix_orN)
   Έξω από το bounding box, και στις 4 πλευρές:
     • αλυσίδα: αρχή/τέλος κάθε ΠΕΡΙΜΕΤΡΙΚΗΣ κολώνας και τα κενά μεταξύ τους
     • από πάνω (πιο έξω) μία συνολική διάσταση
   Γράφεται ως ρητή γεωμετρία (γραμμές + ακίδες 45° + MTEXT ύψους 0.15),
   ώστε να ανοίγει παντού· δεν είναι οντότητες DIMENSION του AutoCAD.
   ===================================================================== */
function _worldLineBox(f, blk) {
  /* bbox ΜΟΝΟ από LINE - τα HATCH έχουν σημείο αναφοράς (0,0) που θα
     δηλητηρίαζε το πλαίσιο */
  var pos = f.inserts[blk.name] || [[0, 0, 1, 1]];
  var B = { mnx: 1e18, mny: 1e18, mxx: -1e18, mxy: -1e18 };
  var lines = block_lines_cache(blk);
  for (var p = 0; p < pos.length; p++) {
    var px = pos[p][0], py = pos[p][1], kx = pos[p][2] || 1, ky = pos[p][3] || 1;
    for (var l = 0; l < lines.length; l++) {
      var q = lines[l];
      var xs = [px + kx * (q[0] - blk.bx), px + kx * (q[2] - blk.bx)];
      var ys = [py + ky * (q[1] - blk.by), py + ky * (q[3] - blk.by)];
      for (var i = 0; i < 2; i++) {
        if (xs[i] < B.mnx) B.mnx = xs[i]; if (xs[i] > B.mxx) B.mxx = xs[i];
        if (ys[i] < B.mny) B.mny = ys[i]; if (ys[i] > B.mxy) B.mxy = ys[i];
      }
    }
  }
  return B.mnx < 1e17 ? B : null;
}
var _blc = {};
function block_lines_cache(blk) {
  var k = blk.name;
  if (_blc[k] && _blc[k].b === blk) return _blc[k].v;
  var v = [];
  for (var i = 0; i < blk.body.length; i++) {
    var ch = blk.body[i];
    if (ch[0].v.trim() !== 'LINE') continue;
    var x1 = parseFloat(val(ch, 10)), y1 = parseFloat(val(ch, 20));
    var x2 = parseFloat(val(ch, 11)), y2 = parseFloat(val(ch, 21));
    if (isNaN(x1) || isNaN(y1) || isNaN(x2) || isNaN(y2)) continue;
    v.push([x1, y1, x2, y2]);
  }
  _blc[k] = { b: blk, v: v };
  return v;
}

function dimensionPlan(text, cfgOverride) {
  var cfg = {}; for (var k0 in CFG) cfg[k0] = CFG[k0];
  if (cfgOverride) for (var k1 in cfgOverride) cfg[k1] = cfgOverride[k1];
  var f = analyzeFile(text);

  /* --- κολώνες & συνολικό πλαίσιο --- */
  var cols = [], B = { mnx: 1e18, mny: 1e18, mxx: -1e18, mxy: -1e18 };
  for (var b = 0; b < f.blocks.length; b++) {
    var blk = f.blocks[b], nm = blk.name;
    if (nm.indexOf('TEXT') >= 0) continue;
    var isCol = /^FL-?\d+_(COLUMN|FOOTING)\d+$/.test(nm);
    var isStruct = CFG.STRUCT_RE.test(nm);
    if (!isCol && !isStruct) continue;
    var box = _worldLineBox(f, blk);
    if (!box) continue;
    if (isStruct) {
      if (box.mnx < B.mnx) B.mnx = box.mnx; if (box.mxx > B.mxx) B.mxx = box.mxx;
      if (box.mny < B.mny) B.mny = box.mny; if (box.mxy > B.mxy) B.mxy = box.mxy;
    }
    if (isCol) cols.push({ name: nm, box: box });
  }
  /* στη ΘΕΜΕΛΙΩΣΗ δεν υπάρχουν κολώνες αλλά πέδιλα - χρησιμοποιούνται αυτά */
  var kind = 'κολώνες';
  var onlyCols = cols.filter(function (c) { return /_COLUMN\d+$/.test(c.name); });
  if (onlyCols.length) cols = onlyCols;
  else if (cols.length) kind = 'πέδιλα';
  if (!cols.length || B.mnx > 1e17)
    return { text: text, added: 0, bbox: B.mnx > 1e17 ? null : B, columns: 0, sides: {},
             warnings: ['Δεν βρέθηκαν κολώνες ή πέδιλα - δεν μπήκαν διαστάσεις.'] };

  /* --- πλαίσιο ΠΕΡΙΕΧΟΜΕΝΟΥ: ΟΛΑ όσα ζωγραφίζονται, μαζί με τα πλαίσια των
     κειμένων. Πάνω σε αυτό (και όχι στο bounding box του σκελετού) πατά η
     απόσταση ασφαλείας, ώστε καμία διάσταση να μην ακουμπά τίποτα. --- */
  var C = { mnx: 1e18, mny: 1e18, mxx: -1e18, mxy: -1e18 };
  function putC(x, y) {
    if (isNaN(x) || isNaN(y)) return;
    if (x < C.mnx) C.mnx = x; if (x > C.mxx) C.mxx = x;
    if (y < C.mny) C.mny = y; if (y > C.mxy) C.mxy = y;
  }
  function scanChunk(ch, px, py, sx, sy) {
    var t = ch[0].v.trim();
    if (t === 'MTEXT' || t === 'TEXT') {
      var tx = parseFloat(val(ch, 10)), ty = parseFloat(val(ch, 20));
      var th = parseFloat(val(ch, 40) || '0.1') * (sy || 1);
      var tr = parseFloat(val(ch, 50) || '0');
      var ta = parseInt(val(ch, 71) || '1', 10);
      /* ΧΩΡΙΣ trim: το val() κόβει τα κενά και το αρχικό κενό μετράει στο πλάτος.
         Επίσης τα MTEXT >250 χαρακτήρων σπάνε σε κωδικούς 3 + τελικό 1. */
      var tc = '';
      for (var z = 1; z < ch.length; z++) if (ch[z].c === 3) tc += ch[z].v;
      for (var z2 = 1; z2 < ch.length; z2++) if (ch[z2].c === 1) { tc += ch[z2].v; break; }
      if (!isNaN(tx) && !isNaN(ty))
        _textBox(px + (sx || 1) * (tx - 0), py + (sy || 1) * (ty - 0), th, isNaN(tr) ? 0 : tr,
                 t === 'TEXT' ? 7 : ta, tc, putC);
      return;
    }
    var first10 = -1;
    for (var k = 1; k < ch.length - 1; k++) {
      var cd = ch[k].c;
      if (cd < 10 || cd > 13 || ch[k + 1].c !== cd + 10) continue;
      if (t === 'HATCH') { if (first10 < 0) { first10 = k; continue; } }  // σημείο αναφοράς
      var x = parseFloat(ch[k].v), y = parseFloat(ch[k + 1].v);
      putC(px + (sx || 1) * x, py + (sy || 1) * y);
      if (t === 'CIRCLE' || t === 'ARC') {
        var r = parseFloat(val(ch, 40) || '0') * (sx || 1);
        if (!isNaN(r)) { putC(px + (sx || 1) * x - r, py + (sy || 1) * y - r); putC(px + (sx || 1) * x + r, py + (sy || 1) * y + r); }
      }
    }
  }
  for (var cb = 0; cb < f.blocks.length; cb++) {
    var kb = f.blocks[cb], kp = f.inserts[kb.name];
    if (!kp) continue;
    for (var kq = 0; kq < kp.length; kq++)
      for (var kr = 0; kr < kb.body.length; kr++)
        scanChunk(kb.body[kr], kp[kq][0] - (kp[kq][2] || 1) * kb.bx, kp[kq][1] - (kp[kq][3] || 1) * kb.by,
                  kp[kq][2] || 1, kp[kq][3] || 1);
  }
  for (var ce = 0; ce < f.ents.length; ce++) {
    var ec = f.ents[ce];
    if (ec[0].v.trim() === 'INSERT') continue;
    if (((val(ec, 8) || '')).trim() === cfg.DIM_LAYER) continue;   // παλιές διαστάσεις αγνοούνται
    scanChunk(ec, 0, 0, 1, 1);
  }
  if (C.mnx > 1e17) C = { mnx: B.mnx, mny: B.mny, mxx: B.mxx, mxy: B.mxy };

  /* --- περιμετρικές κολώνες ανά πλευρά ---
     «περιμετρική» = καμία άλλη κολώνα δεν βρίσκεται πιο έξω μέσα στο ίδιο
     λωρίδι· δηλαδή είναι ορατή από εκείνη την πλευρά. */
  var EPS = 1e-6;
  function perim(side) {
    return cols.filter(function (c) {
      return !cols.some(function (o) {
        if (o === c) return false;
        if (side === 'B' || side === 'T') {
          var ov = Math.min(c.box.mxx, o.box.mxx) - Math.max(c.box.mnx, o.box.mnx);
          if (ov <= EPS) return false;
          return side === 'B' ? o.box.mny < c.box.mny - EPS : o.box.mxy > c.box.mxy + EPS;
        }
        var ov2 = Math.min(c.box.mxy, o.box.mxy) - Math.max(c.box.mny, o.box.mny);
        if (ov2 <= EPS) return false;
        return side === 'L' ? o.box.mnx < c.box.mnx - EPS : o.box.mxx > c.box.mxx + EPS;
      });
    });
  }
  function chainOf(side) {
    var P = perim(side), horiz = (side === 'B' || side === 'T'), pts = [];
    P.forEach(function (c) {
      pts.push(horiz ? c.box.mnx : c.box.mny);
      pts.push(horiz ? c.box.mxx : c.box.mxy);
    });
    pts.sort(function (a, b2) { return a - b2; });
    var out = [];
    pts.forEach(function (v) { if (!out.length || v - out[out.length - 1] > cfg.DIM_MIN_SEG) out.push(v); });
    return { pts: out, cols: P };
  }

  /* --- πρότυπο MTEXT & layer --- */
  var tmpl = null, tmplLine = null;
  for (var a1 = 0; a1 < f.ents.length && !tmpl; a1++) if (f.ents[a1][0].v.trim() === 'MTEXT') tmpl = f.ents[a1];
  for (var a4 = 0; a4 < f.ents.length && !tmplLine; a4++) if (f.ents[a4][0].v.trim() === 'LINE') tmplLine = f.ents[a4];
  for (var a5 = 0; a5 < f.blocks.length && !tmplLine; a5++)
    for (var a6 = 0; a6 < f.blocks[a5].body.length && !tmplLine; a6++)
      if (f.blocks[a5].body[a6][0].v.trim() === 'LINE') tmplLine = f.blocks[a5].body[a6];
  for (var a2 = 0; a2 < f.blocks.length && !tmpl; a2++)
    for (var a3 = 0; a3 < f.blocks[a2].body.length && !tmpl; a3++)
      if (f.blocks[a2].body[a3][0].v.trim() === 'MTEXT') tmpl = f.blocks[a2].body[a3];

  var newH = (function () {
    var mx = f.maxHandle || 0;
    return function () { return (++mx).toString(16).toUpperCase(); };
  })();

  var ents = [];
  /* κλώνος υπαρκτής LINE του σχεδίου: ίδια ακολουθία group codes, ώστε να
     μη διαφέρει σε τίποτα από ό,τι δέχεται ήδη το CAD σου */
  var DROP_L = { 62: 1, 370: 1, 330: 1, 360: 1, 6: 1, 48: 1, 39: 1 };
  function line(x1, y1, x2, y2) {
    var vals = {}, used = {}, order = [5, 8, 10, 20, 30, 11, 21, 31];
    var ensure = { 5: 1, 8: 1, 10: 1, 20: 1, 11: 1, 21: 1 };
    if (f.hasHandles) vals[5] = newH(); else delete ensure[5];
    vals[8] = cfg.DIM_LAYER;
    vals[10] = num(x1); vals[20] = num(y1); vals[30] = '0.0';
    vals[11] = num(x2); vals[21] = num(y2); vals[31] = '0.0';
    if (!tmplLine) {
      var basic = [mk(0, 'LINE')];
      if (f.hasHandles) basic.push(mk(5, vals[5]));
      basic.push(mk(8, cfg.DIM_LAYER), mk(10, vals[10]), mk(20, vals[20]),
                 mk(11, vals[11]), mk(21, vals[21]));
      ents.push(basic); return;
    }
    var out = [mk(0, 'LINE')];
    for (var i = 1; i < tmplLine.length; i++) {
      var p = tmplLine[i];
      if (p.c === 102) { while (i < tmplLine.length && !(tmplLine[i].c === 102 && tmplLine[i].v.trim() === '}')) i++; continue; }
      if (DROP_L[p.c]) continue;
      if (vals.hasOwnProperty(p.c)) { if (used[p.c]) continue; used[p.c] = 1; out.push(mk(p.c, vals[p.c])); }
      else out.push(p);
    }
    for (var j = 0; j < order.length; j++) {
      var cd = order[j];
      if (used[cd] || !ensure[cd]) continue;
      var at = -1;
      for (var k = j - 1; k >= 0 && at < 0; k--)
        for (var m = out.length - 1; m >= 0; m--) if (out[m].c === order[k]) { at = m; break; }
      if (at >= 0) out.splice(at + 1, 0, mk(cd, vals[cd])); else out.push(mk(cd, vals[cd]));
      used[cd] = 1;
    }
    ents.push(out);
  }
  function label(x, y, rot, s) {
    if (!tmpl) return;
    var vals = {}, used = {}, order = [5, 8, 10, 20, 30, 40, 41, 71, 50, 1];
    /* Τα 50 (στροφή) και 71 (αγκύρωση) γράφονται ΠΑΝΤΑ, όχι μόνο αν υπάρχουν
       ήδη στο πρότυπο: το MTEXT της FESPA ΔΕΝ έχει κωδικό 50 (η στροφή του
       δηλώνεται στο διάνυσμα 11/21, που εδώ το πετάει το DROP), οπότε χωρίς
       ensure η στροφή δεν γραφόταν ποτέ και ΟΛΕΣ οι κατακόρυφες διαστάσεις
       έβγαιναν ξαπλωμένες (0° αντί 90°). Βρέθηκε σε πραγματικό DAMAR07. */
    var ensure = { 5: 1, 8: 1, 10: 1, 20: 1, 40: 1, 41: 1, 71: 1, 50: 1, 1: 1 };
    if (f.hasHandles) vals[5] = newH(); else delete ensure[5];
    vals[8] = cfg.DIM_LAYER; vals[10] = num(x); vals[20] = num(y); vals[30] = '0.0';
    vals[40] = num(cfg.DIM_TEXT_H); vals[41] = num(cfg.DIM_TEXT_H * 12);
    vals[71] = '5'; vals[50] = num(rot); vals[1] = s;
    var DROP = { 105: 1, 330: 1, 360: 1, 3: 1, 11: 1, 21: 1, 31: 1, 62: 1 };
    var out = [mk(0, 'MTEXT')];
    for (var i = 1; i < tmpl.length; i++) {
      var p = tmpl[i];
      /* ομάδα 102 {…} (π.χ. ACAD_XDICTIONARY) φεύγει ΟΛΟΚΛΗΡΗ - αν έμενε μόνο
         το άνοιγμα/κλείσιμο χωρίς το 360 από μέσα (που το DROP παρακάτω βγάζει),
         το group είναι κατεστραμμένο και το AutoCAD/ezdxf το απορρίπτει. */
      if (p.c === 102) { while (i < tmpl.length && !(tmpl[i].c === 102 && tmpl[i].v.trim() === '}')) i++; continue; }
      if (DROP[p.c]) continue;
      if (vals.hasOwnProperty(p.c)) { if (used[p.c]) continue; used[p.c] = 1; out.push(mk(p.c, vals[p.c])); }
      else out.push(p);
    }
    for (var j = 0; j < order.length; j++) {
      var cd = order[j];
      if (used[cd] || !ensure[cd]) continue;
      var at = -1;
      for (var k = j - 1; k >= 0 && at < 0; k--)
        for (var m = out.length - 1; m >= 0; m--) if (out[m].c === order[k]) { at = m; break; }
      if (at >= 0) out.splice(at + 1, 0, mk(cd, vals[cd])); else out.push(mk(cd, vals[cd]));
      used[cd] = 1;
    }
    ents.push(out);
  }

  var T = cfg.DIM_TICK, H = cfg.DIM_TEXT_H, SAFE = cfg.DIM_SAFE;
  function draw(side, pts, colsOfSide) {
    if (pts.length < 2) return 0;
    var horiz = (side === 'B' || side === 'T');
    var sgn = (side === 'B' || side === 'L') ? -1 : 1;              // φορά προς τα έξω
    /* βάση = η ΑΚΡΑΙΑ ΘΕΣΗ ΤΟΥ ΠΕΡΙΕΧΟΜΕΝΟΥ σε αυτή την πλευρά */
    var edge = horiz ? (sgn < 0 ? C.mny : C.mxy) : (sgn < 0 ? C.mnx : C.mxx);
    var d1 = edge + sgn * (SAFE + T);                               // γραμμή αλυσίδας

    function dline(a, b2, d) { horiz ? line(a, d, b2, d) : line(d, a, d, b2); }
    function tick(v, d) { horiz ? line(v - T, d - T, v + T, d + T) : line(d - T, v - T, d + T, v + T); }
    function ext(v, from, d) {
      var s2 = from, e = d + sgn * cfg.DIM_EXT_OVER;
      horiz ? line(v, s2, v, e) : line(s2, v, e, v);
    }
    function needStagger(len) {
      var w = String(len.toFixed(2)).length * H * 0.62;
      return w > (len - 2 * T - cfg.DIM_TEXT_GAP);
    }
    function txt(a, b2, d, stag) {
      var len = Math.abs(b2 - a);
      if (len < cfg.DIM_MIN_SEG) return;
      var mid = (a + b2) / 2;
      var off = cfg.DIM_TEXT_GAP + H / 2 + (stag ? H * 1.7 : 0);
      var pos = d + sgn * off;
      horiz ? label(mid, pos, 0, len.toFixed(2)) : label(pos, mid, 90, len.toFixed(2));
    }

    /* αλυσίδα */
    dline(pts[0], pts[pts.length - 1], d1);
    pts.forEach(function (v) { tick(v, d1); });
    var maxOut = 0;
    for (var i = 0; i + 1 < pts.length; i++) {
      var len = pts[i + 1] - pts[i];
      if (len < cfg.DIM_MIN_SEG) continue;
      var st = needStagger(len);
      txt(pts[i], pts[i + 1], d1, st);
      var out = cfg.DIM_TEXT_GAP + H + (st ? H * 1.7 : 0);
      if (out > maxOut) maxOut = out;
    }
    pts.forEach(function (v) { ext(v, edge + sgn * SAFE, d1); });

    /* συνολική: πιο έξω από ΟΛΑ τα κείμενα της αλυσίδας, πάλι με 0.25 */
    var d2 = d1 + sgn * (maxOut + SAFE + T);
    dline(pts[0], pts[pts.length - 1], d2);
    tick(pts[0], d2); tick(pts[pts.length - 1], d2);
    txt(pts[0], pts[pts.length - 1], d2, false);
    ext(pts[0], d1 + sgn * (maxOut + SAFE), d2);
    ext(pts[pts.length - 1], d1 + sgn * (maxOut + SAFE), d2);
    return pts.length - 1;
  }

  var report = {};
  ['B', 'T', 'L', 'R'].forEach(function (s) {
    var ch = chainOf(s);
    report[s] = { cols: ch.cols.length, segs: draw(s, ch.pts, ch.cols) };
  });

  /* --- layer διαστάσεων (κλώνος υπαρκτού, για συμβατότητα R2000) --- */
  var P = f.pairs.slice();
  var layIdx = -1, layEnd = -1, tmplLayer = null, exists = false;
  for (var i2 = 0; i2 < P.length - 1; i2++) {
    if (!(P[i2].c === 0 && P[i2].v.trim() === 'LAYER')) continue;
    var j2 = i2 + 1, nm2 = null;
    while (j2 < P.length && P[j2].c !== 0) { if (P[j2].c === 2 && nm2 === null) nm2 = P[j2].v.trim(); j2++; }
    if (nm2 === cfg.DIM_LAYER) exists = true;
    if (!tmplLayer) { tmplLayer = P.slice(i2, j2); layEnd = j2; }
  }
  var addPairs = [];
  if (!exists && tmplLayer) {
    var usedL = {};
    var tmpOut = [];
    for (var ti = 0; ti < tmplLayer.length; ti++) {
      var pL = tmplLayer[ti];
      /* ομάδα 102 {…} (π.χ. ACAD_XDICTIONARY) δεν κληρονομείται ΠΟΤΕ σε κλώνο -
         αλλιώς δύο LAYER θα δείχνουν στο ΙΔΙΟ extension dictionary handle και
         το AutoCAD/ezdxf το απορρίπτει ως ACAD_XDICTIONARY error. */
      if (pL.c === 102 && pL.v.trim().charAt(0) === '{') {
        while (ti < tmplLayer.length && !(tmplLayer[ti].c === 102 && tmplLayer[ti].v.trim() === '}')) ti++;
        continue;
      }
      if (pL.c === 360) continue;                              // extension dictionary handle
      if (pL.c === 2 && !usedL[2]) { usedL[2] = 1; tmpOut.push(mk(2, cfg.DIM_LAYER)); continue; }
      if (pL.c === 62 && !usedL[62]) { usedL[62] = 1; tmpOut.push(mk(62, cfg.DIM_COLOR)); continue; }
      if (pL.c === 5) { tmpOut.push(mk(5, newH())); continue; }
      if (pL.c === 70) { tmpOut.push(mk(70, 0)); continue; }
      tmpOut.push(pL);
    }
    addPairs = tmpOut;
  }

  /* --- εισαγωγή στο αρχείο --- */
  var a0 = -1, b0 = -1;
  for (var w = 0; w < P.length; w++) if (P[w].c === 2 && P[w].v.trim() === 'ENTITIES') { a0 = w + 1; break; }
  for (var w2 = a0; w2 < P.length; w2++) if (P[w2].c === 0 && P[w2].v.trim() === 'ENDSEC') { b0 = w2; break; }
  var flat = [];
  ents.forEach(function (c) { push_(flat, c); });
  var NP = P.slice(0, a0).concat(P.slice(a0, b0), flat, P.slice(b0));
  if (addPairs.length) {
    var shift = (layEnd <= a0) ? 0 : flat.length;
    NP = NP.slice(0, layEnd).concat(addPairs, NP.slice(layEnd));
  }
  /* $HANDSEED */
  if (f.hasHandles) {
    for (var hs = 0; hs + 1 < NP.length; hs++)
      if (NP[hs].c === 9 && NP[hs].v.trim() === '$HANDSEED' && NP[hs + 1].c === 5)
        NP[hs + 1] = mk(5, (f.maxHandle + ents.length * 4 + 32).toString(16).toUpperCase());
  }
  return {
    text: ser(NP, f.eol), added: ents.length, sides: report,
    bbox: B, content: C, columns: cols.length, kind: kind, warnings: []
  };
}


/* =====================================================================
   ΑΝΤΙΓΡΑΦΟ ΣΚΕΛΕΤΟΥ: δίπλα σε κάθε κάτοψη, αντίγραφο ΜΟΝΟ των blocks
   COLUMN και BEAM, μετατοπισμένο κατά DUP_DX (100μ αριστερά).
   Δεν δημιουργούνται νέα blocks - μπαίνουν μόνο νέα INSERT στα ίδια
   blocks, οπότε η γεωμετρία είναι εξ ορισμού πανομοιότυπη.
   ===================================================================== */
function copyStructure(text, cfgOverride) {
  var cfg = {}; for (var k0 in CFG) cfg[k0] = CFG[k0];
  if (cfgOverride) for (var k1 in cfgOverride) cfg[k1] = cfgOverride[k1];
  var f = analyzeFile(text);
  var mx = f.maxHandle || 0;
  function newH() { return (++mx).toString(16).toUpperCase(); }

  /* Α' φάση: ποιο είναι το δεξί άκρο ΤΟΥ ΑΝΤΙΓΡΑΦΟΥ και το αριστερό άκρο της
     κάτοψης (μαζί με τις διαστάσεις της) - ώστε το ΚΕΝΟ να βγει ακριβώς
     DUP_GAP, όχι η μετατόπιση. */
  var srcMaxX = -1e18;
  function noteX(x) { if (x > srcMaxX) srcMaxX = x; }
  for (var s1 = 0; s1 < f.blocks.length; s1++) {
    var sb = f.blocks[s1], sp = f.inserts[sb.name];
    if (!sp || !cfg.DUP_RE.test(sb.name)) continue;
    var sl = linesOfBlock(sb);
    for (var s2 = 0; s2 < sp.length; s2++)
      for (var s3 = 0; s3 < sl.length; s3++) {
        noteX(sp[s2][0] + (sp[s2][2] || 1) * (sl[s3][0] - sb.bx));
        noteX(sp[s2][0] + (sp[s2][2] || 1) * (sl[s3][2] - sb.bx));
      }
  }
  var wantLayer0 = {};
  (cfg.DUP_LINE_LAYERS || []).forEach(function (L) { wantLayer0[L] = 1; });
  for (var s4 = 0; s4 < f.blocks.length; s4++) {
    var sb2 = f.blocks[s4], sp2 = f.inserts[sb2.name];
    if (!sp2) continue;
    for (var s5 = 0; s5 < sb2.body.length; s5++) {
      var sc = sb2.body[s5];
      if (sc[0].v.trim() !== 'LINE' || !wantLayer0[(val(sc, 8) || '').trim()]) continue;
      var a1 = parseFloat(val(sc, 10)), a2 = parseFloat(val(sc, 11));
      for (var s6 = 0; s6 < sp2.length; s6++) {
        noteX(sp2[s6][0] + (sp2[s6][2] || 1) * (a1 - sb2.bx));
        noteX(sp2[s6][0] + (sp2[s6][2] || 1) * (a2 - sb2.bx));
      }
    }
  }
  var planMinX = (f.allBox ? f.allBox.minx : 0);
  var DX = (srcMaxX > -1e17) ? (planMinX - cfg.DUP_GAP - srcMaxX) : -cfg.DUP_GAP;
  cfg.DUP_DX = DX;

  var copies = [], names = {};
  for (var i = 0; i < f.ents.length; i++) {
    var ch = f.ents[i];
    if (ch[0].v.trim() !== 'INSERT') continue;
    var nm = val(ch, 2);
    if (!nm || !cfg.DUP_RE.test(nm)) continue;
    names[nm] = 1;
    var seen10 = false, seen20 = false;
    var out = ch.map(function (p) {
      if (p.c === 5) return mk(5, newH());
      if (p.c === 10 && !seen10) { seen10 = true; var x = parseFloat(p.v); return isNaN(x) ? p : mk(10, num(x + cfg.DUP_DX)); }
      if (p.c === 20 && !seen20) { seen20 = true; var y = parseFloat(p.v); return isNaN(y) ? p : mk(20, num(y + cfg.DUP_DY)); }
      return p;
    });
    copies.push(out);
  }
  /* Οι γραμμές του slab_poly ζουν ΜΕΣΑ στα SLAB blocks, μαζί με δείκτες και
     κείμενα - άρα δεν γίνεται να αντιγραφεί το block. Αντιγράφονται μία-μία
     ως αυτόνομες LINE, με κλώνο της ίδιας της γραμμής (ίδιοι κωδικοί, ίδιο
     layer) και συντεταγμένες μετατρεμμένες σε πραγματικές. */
  var wantLayer = {};
  (cfg.DUP_LINE_LAYERS || []).forEach(function (L) { wantLayer[L] = 1; });
  var nLines = 0, DROP_C = { 102: 1, 330: 1, 360: 1 };
  for (var bi = 0; bi < f.blocks.length; bi++) {
    var blk = f.blocks[bi], pos = f.inserts[blk.name];
    if (!pos) continue;
    for (var bj = 0; bj < blk.body.length; bj++) {
      var ch2 = blk.body[bj];
      if (ch2[0].v.trim() !== 'LINE') continue;
      if (!wantLayer[(val(ch2, 8) || '').trim()]) continue;
      var lx1 = parseFloat(val(ch2, 10)), ly1 = parseFloat(val(ch2, 20));
      var lx2 = parseFloat(val(ch2, 11)), ly2 = parseFloat(val(ch2, 21));
      if (isNaN(lx1) || isNaN(ly1) || isNaN(lx2) || isNaN(ly2)) continue;
      for (var bp = 0; bp < pos.length; bp++) {
        var px = pos[bp][0], py = pos[bp][1], sx = pos[bp][2] || 1, sy = pos[bp][3] || 1;
        var X1 = px + sx * (lx1 - blk.bx) + cfg.DUP_DX, Y1 = py + sy * (ly1 - blk.by) + cfg.DUP_DY;
        var X2 = px + sx * (lx2 - blk.bx) + cfg.DUP_DX, Y2 = py + sy * (ly2 - blk.by) + cfg.DUP_DY;
        var seen = {}, out2 = [];
        for (var q = 0; q < ch2.length; q++) {
          var p2 = ch2[q];
          if (p2.c === 102) { while (q < ch2.length && !(ch2[q].c === 102 && ch2[q].v.trim() === '}')) q++; continue; }
          if (DROP_C[p2.c]) continue;
          if (p2.c === 5) { out2.push(mk(5, newH())); continue; }
          if (p2.c === 10 && !seen[10]) { seen[10] = 1; out2.push(mk(10, num(X1))); continue; }
          if (p2.c === 20 && !seen[20]) { seen[20] = 1; out2.push(mk(20, num(Y1))); continue; }
          if (p2.c === 11 && !seen[11]) { seen[11] = 1; out2.push(mk(11, num(X2))); continue; }
          if (p2.c === 21 && !seen[21]) { seen[21] = 1; out2.push(mk(21, num(Y2))); continue; }
          out2.push(p2);
        }
        copies.push(out2); nLines++;
      }
    }
  }

  if (!copies.length) return { text: text, copied: 0, blocks: 0, lines: 0, warnings: ['Δεν βρέθηκαν blocks COLUMN/BEAM - δεν έγινε αντίγραφο.'] };

  var P = f.pairs, a0 = -1, b0 = -1;
  for (var w = 0; w < P.length; w++) if (P[w].c === 2 && P[w].v.trim() === 'ENTITIES') { a0 = w + 1; break; }
  for (var w2 = a0; w2 < P.length; w2++) if (P[w2].c === 0 && P[w2].v.trim() === 'ENDSEC') { b0 = w2; break; }
  var flat = [];
  copies.forEach(function (c) { push_(flat, c); });
  var NP = P.slice(0, b0).concat(flat, P.slice(b0));
  if (f.hasHandles) {
    for (var hs = 0; hs + 1 < NP.length; hs++)
      if (NP[hs].c === 9 && NP[hs].v.trim() === '$HANDSEED' && NP[hs + 1].c === 5)
        NP[hs + 1] = mk(5, (mx + 16).toString(16).toUpperCase());
  }
  return { text: ser(NP, f.eol), copied: copies.length, blocks: Object.keys(names).length,
           lines: nLines, dx: DX, gap: cfg.DUP_GAP, warnings: [] };
}

/* ---------- κύρια συνάρτηση ---------- */
/* items: [{name, text}]  ->  {text, report:[], warnings:[]} */
function mergeLevels(items, cfgOverride) {
  var cfg = {}; for (var k in CFG) cfg[k] = CFG[k];
  if (cfgOverride) for (var k2 in cfgOverride) cfg[k2] = cfgOverride[k2];

  var warnings = [], report = [], recs = [];

  for (var i = 0; i < items.length; i++) {
    var lv = parseLevelName(items[i].name);
    if (!lv) { warnings.push('Παραλείπεται (δεν ταιριάζει με …_orN.dxf): ' + items[i].name); continue; }
    recs.push({ name: items[i].name, prefix: lv.prefix, level: lv.level,
                kind: items[i].kind || lv.kind || 'plan', f: analyzeFile(items[i].text) });
  }
  if (!recs.length) throw new Error('Κανένα αρχείο με έγκυρο όνομα στάθμης (…_or-2 … _or9).');

  /* ομαδοποίηση κατά prefix - αν έχει πολλά, ενοποιούμε μόνο το πρώτο */
  var prefixes = [];
  for (var r = 0; r < recs.length; r++) if (prefixes.indexOf(recs[r].prefix) < 0) prefixes.push(recs[r].prefix);
  if (prefixes.length > 1) warnings.push('Βρέθηκαν πολλά προθέματα: ' + prefixes.join(', '));

  recs.sort(function (a, b) { return (a.level - b.level) || (a.kind === 'plan' ? -1 : 1); });
  var seenLK = {};
  recs.forEach(function (r) {
    var k = r.level + '|' + r.kind;
    if (seenLK[k]) throw new Error('Δύο αρχεία ' + (r.kind === 'detail' ? 'λεπτομερειών' : 'ξυλοτύπου') + ' για τη στάθμη or' + r.level + '.');
    seenLK[k] = 1;
  });
  var levels = [];
  recs.forEach(function (r) { if (levels.indexOf(r.level) < 0) levels.push(r.level); });
  levels.sort(function (a, b) { return a - b; });
  for (var d = 1; d < levels.length; d++)
    if (levels[d] !== levels[d - 1] + 1)
      warnings.push('Κενό στη σειρά σταθμών: μετά το or' + levels[d - 1] + ' ακολουθεί or' + levels[d] + '.');
  /* ---- handles: ο πρώτος (χαμηλότερη στάθμη) είναι ο «οικοδεσπότης».
         Όλων των υπολοίπων τα handles ξαναριθμούνται ώστε να μη διπλοχτυπήσουν. ---- */
  var host = recs[0].f;
  var hasH = host.hasHandles;
  var seed = host.maxHandle;
  for (var s1 = 0; s1 < recs.length; s1++) if (recs[s1].f.maxHandle > seed) seed = recs[s1].f.maxHandle;
  var nextH = seed + 1;
  function newHandle() { return (nextH++).toString(16).toUpperCase(); }
  for (var s2 = 0; s2 < recs.length; s2++) recs[s2].hmap = {};
  var hostPS = host.plotStyle;
  /* ευρετήριο πινάκων του τελικού: τύπος -> {όνομα: handle} */
  var tableIndex = {};
  if (hasH) {
    host.tables.forEach(function (t) {
      var m = tableIndex[t.type] = {};
      t.entries.forEach(function (e) { var h = val(e.chunk, 5); if (h) m[e.name] = h.trim(); });
    });
  }
  function remapH(rec, v) {
    var k = String(v).trim().toUpperCase();
    if (!rec.hmap[k]) rec.hmap[k] = newHandle();
    return rec.hmap[k];
  }
  /* αντικατάσταση handles/αναφορών σε ένα chunk (μόνο για ΜΗ-οικοδεσπότες) */
  function fixH(rec, chunk) {
    if (!hasH || rec === recs[0]) return chunk;
    var out = [];
    for (var i7 = 0; i7 < chunk.length; i7++) {
      var p = chunk[i7];
      /* ομάδες 102 {…} δείχνουν σε OBJECTS που δεν μεταφέρονται - φεύγουν */
      if (p.c === 102 && p.v.trim().charAt(0) === '{') {
        while (i7 < chunk.length && !(chunk[i7].c === 102 && chunk[i7].v.trim() === '}')) i7++;
        continue;
      }
      if (p.c === 5 || p.c === 105) { out.push(mk(p.c, remapH(rec, p.v))); continue; }
      if (p.c === 360) continue;                       // extension dictionary
      if (p.c === 390) {                               // PlotStyleName: ΠΟΤΕ δεν λείπει
        out.push(mk(390, hostPS || p.v.trim()));
        continue;
      }
      if (p.c === 330 || p.c === 340 || p.c === 350 || p.c === 347) {
        var tk = p.v.trim().toUpperCase();
        if (rec.hmap[tk]) out.push(mk(p.c, rec.hmap[tk]));
        continue;                                      // αλλιώς κόβεται (δείχνει σε ό,τι δεν μεταφέρθηκε)
      }
      out.push(p);
    }
    return out;
  }
  /* ΠΡΟΕΡΓΑΣΙΑ: πρώτα οι εγγραφές πινάκων (τα διπλά ονόματα δείχνουν στου
     οικοδεσπότη), μετά όλα τα υπόλοιπα handles - ώστε κάθε αναφορά να λύνεται. */
  function prepHandles() {
    if (!hasH) return;
    for (var r1 = 1; r1 < recs.length; r1++) {
      var rec = recs[r1];
      rec.f.tables.forEach(function (t) {
        var idx = tableIndex[t.type] || (tableIndex[t.type] = {});
        t.entries.forEach(function (e) {
          var h = val(e.chunk, 5); if (!h) return;
          var nm = (t.type === 'BLOCK_RECORD' && rec.rename[e.name]) ? rec.rename[e.name] : e.name;
          var k = h.trim().toUpperCase();
          if (idx[nm]) rec.hmap[k] = idx[nm];           // διπλό όνομα -> handle του τελικού
          else { rec.hmap[k] = newHandle(); idx[nm] = rec.hmap[k]; }
        });
      });
      for (var q9 = 0; q9 < rec.f.pairs.length; q9++) {
        var pp = rec.f.pairs[q9];
        if (pp.c !== 5 && pp.c !== 105) continue;
        var kk = pp.v.trim().toUpperCase();
        if (!rec.hmap[kk]) rec.hmap[kk] = newHandle();
      }
    }
  }
  if (host.acadver && host.acadver !== 'AC1009' && !hasH)
    warnings.push('Έκδοση ' + host.acadver + ' χωρίς handles - έλεγξε το αποτέλεσμα στο CAD.');

  /* ---- συλλογή blocks με έλεγχο συγκρούσεων ---- */
  var blockSeen = {};      // name -> hash
  var outBlocks = [];      // pairs
  for (var ri = 0; ri < recs.length; ri++) {
    var rec = recs[ri], f = rec.f;
    rec.rename = {};
    for (var bi = 0; bi < f.blocks.length; bi++) {
      var b = f.blocks[bi], h = hashChunks(f.blocks[bi].body);
      if (blockSeen.hasOwnProperty(b.name)) {
        if (blockSeen[b.name] === h) { b.skip = true; continue; }   // ίδιο block, γράφεται μία φορά
        var suffix = '$' + (rec.kind === 'detail' ? 'D' : 'L') + String(rec.level).replace('-', 'M');
        var nn = (b.name.length + suffix.length > cfg.MAX_NAME)
          ? b.name.slice(0, cfg.MAX_NAME - suffix.length) + suffix : b.name + suffix;
        var g = 1; while (blockSeen.hasOwnProperty(nn)) { nn = nn.slice(0, cfg.MAX_NAME - 2) + '$' + (g++); }
        rec.rename[b.name] = nn;
        rec.nRenamed = (rec.nRenamed || 0) + 1;
        if (rec.nRenamed === 1) rec.firstRenamed = b.name + ' → ' + nn;
        b.newName = nn;
      }
      blockSeen[b.newName || b.name] = h;
    }
    if (rec.nRenamed)
      warnings.push(rec.nRenamed + ' blocks του ' + rec.name + ' είχαν όνομα που χρησιμοποιείται ήδη με άλλο περιεχόμενο ' +
                    'και μετονομάστηκαν (π.χ. ' + rec.firstRenamed + ') - τα INSERT ενημερώθηκαν.');
  }

  prepHandles();

  function applyRename(chunk, ren) {
    if (chunk[0].v.trim() !== 'INSERT') return chunk;
    var out = chunk.slice();
    for (var i3 = 1; i3 < out.length; i3++)
      if (out[i3].c === 2 && ren.hasOwnProperty(out[i3].v.trim())) out[i3] = mk(2, ren[out[i3].v.trim()]);
    return out;
  }

  /* ---- blocks section ---- */
  for (var ri2 = 0; ri2 < recs.length; ri2++) {
    var rc = recs[ri2], hasRen = Object.keys(rc.rename).length > 0;
    for (var bj = 0; bj < rc.f.blocks.length; bj++) {
      var bb = rc.f.blocks[bj];
      if (bb.skip) continue;
      var head = fixH(rc, bb.head).slice();
      if (bb.newName) {
        for (var hi = 1; hi < head.length; hi++)
          if (head[hi].c === 2 || head[hi].c === 3) head[hi] = mk(head[hi].c, bb.newName);
      }
      push_(outBlocks, head);
      for (var bo = 0; bo < bb.body.length; bo++)
        push_(outBlocks, fixH(rc, hasRen ? applyRename(bb.body[bo], rc.rename) : bb.body[bo]));
      push_(outBlocks, fixH(rc, bb.end));
    }
  }

  /* ---- tables (ένωση κατά όνομα, πρώτο κερδίζει) ---- */
  var outTabs = [], byType = {};
  for (var t0 = 0; t0 < recs[0].f.tables.length; t0++) {
    var T = recs[0].f.tables[t0];
    var copy = { type: T.type, head: T.head, entries: T.entries.slice(), seen: {} };
    for (var q0 = 0; q0 < copy.entries.length; q0++) copy.seen[copy.entries[q0].name] = 1;
    outTabs.push(copy); byType[T.type] = copy;
  }
  for (var t1 = 1; t1 < recs.length; t1++) {
    var TT = recs[t1].f.tables, rcT = recs[t1];
    for (var t2 = 0; t2 < TT.length; t2++) {
      var dst = byType[TT[t2].type];
      if (!dst) { dst = { type: TT[t2].type, head: TT[t2].head, entries: [], seen: {} }; outTabs.push(dst); byType[TT[t2].type] = dst; }
      for (var t3 = 0; t3 < TT[t2].entries.length; t3++) {
        var en = TT[t2].entries[t3];
        var nm2 = en.name;
        var chunk2 = en.chunk;
        /* block που μετονομάστηκε -> μετονομάζεται και το BLOCK_RECORD του */
        if (TT[t2].type === 'BLOCK_RECORD' && rcT.rename.hasOwnProperty(nm2)) {
          nm2 = rcT.rename[nm2];
          chunk2 = chunk2.map(function (p) { return p.c === 2 ? mk(2, nm2) : p; });
        }
        if (dst.seen[nm2]) continue;
        dst.seen[nm2] = 1;
        chunk2 = fixH(rcT, chunk2);
        if (hasH) {
          var ownH = host.tableHead[TT[t2].type];
          if (ownH) chunk2 = chunk2.map(function (p) { return p.c === 330 ? mk(330, ownH) : p; });
        }
        dst.entries.push({ name: nm2, chunk: chunk2 });
      }
    }
  }
  /* layer τίτλων: κλώνος υπαρκτού LAYER (κρατά subclass markers/390/handle μορφή) */
  if (cfg.TITLE) {
    var lay = byType['LAYER'];
    if (lay && !lay.seen[cfg.TITLE_LAYER]) {
      lay.seen[cfg.TITLE_LAYER] = 1;
      var tmplL = lay.entries.length ? lay.entries[0].chunk : null, chunkL, usedL = {};
      if (tmplL) {
        chunkL = tmplL.map(function (p) {
          if (p.c === 2 && !usedL[2]) { usedL[2] = 1; return mk(2, cfg.TITLE_LAYER); }
          if (p.c === 62 && !usedL[62]) { usedL[62] = 1; return mk(62, cfg.TITLE_COLOR); }
          if (p.c === 5) return mk(5, newHandle());
          if (p.c === 70) return mk(70, 0);
          return p;
        });
      } else {
        chunkL = [mk(0, 'LAYER'), mk(2, cfg.TITLE_LAYER), mk(70, 0), mk(62, cfg.TITLE_COLOR), mk(6, 'CONTINUOUS')];
      }
      lay.entries.push({ name: cfg.TITLE_LAYER, chunk: chunkL });
    }
  }

  /* ---- πρότυπο MTEXT (κληρονομεί style/κωδικούς του σχεδίου) ---- */
  var tmpl = null;
  (function () {
    for (var a1 = 0; a1 < 1 && !tmpl; a1++) {          // μόνο ο οικοδεσπότης
      var ff = recs[a1].f;
      for (var a2 = 0; a2 < ff.ents.length; a2++) if (ff.ents[a2][0].v.trim() === 'MTEXT') { tmpl = ff.ents[a2]; return; }
      for (var a3 = 0; a3 < ff.blocks.length; a3++)
        for (var a4 = 0; a4 < ff.blocks[a3].body.length; a4++)
          if (ff.blocks[a3].body[a4][0].v.trim() === 'MTEXT') { tmpl = ff.blocks[a3].body[a4]; return; }
    }
  })();

  /* Ο τίτλος γράφεται ως ΚΛΩΝΟΣ υπαρκτού MTEXT του σχεδίου (κληρονομεί
     style/subclass markers/κωδικούς - ό,τι δέχεται ήδη το CAD σου).
     Αν το σχέδιο δεν έχει MTEXT, πέφτουμε σε TEXT (R12-native, πάντα ασφαλές). */
  /* handles (θα ήταν διπλά), επιπλέον κομμάτια κειμένου, και το διάνυσμα
     κατεύθυνσης 11/21/31 (θα κληρονομούσε στροφή ξένου κειμένου) */
  var DROP = { 105: 1, 330: 1, 360: 1, 3: 1, 11: 1, 21: 1, 31: 1, 62: 1 };
  function titleEntity(x, y, txt) {
    if (!tmpl) {
      var style = null;
      var st = byType['STYLE'];
      if (st && st.entries.length) style = st.entries[0].name;
      var th0 = [mk(0, 'TEXT')];
      if (hasH) th0.push(mk(5, newHandle()));
      return th0.concat([mk(8, cfg.TITLE_LAYER),
        mk(10, num(x)), mk(20, num(y)), mk(30, '0.0'),
        mk(40, num(cfg.TITLE_H)), mk(1, dxfText(txt, host.codepage)),
        mk(50, '0.0'), mk(7, style || 'STANDARD'), mk(72, 0), mk(73, 0)]);
    }
    /* αντικατάσταση ΕΠΙ ΤΟΠΟΥ ώστε να μη χαλάσει η σειρά των group codes
       (τα 100 subclass markers πρέπει να προηγούνται των πεδίων τους) */
    /* canon = κανονική σειρά group codes (για τη ΘΕΣΗ όσων λείπουν)
       ensure = μόνο αυτά προστίθενται αν λείπουν· τα υπόλοιπα (30/41/71/50)
       γράφονται μόνο αν υπάρχουν ήδη, ώστε ο τίτλος να έχει ΑΚΡΙΒΩΣ την ίδια
       ακολουθία κωδικών με τα MTEXT του σχεδίου. */
    var vals = {}, used = {}, order = [5, 8, 10, 20, 30, 40, 41, 71, 50, 1];
    var ensure = { 5: 1, 8: 1, 10: 1, 20: 1, 40: 1, 1: 1 };
    if (hasH) vals[5] = newHandle(); else { DROP[5] = 1; delete ensure[5]; }
    vals[8] = cfg.TITLE_LAYER; vals[10] = num(x); vals[20] = num(y); vals[30] = '0.0';
    vals[40] = num(cfg.TITLE_H); vals[41] = num(cfg.TITLE_H * 60);
    vals[71] = '1'; vals[50] = '0.0'; vals[1] = dxfText(txt, host.codepage);
    var out = [mk(0, 'MTEXT')];
    for (var i5 = 1; i5 < tmpl.length; i5++) {
      var p = tmpl[i5];
      if (p.c === 102) { while (i5 < tmpl.length && !(tmpl[i5].c === 102 && tmpl[i5].v.trim() === '}')) i5++; continue; }
      if (DROP[p.c]) continue;
      if (vals.hasOwnProperty(p.c)) {
        if (used[p.c]) continue;                   // διπλός κωδικός -> αγνοείται
        used[p.c] = 1; out.push(mk(p.c, vals[p.c]));
      } else out.push(p);                          // 100 markers, 7 style, 210 κ.λπ.
    }
    /* Κωδικοί που λείπουν από το πρότυπο ΔΕΝ κολλάνε στο τέλος: μπαίνουν
       στην κανονική τους θέση. (Το 30 στο τέλος έκανε αυστηρούς readers
       να απορρίπτουν ΟΛΟ το σχέδιο: "Invalid group code 30".) */
    for (var i6 = 0; i6 < order.length; i6++) {
      var code = order[i6];
      if (used[code] || !ensure[code]) continue;
      var at = -1;
      for (var i8 = i6 - 1; i8 >= 0 && at < 0; i8--) {          // ο προηγούμενος υπαρκτός
        for (var i9 = out.length - 1; i9 >= 0; i9--)
          if (out[i9].c === order[i8]) { at = i9; break; }
      }
      if (at >= 0) out.splice(at + 1, 0, mk(code, vals[code]));
      else out.push(mk(code, vals[code]));
      used[code] = 1;
    }
    return out;
  }

  /* ---- entities: μετατόπιση ---- */
  var outEnts = [], ext = { minx: Infinity, miny: Infinity, maxx: -Infinity, maxy: -Infinity };
  var nIn = 0, nOut = 0;
  for (var m = 0; m < recs.length; m++) {
    var R = recs[m], F = R.f;
    var isDet = R.kind === 'detail';
    var pick = (isDet ? F.secBox : F.structBox) || F.allBox;
    if (!pick) throw new Error('Δεν βρέθηκε γεωμετρία στο ' + R.name);
    if (isDet ? !F.secBox : !F.structBox)
      warnings.push('Στο ' + R.name + ' δεν βρέθηκαν blocks ' + (isDet ? 'διατομών' : 'φέροντος σκελετού') +
                    ' - το pick point υπολογίστηκε από όλη τη γεωμετρία.');
    var ty = (R.level - cfg.BASE_LEVEL) * cfg.STEP_Y;
    var bx = isDet ? cfg.DETAIL_BASE_X : cfg.BASE_X;
    var dx = (isDet ? bx - pick.minx : bx - pick.maxx), dy = ty - pick.miny;
    var hasRen2 = Object.keys(R.rename).length > 0;

    for (var n = 0; n < F.ents.length; n++) {
      var ch2 = F.ents[n];
      if (ch2[0].v.trim() === 'ENDSEC' || ch2[0].v.trim() === 'EOF') continue;
      nIn++;
      var mv = translateChunk(fixH(R, hasRen2 ? applyRename(ch2, R.rename) : ch2), dx, dy);
      push_(outEnts, mv); nOut++;
    }
    var hgt = (F.allBox ? F.allBox.maxy - F.allBox.miny : 0);
    if (hgt > cfg.STEP_Y)
      warnings.push('Το ' + R.name + ' έχει ύψος ' + hgt.toFixed(1) + ' > βήμα ' + cfg.STEP_Y +
                    ' - θα ακουμπήσει την επόμενη στάθμη.');
    var ab = F.allBox || pick;
    if (ab.minx + dx < ext.minx) ext.minx = ab.minx + dx;
    if (ab.miny + dy < ext.miny) ext.miny = ab.miny + dy;
    if (ab.maxx + dx > ext.maxx) ext.maxx = ab.maxx + dx;
    if (ab.maxy + dy > ext.maxy) ext.maxy = ab.maxy + dy;

    var title = levelTitle(R.level, levels);
    if (cfg.TITLE && !isDet) {
      var tx = pick.minx + dx, tyy = ty + cfg.TITLE_DY;
      push_(outEnts, titleEntity(tx, tyy, title));
      if (tyy - cfg.TITLE_H < ext.miny) ext.miny = tyy - cfg.TITLE_H;
    }
    report.push({
      file: R.name, level: R.level, title: isDet ? '(λεπτομέρειες)' : title, kind: R.kind,
      pick: [isDet ? pick.minx : pick.maxx, pick.miny], target: [bx, ty],
      delta: [dx, dy], entities: F.ents.length, blocks: F.blocks.length
    });
  }

  /* ---- header ---- */
  var head = recs[0].f.header.slice();
  function setVar(name, x, y) {
    for (var i4 = 0; i4 + 3 < head.length; i4++) {
      if (head[i4].c === 9 && head[i4].v.trim() === name) {
        for (var j4 = i4 + 1; j4 < Math.min(i4 + 5, head.length); j4++) {
          if (head[j4].c === 10) head[j4] = mk(10, num(x));
          if (head[j4].c === 20) head[j4] = mk(20, num(y));
        }
        return true;
      }
    }
    return false;
  }
  var pad = 5.0;
  setVar('$EXTMIN', ext.minx - pad, ext.miny - pad);
  setVar('$EXTMAX', ext.maxx + pad, ext.maxy + pad);
  setVar('$LIMMIN', ext.minx - pad, ext.miny - pad);
  setVar('$LIMMAX', ext.maxx + pad, ext.maxy + pad);

  /* ---- συναρμολόγηση ---- */
  var P = [];
  function section(name, body) {
    P.push(mk(0, 'SECTION')); P.push(mk(2, name));
    push_(P, body);
    P.push(mk(0, 'ENDSEC'));
  }
  if (hasH) {
    for (var hs = 0; hs + 1 < head.length; hs++)
      if (head[hs].c === 9 && head[hs].v.trim() === '$HANDSEED' && head[hs + 1].c === 5)
        head[hs + 1] = mk(5, (nextH + 16).toString(16).toUpperCase());
  }
  section('HEADER', head);
  if (host.sec.CLASSES && host.sec.CLASSES.length) section('CLASSES', host.sec.CLASSES);
  var tbody = [];
  for (var w = 0; w < outTabs.length; w++) {
    var TB = outTabs[w], hd = TB.head.slice();
    for (var w2 = 1; w2 < hd.length; w2++) if (hd[w2].c === 70) hd[w2] = mk(70, TB.entries.length);
    push_(tbody, hd);
    for (var w3 = 0; w3 < TB.entries.length; w3++) push_(tbody, TB.entries[w3].chunk);
    tbody.push(mk(0, 'ENDTAB'));
  }
  section('TABLES', tbody);
  section('BLOCKS', outBlocks);
  section('ENTITIES', outEnts);
  if (host.sec.OBJECTS && host.sec.OBJECTS.length) section('OBJECTS', host.sec.OBJECTS);
  P.push(mk(0, 'EOF'));

  return {
    text: ser(P, recs[0].f.eol),
    report: report, warnings: warnings,
    counts: { entitiesIn: nIn, entitiesOut: nOut, levels: levels.length,
              plans: recs.filter(function (r) { return r.kind !== 'detail'; }).length,
              details: recs.filter(function (r) { return r.kind === 'detail'; }).length,
              titles: cfg.TITLE ? recs.filter(function (r) { return r.kind !== 'detail'; }).length : 0 },
    extents: ext
  };
}

var API = {
  CFG: CFG, mergeLevels: mergeLevels, parseLevelName: parseLevelName,
  layoutSections: layoutSections, dimensionPlan: dimensionPlan, copyStructure: copyStructure,
  levelTitle: levelTitle, bytesToStr: bytesToStr, strToBytes: strToBytes,
  _analyze: analyzeFile, _parsePairs: parsePairs, _splitSections: splitSections,
  _chunkByZero: chunkByZero
};
if (typeof module !== 'undefined' && module.exports) module.exports = API;
global.DXFMERGE = API;
})(typeof self !== 'undefined' ? self : this);
