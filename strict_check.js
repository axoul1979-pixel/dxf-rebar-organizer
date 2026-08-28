/* ΑΥΣΤΗΡΟΣ έλεγχος: κάθε chunk του τελικού πρέπει να έχει ακολουθία group codes
   που υπάρχει στα πηγαία αρχεία (εξαιρούνται οι τίτλοι στάθμης). */
const fs=require('fs'), M=require('../src/merge_dxf.js');
module.exports=function(mergedText, srcTexts, quiet){
  const seq=new Set();
  // γνωστά-καλά πρότυπα που ΕΜΕΙΣ προσθέτουμε σκόπιμα (κανονική σειρά DXF)
  seq.add('INSERT:0,5,100,8,100,2,10,20,30,41,42,43');
  seq.add('INSERT:0,5,330,100,8,100,2,10,20,30,41,42,43');
  srcTexts.forEach(t=>{
    const f=M._analyze(t);
    const add=c=>seq.add(c[0].v.trim()+':'+c.map(p=>p.c).join(','));
    f.ents.forEach(add);
    f.blocks.forEach(b=>{b.body.forEach(add); add(b.head); add(b.end);});
    f.tables.forEach(t2=>t2.entries.forEach(e=>seq.add(t2.type+':'+e.chunk.map(p=>p.c).join(','))));
  });
  const m=M._analyze(mergedText);
  const odd=[];
  const byTag={};
  [...seq].forEach(k=>{const i=k.indexOf(':'); (byTag[k.slice(0,i)]=byTag[k.slice(0,i)]||[]).push(k.slice(i+1).split(',').map(Number));});
  const isSub=(a,b)=>{let j=0; for(const x of b){ if(j<a.length&&a[j]===x)j++; } return j===a.length;};
  const chk=(c,tag)=>{
    const t=tag||c[0].v.trim(), codes=c.map(p=>p.c), k=t+':'+codes.join(',');
    if(seq.has(k))return;
    // αποδεκτό αν είναι ΥΠΑΚΟΛΟΥΘΙΑ πηγαίας (δηλ. μόνο αφαιρέσαμε προαιρετικούς κωδικούς)
    if((byTag[t]||[]).some(src=>isSub(codes,src)))return;
    odd.push(k);
  };
  m.ents.forEach(c=>{ if(((c.find(p=>p.c===8)||{}).v||'')!=='LEVEL_TITLE') chk(c); });
  m.blocks.forEach(b=>{b.body.forEach(c=>chk(c)); chk(b.head); chk(b.end);});
  m.tables.forEach(t2=>t2.entries.forEach(e=>{ if(e.name!=='LEVEL_TITLE') chk(e.chunk,t2.type); }));
  // υποχρεωτικά πεδία
  const lay=m.tables.find(t=>t.type==='LAYER');
  const noPS=lay.entries.filter(e=>!e.chunk.some(p=>p.c===390)).map(e=>e.name);
  const noOwner=[];
  // handles
  const H={},dup=[];
  m.pairs.forEach(p=>{if(p.c===5||p.c===105){const v=p.v.trim().toUpperCase(); if(H[v])dup.push(v); H[v]=1;}});
  // κρεμασμένες αναφορές
  const dangling=[];
  m.pairs.forEach(p=>{ if([330,340,347,350,360,390].includes(p.c)){const v=p.v.trim().toUpperCase(); if(v!=='0'&&!H[v])dangling.push(p.c+'→'+v);} });
  if(!quiet){
    console.log('  ακολουθίες κωδικών εκτός πηγής:', odd.length, [...new Set(odd)].slice(0,4));
    console.log('  LAYER χωρίς 390 (PlotStyleName):', noPS.length, noPS.slice(0,5));
    console.log('  LAYER χωρίς 330 (owner):', noOwner.length, noOwner.slice(0,5));
    console.log('  διπλά handles:', dup.length, dup.slice(0,4));
    console.log('  κρεμασμένες αναφορές:', dangling.length, [...new Set(dangling)].slice(0,6));
  }
  return odd.length+noPS.length+noOwner.length+dup.length+dangling.length;
};
