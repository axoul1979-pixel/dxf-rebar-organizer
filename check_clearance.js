/* ΑΝΕΞΑΡΤΗΤΟΣ έλεγχος: ελάχιστη απόσταση κάθε οντότητας διαστάσεων από
   κάθε άλλη οντότητα του σχεδίου (κουτί προς κουτί). */
const fs=require('fs'), M=require('../src/merge_dxf.js');
const H=(a,b)=>Math.max(a.mnx-b.mxx, b.mnx-a.mxx, 0);
const V=(a,b)=>Math.max(a.mny-b.mxy, b.mny-a.mxy, 0);
const gap=(a,b)=>Math.hypot(H(a,b),V(a,b));
function boxesOf(text, DIMLAY){
  const f=M._analyze(text), dim=[], other=[];
  const wid=(s,h)=>{let w=0;const t=String(s).replace(/\\U\+([0-9A-Fa-f]{4})/g,'x').replace(/\\[A-Za-z](\{[^}]*\}|[^\\;]*;)?/g,'').replace(/\\P/g,' ').replace(/[{}]/g,'');
    for(const c of t) w+= ' .,\''.includes(c)?0.38:('iIl|'.includes(c)?0.45:((c>='A'&&c<='Z')||(c>='0'&&c<='9')||(c>='\u0386'&&c<='\u03AB')?0.94:0.83));
    return Math.max(w*h,h*0.5)+0.25*h;};
  const scan=(ch,px,py,sx,sy,dst)=>{
    const t=ch[0].v.trim(); const g=k=>{const p=ch.find(q=>q.c===k);return p?parseFloat(p.v):NaN;};
    let B={mnx:1e18,mny:1e18,mxx:-1e18,mxy:-1e18};
    const put=(x,y)=>{if(isNaN(x)||isNaN(y))return;B.mnx=Math.min(B.mnx,x);B.mxx=Math.max(B.mxx,x);B.mny=Math.min(B.mny,y);B.mxy=Math.max(B.mxy,y);};
    if(t==='MTEXT'||t==='TEXT'){
      const x=px+sx*g(10), y=py+sy*g(20), h=(g(40)||0.1)*sy, w=wid((ch.find(p=>p.c===1)||{v:''}).v,h);
      const rot=g(50)||0, a=parseInt((ch.find(p=>p.c===71)||{v:'1'}).v,10)||1;
      const x0=(a%3===1)?0:(a%3===2?-w/2:-w), y1=(a<=3)?-h*1.25:(a<=6?-h*0.625:0);
      const co=Math.cos(rot*Math.PI/180), si=Math.sin(rot*Math.PI/180);
      [[x0,y1],[x0+w,y1],[x0+w,y1+h*1.25],[x0,y1+h*1.25]].forEach(([a1,b1])=>put(x+a1*co-b1*si,y+a1*si+b1*co));
    } else {
      let first=-1;
      for(let k=1;k<ch.length-1;k++){const c=ch[k].c;
        if(c<10||c>13||ch[k+1].c!==c+10)continue;
        if(t==='HATCH'&&first<0){first=k;continue;}
        put(px+sx*parseFloat(ch[k].v), py+sy*parseFloat(ch[k+1].v));}
    }
    if(B.mnx<1e17) dst.push(B);
  };
  f.blocks.forEach(b=>{ const pos=f.inserts[b.name]; if(!pos)return;
    pos.forEach(([px,py,sx,sy])=>b.body.forEach(ch=>scan(ch,px-(sx||1)*b.bx,py-(sy||1)*b.by,sx||1,sy||1,other)));});
  f.ents.forEach(ch=>{ if(ch[0].v.trim()==='INSERT')return;
    const l=((ch.find(p=>p.c===8)||{}).v||'').trim();
    scan(ch,0,0,1,1, l===DIMLAY?dim:other);});
  return {dim,other};
}
function worstGap(text,quiet){
  const {dim,other}=boxesOf(text,'DIM_PERIMETER');
  let worst=1e18, pair=null;
  dim.forEach(a=>other.forEach(b=>{const d=gap(a,b); if(d<worst){worst=d;pair=[a,b];}}));
  if(!quiet) console.log('  διαστάσεις:',dim.length,'| άλλα:',other.length,'| ΕΛΑΧΙΣΤΗ ΑΠΟΣΤΑΣΗ:',worst.toFixed(3));
  return worst;
};
module.exports=worstGap; module.exports.boxesOf=boxesOf; module.exports.gap=gap;
/* node tests/check_clearance.js <φάκελος>  →  ελέγχει κάθε ξυλότυπο */
if(require.main===module){
  const dir=process.argv[2];
  if(!dir){console.error('Χρήση: node tests/check_clearance.js <φάκελος με τα DXF>');process.exit(2);}
  let bad=0;
  require('fs').readdirSync(dir).filter(n=>/\.dxf$/i.test(n)).forEach(n=>{
    const lv=M.parseLevelName(n); if(!lv||lv.kind!=='plan')return;
    const r=M.dimensionPlan(require('fs').readFileSync(require('path').join(dir,n),'latin1'));
    if(!r.added){console.log('  '+n.padEnd(30)+'— χωρίς διαστάσεις');return;}
    const w=worstGap(r.text,true), ok=w>=M.CFG.DIM_SAFE-1e-3;
    if(!ok)bad++;
    console.log('  '+n.padEnd(30)+'ελάχιστη απόσταση '+w.toFixed(3)+(ok?'  ✓':'  ✗'));
  });
  process.exit(bad?1:0);
}
