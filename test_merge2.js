const FIX=require('path').join(__dirname,'fixtures');
const fs=require('fs'), M=require('../src/merge_dxf.js');
const dir=FIX+'/r12_text';
const items=fs.readdirSync(dir).filter(n=>n.endsWith('.dxf')).sort().map(n=>({name:n,text:fs.readFileSync(dir+'/'+n,'latin1')}));
const res=M.mergeLevels(items);
fs.writeFileSync(dir+'/MERGED.dxf',res.text,'latin1');
console.log('warnings:',res.warnings.length, JSON.stringify(res.counts));
