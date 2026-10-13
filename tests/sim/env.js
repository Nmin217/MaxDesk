const fs=require('fs');
global.__seed=12345;
global.rnd=()=>{__seed=(__seed*1664525+1013904223)%4294967296;return __seed/4294967296};
Math.random=rnd;
const els={};global.els=els;
const mkctx=()=>new Proxy({}, {get:(t,p)=>p in t?t[p]:(()=>{}), set:(t,p,v)=>{t[p]=v;return true}});
function el(id){return els[id]||(els[id]={id,width:1600,height:1000,style:{},dataset:{},value:'1',checked:false,textContent:'',className:'',
 getContext:()=>mkctx(),addEventListener(){},appendChild(){},setPointerCapture(){},getBoundingClientRect:()=>({left:0,top:0,width:800,height:500})})}
global.document={getElementById:el,documentElement:{},querySelectorAll:()=>[]};
global.getComputedStyle=()=>({getPropertyValue:()=>'#888'});
global.matchMedia=()=>({matches:false,addEventListener(){}});
global.addEventListener=()=>{};global.devicePixelRatio=1;
global.performance={now:()=>0};global.requestAnimationFrame=()=>{};
global.THREE=undefined;
global.runWith=(simFile,libFile)=>{ (0,eval)(''); const code=fs.readFileSync(simFile,'utf8')+'\n'+fs.readFileSync(libFile,'utf8'); eval(code); };

global.__persist=false;const __st={};global.localStorage={getItem:k=>__persist&&k in __st?__st[k]:null,setItem:(k,v)=>{if(__persist)__st[k]=String(v)},removeItem:k=>{delete __st[k]}};
