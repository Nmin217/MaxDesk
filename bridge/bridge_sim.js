// Serves the REAL simulator (the same code as the browser page) over HTTP so the iPhone app can be developed
// without hardware.  node bridge_sim.js   (Node 18+, no dependencies)
const fs=require('fs'),path=require('path');
global.els={};
const mkctx=()=>new Proxy({},{get:(t,p)=>p in t?t[p]:(()=>{}),set:(t,p,v)=>{t[p]=v;return true}});
function el(id){return els[id]||(els[id]={id,width:1600,height:1000,style:{},dataset:{},value:'1',checked:false,textContent:'',className:'',
  getContext:()=>mkctx(),addEventListener(){},appendChild(){},setPointerCapture(){},getBoundingClientRect:()=>({left:0,top:0,width:800,height:500})})}
global.document={getElementById:el,documentElement:{},querySelectorAll:()=>[]};
global.getComputedStyle=()=>({getPropertyValue:()=>'#888'});
global.matchMedia=()=>({matches:false,addEventListener(){}});
global.addEventListener=()=>{};global.devicePixelRatio=1;
global.performance={now:()=>0};global.requestAnimationFrame=()=>{};
global.localStorage={getItem:()=>null,setItem(){},removeItem(){}};
const code=fs.readFileSync(path.join(__dirname,'sim_core.js'),'utf8')+'\n'+fs.readFileSync(path.join(__dirname,'bridge_core.js'),'utf8');
eval(code);
