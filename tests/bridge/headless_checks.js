// In-process checks of bridge_core.js: loads bridge_sim.js with test code appended to its eval'd source.
//   PORT=8796 node tests/bridge/headless_checks.js     prints one {"check","ok"} line per check, exits 1 on any failure
const fs=require('fs'),path=require('path');
const dir=path.join(__dirname,'..','..','bridge');
const extra=`
setTimeout(()=>{
  const out=[],add=(check,ok)=>out.push({check,ok:!!ok});
  const lb=typeof isLoopback==='function';
  add('bridge_loopback_predicate',lb&&isLoopback('127.0.0.1')&&isLoopback('::1')&&isLoopback('::ffff:127.0.0.1')&&
    !isLoopback('192.168.1.20')&&!isLoopback('::ffff:10.0.0.5')&&!isLoopback('fe80::1')&&!isLoopback(undefined)&&!isLoopback('127.0.0.1.evil'));
  // Low battery: a headless summon must be redirected to the dock, as the simulator's Come-to-me button does.
  estop=false;cordIn=false;active=false;path=[];bat.r=5;mode='user';
  pendingCommands.push({id:9001,type:'summon',x:14,y:8.6,epoch});drainHeadless();
  add('bridge_headless_summon_low_battery',mode==='home');
  // E-stop: a headless summon must not start a trip.
  active=false;path=[];bat.r=80;estop=true;
  pendingCommands.push({id:9002,type:'summon',x:14,y:8.6,epoch});drainHeadless();
  add('bridge_headless_summon_estop',active===false&&/E-stop/i.test($('st').textContent));
  out.forEach(o=>console.log(JSON.stringify(o)));process.exit(out.every(o=>o.ok)?0:1);
},200);`;
let src=fs.readFileSync(path.join(dir,'bridge_sim.js'),'utf8').replace(/__dirname/g,JSON.stringify(dir));
if(!src.includes('eval(code);'))throw new Error('bridge_sim.js layout changed');
src=src.replace('eval(code);','eval(code+'+JSON.stringify(extra)+');');
eval(src);
