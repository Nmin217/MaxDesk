function setUI(manual,interlock){$('dm').value=manual?'manual':'auto';$('il').checked=!!interlock}
function runSec(sec,st){const dt=1/60;let t=0;while(t<sec){update(dt);t+=dt;if(st){const c=trueCollision();if(c.length&&!st.inside){st.inside=true;st.coll++}else if(!c.length)st.inside=false}}}
function until(cond,max,st){const dt=1/60;let t=0;while(t<max){update(dt);t+=dt;const c=trueCollision();if(c.length&&!st.inside){st.inside=true;st.coll++}else if(!c.length)st.inside=false;if(cond())return t}return null}
const READY=()=>!active&&parkTh===null&&/^Ready/.test($('st').textContent);
function speed(){return Math.hypot(T.vx,T.vy)}
console.log('A) plug in DURING a trip, interlock ON');
__seed=4;reset();user.x=700;user.y=430;setUI(true,true);let st={coll:0,inside:false};
els['go'].onclick();runSec(1.2,st);
const sp0=speed(),d0=Math.hypot(T.x-HOME.x,T.y-HOME.y);
els['cord'].onclick();runSec(0.1,st);
console.log('   before plugging: speed',sp0.toFixed(1),'px/s, ',d0.toFixed(0),'px from the dock | cord plugged:',cordIn);
runSec(1.0,st);
console.log('   after plugging: active',active,'| speed',speed().toFixed(2),'| status:',$('st').textContent.slice(0,55),'| alert',alertT>0);
runSec(4,st);console.log('   4 s later: still stopped? speed',speed().toFixed(2),'| moved only',Math.hypot(T.x-HOME.x,T.y-HOME.y).toFixed(0),'px from dock | charging:',charging,'| breakaways',breakaways,'| collisions',st.coll);
els['go'].onclick();update(1/60);console.log('   Come to me while plugged:',active?'MOVED (bad)':'refused');
els['cord'].onclick();els['go'].onclick();const t=until(READY,150,st);
console.log('   unplug, then Come to me -> arrived in',t&&t.toFixed(1),'s | collisions',st.coll);
console.log('B) same, interlock OFF (second layer only): drives on and the breakaway releases');
__seed=4;reset();user.x=700;user.y=430;setUI(true,false);st={coll:0,inside:false};
els['go'].onclick();runSec(1.2,st);els['cord'].onclick();const t2=until(READY,150,st);
console.log('   trip completed in',t2&&t2.toFixed(1),'s | breakaways',breakaways,'| collisions',st.coll);
console.log('C) AUTO contacts mode: charging must not stop it leaving');
__seed=4;reset();user.x=700;user.y=430;setUI(false,true);st={coll:0,inside:false};
bat.r=60;runSec(3,st);const chg=charging;els['go'].onclick();const t3=until(READY,150,st);
console.log('   charging before leaving:',chg,'| trip completed in',t3&&t3.toFixed(1),'s | collisions',st.coll);
