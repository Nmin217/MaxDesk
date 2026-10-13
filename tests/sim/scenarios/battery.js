function run(sec,st){const dt=1/60;let t=0;while(t<sec){update(dt);t+=dt;const c=trueCollision();if(c.length&&!st.inside){st.inside=true;st.coll++}else if(!c.length)st.inside=false}}
function until(cond,max,st){const dt=1/60;let t=0;while(t<max){update(dt);t+=dt;const c=trueCollision();if(c.length&&!st.inside){st.inside=true;st.coll++}else if(!c.length)st.inside=false;if(cond())return t}return null}
const st={coll:0,inside:false};
__seed=9;reset();user.x=700;user.y=430;
console.log('1. starts docked:',charging?'charging':'NOT charging','| dock text:',$('dk').textContent||'(n/a until draw)');
const r0=bat.r,a0=bat.a;run(6,st);
console.log('   after 6 s on the pad: robot',r0.toFixed(1),'->',bat.r.toFixed(1),'| ups',a0.toFixed(1),'->',bat.a.toFixed(1),'(should rise)');
bat.r=60;bat.a=55;
els['go'].onclick();
const tUser=until(()=>!active&&parkTh===null&&Math.abs(userH-deskH)<1e-4&&/^Ready/.test($('st').textContent),150,st);
console.log('2. trip to user took',tUser&&tUser.toFixed(1),'s | robot 60 ->',bat.r.toFixed(1),'| ups 55 ->',bat.a.toFixed(1),'(should drop) | charging while away:',charging);
bat.r=22;                                   // low while idle at the user
const tLow=until(()=>mode==='home'&&active,20,st);
console.log('3. low battery (22%) idle -> auto return started after',tLow&&tLow.toFixed(1),'s | status:',$('st').textContent.slice(0,48));
const tDock=until(()=>charging,150,st);
console.log('   docked and charging after',tDock&&tDock.toFixed(1),'s | pos',Math.round(T.x),Math.round(T.y),'| robot',bat.r.toFixed(1));
run(15,st);console.log('   15 s later: robot',bat.r.toFixed(1),'| ups',bat.a.toFixed(1),'| status:',$('st').textContent.slice(0,48));
// very low battery -> Come to me refuses and docks
reset();user.x=700;user.y=430;els['go'].onclick();
until(()=>!active&&parkTh===null&&Math.abs(userH-deskH)<1e-4&&/^Ready/.test($('st').textContent),150,st);
bat.r=5;els['go'].onclick();
console.log('4. battery 5% + Come to me -> mode',mode,'active',active,'| status:',$('st').textContent.slice(0,52));
console.log('collisions during all of the above:',st.coll);
