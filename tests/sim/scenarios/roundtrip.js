function runUntil(cond,tmax,st){
  const dt=1/60;let t=0;
  while(t<tmax){update(dt);t+=dt;
    const c=trueCollision();if(c.length&&!st.inside){st.inside=true;st.coll++}else if(!c.length)st.inside=false;
    if(cond())return t}
  return null;
}
function round(name,ux,uy,auto){
  __seed=31;reset();user.x=ux;user.y=uy;
  $('ar').checked=!!auto;
  const st={coll:0,inside:false};
  els['go'].onclick();
  const ready=()=>!active&&parkTh===null&&Math.abs(userH-deskH)<1e-4&&/^Ready/.test($('st').textContent);
  const t1=runUntil(ready,150,st);
  if(t1===null)return {name,result:'never reached you',status:$('st').textContent.slice(0,50),collisions:st.coll};
  const atUser={pos:[Math.round(T.x),Math.round(T.y)],status:$('st').textContent};
  let t2;
  if(auto){ // wait for the automatic return to kick in, then for it to finish
    const back=()=>mode==='home'&&!active&&parkTh===null&&Math.abs(userH-deskH)<1e-4&&/^Ready - (back|docked)/.test($('st').textContent);
    t2=runUntil(back,200,st);
  }else{
    els['back'].onclick();
    const back=()=>!active&&parkTh===null&&Math.abs(userH-deskH)<1e-4&&/^Ready - (back|docked)/.test($('st').textContent);
    t2=runUntil(back,150,st);
  }
  if(t2===null)return {name,result:'did not get back',status:$('st').textContent.slice(0,60),pos:[Math.round(T.x),Math.round(T.y)],collisions:st.coll};
  const th=((T.th%(2*Math.PI))+3*Math.PI)%(2*Math.PI)-Math.PI;       // -PI..PI, 0 = monitor back pointing up the screen
  return {name,auto:!!auto,toYouSec:+t1.toFixed(1),backSec:+t2.toFixed(1),homeErrPx:+Math.hypot(T.x-HOME.x,T.y-HOME.y).toFixed(1),
    headingErrDeg:Math.round(Math.abs(th)*180/Math.PI),deskCm:Math.round(deskH*100),collisions:st.coll};
}
for(const [n,x,y,a] of [['default',700,430,0],['right-mid',740,230,0]])
  console.log(JSON.stringify(round(n,x,y,a)));
