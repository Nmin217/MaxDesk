function runAdv(seedv,ux,uy,events,dtf,warpN,tmax){
  __seed=seedv;reset();user.x=ux;user.y=uy;
  const dt0=1/60;let t=0,hits={rect:0,box:0,person:0},first=null,inside=false,arrived=null,maxV=0,ev=events.slice().sort((a,b)=>a[0]-b[0]);
  active=true;rp=0;
  while(t<tmax){
    const dt=Math.min(0.05,dt0*dtf(t));
    while(ev.length&&t>=ev[0][0]){els[ev.shift()[1]].onclick()}
    for(let w=0;w<warpN;w++){update(dt);t+=dt;
      const c=trueCollision();
      if(c.length&&!inside){inside=true;for(const k of new Set(c))hits[k]++;if(!first)first={t:+t.toFixed(1),what:c[0],x:Math.round(T.x),y:Math.round(T.y),v:Math.round(Math.hypot(T.vx,T.vy))}}
      else if(!c.length)inside=false;
      maxV=Math.max(maxV,Math.hypot(T.vx,T.vy));
    }
    if(!active&&parkTh===null&&/^(Arrived|Parking|Ready)/.test($('st').textContent)){arrived=+t.toFixed(1);break}
  }
  return {arrived,hits,first,maxV:Math.round(maxV),st:$('st').textContent};
}
const tg=[[700,430],[90,440],[720,50],[400,450],[740,230]];
const evSets=[[],[[10,'obs']],[[12,'obs'],[16,'obs']],[[11,'walk']],[[11,'sit']],[[11,'talk']],[[9,'obs'],[11,'sit'],[13,'walk']]];
const cfgs={'60':['60fps x1',()=>1,1],'20x4':['20fps x4',()=>3,4]};
const [cn,dtf,wn]=cfgs[process.env.CFG||'60'];
let tot=0,crash=0,arr=0;const ex=[],na=[];let vmax=0;
for(let ti=+process.env.T0;ti<=+process.env.T1;ti++)for(let ei=0;ei<evSets.length;ei++){
  const r=runAdv(100+ti*10+ei,tg[ti][0],tg[ti][1],evSets[ei],dtf,wn,100);tot++;
  const nc=r.hits.rect+r.hits.box+r.hits.person; vmax=Math.max(vmax,r.maxV);
  if(nc){crash++;ex.push({tgt:tg[ti],ev:JSON.stringify(evSets[ei]),first:r.first,hits:r.hits})}
  if(r.arrived)arr++;else na.push({tgt:tg[ti],ev:JSON.stringify(evSets[ei]),st:r.st});
  console.log('run',tot,JSON.stringify({tgt:tg[ti],ev:evSets[ei].length,arrived:r.arrived,hits:r.hits}));
}
console.log('SUMMARY',cn,JSON.stringify({runs:tot,withCollision:crash,arrived:arr,maxSpeed:vmax}));console.log('COLL',JSON.stringify(ex));console.log('NOARR',JSON.stringify(na));
