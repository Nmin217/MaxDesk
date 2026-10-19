function gapTest(name,gapW,startX,startY,ux,uy,horizontalWall){
  __seed=5;reset();
  rects.length=0;circs.length=0;people.length=0;
  if(horizontalWall){const gx=400-gapW/2;rects.push({x:0,y:240,w:gx,h:20},{x:gx+gapW,y:240,w:800-gx-gapW,h:20})}
  else{const gy=250-gapW/2;rects.push({x:380,y:0,w:20,h:gy},{x:380,y:gy+gapW,w:20,h:500-gy-gapW})}
  T.x=startX;T.y=startY;T.th=0.3;user.x=ux;user.y=uy;active=true;rp=0;
  const dt=1/60;let t=0,coll=0,inside=false,arrived=null,maxPlanMs=0,thAtGap=null,rotMsgs=0;
  while(t<140){
    const t0=Date.now();update(dt);t+=dt;
    if(Date.now()-t0>maxPlanMs)maxPlanMs=Date.now()-t0;
    const c=trueCollision();if(c.length&&!inside){inside=true;coll++}else if(!c.length)inside=false;
    if(horizontalWall&&Math.abs(T.y-250)<12&&Math.abs(T.x-400)<gapW/2+5&&thAtGap===null)thAtGap=T.th;
    if(!horizontalWall&&Math.abs(T.x-390)<12&&Math.abs(T.y-250)<gapW/2+5&&thAtGap===null)thAtGap=T.th;
    if($('st').textContent.startsWith('Rotating'))rotMsgs++;
    if(!active&&parkTh===null&&/^Ready/.test($('st').textContent)){arrived=+t.toFixed(1);break}
  }
  const deg=thAtGap===null?null:Math.round(((thAtGap*180/Math.PI)%180+180)%180);
  return {name,gapW,arrived,collisions:coll,orientationAtGapDeg:deg,status:$('st').textContent.slice(0,40),slowestUpdateMs:maxPlanMs};
}
const out=[];
out.push(gapTest('vertical slot, travel down',70,80,90,700,430,true));
out.push(gapTest('vertical slot, narrower',62,80,90,700,430,true));
out.push(gapTest('horizontal slot, travel right',70,80,250,700,250,false));
console.log(out.map(o=>JSON.stringify(o)).join('\n'));
