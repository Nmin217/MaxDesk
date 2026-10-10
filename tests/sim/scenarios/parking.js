function parkTest(ux,uy){
  __seed=21;reset();user.x=ux;user.y=uy;active=true;rp=0;
  const dt=1/60;let t=0,coll=0,inside=false,doneAt=null,arrivedAt=null;
  while(t<150){update(dt);t+=dt;
    const c=trueCollision();if(c.length&&!inside){inside=true;coll++}else if(!c.length)inside=false;
    if(!active&&arrivedAt===null&&$('st').textContent.length)arrivedAt=t;
    if(!active&&parkTh===null&&arrivedAt!==null&&t>arrivedAt+0.1){doneAt=t;break}
  }
  if(doneAt===null)return {user:[ux,uy],result:'did not finish',status:$('st').textContent.slice(0,45),collisions:coll};
  const long=Math.round(((T.th*180/Math.PI)%180+180)%180),horiz=Math.min(long,180-long);
  const toUser=Math.atan2(uy-T.y,ux-T.x),kb=T.th+Math.PI/2;           // keyboard-side direction
  let d=Math.abs(((kb-toUser+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI)*180/Math.PI;
  return {user:[ux,uy],doneAt:+doneAt.toFixed(1),deskPos:[Math.round(T.x),Math.round(T.y)],tableOffHorizontalDeg:horiz,keyboardFacesUserWithinDeg:Math.round(d),collisions:coll};
}
for(const [x,y] of [[700,430],[400,450],[740,230],[120,250],[720,50],[90,440]])console.log(JSON.stringify(parkTest(x,y)));
