if(typeof clearMapArrays==='function'){const _rs=reset;reset=function(){_rs();clearMapArrays()}}
function polyOBB(){const c=Math.cos(T.th),s=Math.sin(T.th);return [[-HW,-HH],[HW,-HH],[HW,HH],[-HW,HH]].map(([x,y])=>[T.x+c*x-s*y,T.y+s*x+c*y])}
function sat(A,B){for(const P of[A,B])for(let i=0;i<P.length;i++){const a=P[i],b=P[(i+1)%P.length];const nx=-(b[1]-a[1]),ny=b[0]-a[0];
 let mn1=1e9,mx1=-1e9,mn2=1e9,mx2=-1e9;for(const p of A){const d=p[0]*nx+p[1]*ny;mn1=Math.min(mn1,d);mx1=Math.max(mx1,d)}
 for(const p of B){const d=p[0]*nx+p[1]*ny;mn2=Math.min(mn2,d);mx2=Math.max(mx2,d)}if(mx1<mn2||mx2<mn1)return false}return true}
function circHit(cx,cy,r){const c=Math.cos(-T.th),s=Math.sin(-T.th),dx=cx-T.x,dy=cy-T.y;const lx=c*dx-s*dy,ly=s*dx+c*dy;
 const qx=Math.max(-HW,Math.min(HW,lx)),qy=Math.max(-HH,Math.min(HH,ly));return Math.hypot(lx-qx,ly-qy)<r}
function trueCollision(){const d=polyOBB();const out=[];
 for(const r of rects)if(sat(d,[[r.x,r.y],[r.x+r.w,r.y],[r.x+r.w,r.y+r.h],[r.x,r.y+r.h]]))out.push('rect');
 for(const c of circs)if(circHit(c.x,c.y,c.r))out.push('box');
 for(const p of people)if(circHit(p.x,p.y,p.r))out.push('person');
 return out}
