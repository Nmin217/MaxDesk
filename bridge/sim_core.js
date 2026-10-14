
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
const W=800,H=500,CS=10,GW=80,GH=50,TR=34,INF=56,HW=28,HH=18;
const VMAX=40, VSLOW=14, ABRAKE=60;            // VMAX was 60 px/s (~1.2 m/s); 40 px/s is ~0.8 m/s
const SAFE_DEC=45, STOP_GAP=5, SAFE_M=2, INF_T=TR+8, EMERG_DEC=120; // emergency brake 120 px/s^2 (2.4 m/s^2), just under SLIP_A // safety supervisor + relaxed re-plan inflation
const LIDAR_RAYS=90;
const LIDARS=[[HW*0.85, HH*0.85],[-HW*0.85,-HH*0.85]];
const REC_ALIGN_T=1.2, REC_BACKOFF_T=1.2, REC_SPIN_T=2.0;
const STUCK_TRIGGER=1.0;
const YAW_RATE_MAX=1.8, YAW_GAIN=3.2;   // swerve: yaw is independent of translation (was 1.1 rad/s)
const HEADING_FILTER_TC=1.5, HEADING_COMMIT=0.35, HEADING_SETTLE=0.06, SPEED_DRIVE_MIN=5;
const MOVE_THRESH=3, STAND_WAIT=5.0;
const SLIP_A=2.5;
// ---------------------------------------------------------------------------
const rects=[{x:220,y:170,w:40,h:150},{x:450,y:60,w:150,h:40},{x:520,y:300,w:120,h:70}];
const RECTS0=rects.map(r=>({...r}));
let placeBlocks=false, selectedBlock=null, blockDrag=null, roomStyle='office';
const HOME={x:85,y:95};   // starting spot (centre of the grid cell the desk starts in); parks horizontal, monitor-back forward
const circs=[{x:400,y:260,r:18}];
const CIRCS0=circs.map(c=>({...c}));
const offs=[];for(let a=-5;a<=5;a++)for(let b=-5;b<=5;b++)if(a*a+b*b<=INF*INF/CS/CS)offs.push([a,b]);
const offsT=[];for(let a=-5;a<=5;a++)for(let b=-5;b<=5;b++)if(a*a+b*b<=INF_T*INF_T/CS/CS)offsT.push([a,b]);
const last=new Float32Array(GW*GH).fill(-99);
const $=id=>document.getElementById(id);
const since=new Float32Array(GW*GH).fill(-99),ESX=-14,HMIN=.63,LIFT=.05;
// Static map. The old costmap forgot every lidar hit after 1.2 s, so walls beyond lidar range (260 px) vanished and
// the desk would drive off, rediscover them, and come back, forever. Now: a cell seen occupied for >=1 s (and not a
// moving person) is remembered for up to 90 s, and any lidar ray that passes through it clears it.
const smap=new Float32Array(GW*GH); let clrRays=[];
// Long-term memory. smap = static obstacles (kept until a lidar ray passes through the cell). seen = cells the lidar has
// ever covered. traffic = where people tend to be. All three persist between trips and, in this browser, between visits.
const seen=new Uint8Array(GW*GH), traffic=new Float32Array(GW*GH);
let sceneKey='default',mapChanged=false,lastSaveT=0,mapCalls=0,tourI=0,tourT=0,readyPending=false,wasCharging=false,cordIn=false,breakaways=0,alertT=0;
const RISK=new Float32Array(GW*GH), UNK_PEN=.15, TRAF_K=.015, TRAF_MAX=.4;
const TOURS={default:[[160,430],[420,430],[700,420],[730,240],[540,200],[330,140],[330,340],[150,260]],
             slot:[[160,120],[620,120],[400,200],[400,330],[180,430],[650,430]]};
function tourPts(){return TOURS[sceneKey]||TOURS.default}
function dest(){
  if(mode==='home')return HOME;
  if(mode==='tour'){const p=tourPts()[Math.min(tourI,tourPts().length-1)];return{x:p[0],y:p[1]}}
  return user;
}
function mapKey(){return 'summonMap:'+sceneKey}
function clearMapArrays(){smap.fill(0);seen.fill(0);traffic.fill(0);mapChanged=false}
function forgetMap(){clearMapArrays();try{localStorage.removeItem(mapKey())}catch(e){}}
function saveMap(){
  try{
    const n=GW*GH,a=new Uint8Array(3*n);
    for(let i=0;i<n;i++){a[i]=smap[i]>0?1:0;a[n+i]=seen[i];a[2*n+i]=Math.min(255,Math.round(traffic[i]*6))}
    let s='';for(let i=0;i<a.length;i+=4096)s+=String.fromCharCode.apply(null,a.subarray(i,i+4096));
    localStorage.setItem(mapKey(),btoa(s));
  }catch(e){}
}
function loadMap(){
  try{
    const raw=localStorage.getItem(mapKey());if(!raw)return false;
    const s=atob(raw),n=GW*GH;if(s.length!==3*n)return false;
    for(let i=0;i<n;i++){smap[i]=s.charCodeAt(i)?.001:0;seen[i]=s.charCodeAt(n+i);traffic[i]=s.charCodeAt(2*n+i)/6}
    return true;
  }catch(e){return false}
}
let warp=1,peakA=0,deskH=1.05,userH=1.05,estop=false,T,people,path=[],goal=null,hits=[],time=0,active=false,contacts=0,inC=false,minGap=1e9,rp=0,drag=false,parkTh=null,parkT=0,mode='user',didArrive=false,arT=0,bat={r:80,a:70},charging=false,lowT=0,prevDeskH=1.05,trail=[],retreats=0,waitT=0,retreatD=0,noPathT=0,lowSpeedT=0,patientUntil=0,lastRecEnd=-99;
let stuckTime=0,recovery=null,recoveryT=0;
let headTarget=0.3, headState='SETTLED';
// Phone joystick commands are simulation-only; values are normalized to [-1, 1].
let manualDrive={enabled:false,x:0,y:0,turn:0,speed:0.35,lastCommandAt:0};
let peopleInPathCount=0, safetyHolds=0, holdLatch=false, squeeze=false, squeezeUntil=0;
const user={x:700,y:430,r:13};
const modAng=[0,0,0,0];

// --- Person factory functions ---
function mkWalker(x,y){return{x,y,r:13,tx:x,ty:y,t:0,sp:22+Math.random()*10,type:'walker',
  lastX:x,lastY:y,stillTime:0,settled:false,convertOnArrival:false};}
function mkSitter(x,y){return{x,y,r:13,tx:x,ty:y,t:1e9,sp:0,type:'sitter',
  lastX:x,lastY:y,stillTime:999,settled:true};}
function mkTalker(x,y){return{x,y,r:13,tx:x,ty:y,t:1e9,sp:2.5,type:'talker',
  anchorX:x,anchorY:y,jitterPhase:Math.random()*6.28,lastX:x,lastY:y,stillTime:0,settled:false};}

function setSt(t,c){$('st').textContent=t;$('st').className=c||''}

function reset(){
  manualDrive={enabled:false,x:0,y:0,turn:0,speed:0.35};
  rects.length=0;for(const r of RECTS0)rects.push({...r});   // Reset scene also restores the normal room
  selectedBlock=null;blockDrag=null;placeBlocks=false;if($('placeMode'))$('placeMode').textContent='Place blocks: off';
  T={x:HOME.x,y:HOME.y,th:0,vx:0,vy:0,w:0};   // starts docked on the charging pad
  people=[
    // Two people seated at the two "desks" (rects). Stationary, already routable.
    mkSitter(285, 220),
    mkSitter(525, 135),
    // Two people standing and talking. Just started — will settle after 5 s.
    mkTalker(340, 375),
    mkTalker(378, 385),
    // Two people walking around.
    mkWalker(620, 400),
    mkWalker(160, 380),
  ];
  last.fill(-99); since.fill(-99); clrRays=[]; circs.length=0; for(const c of CIRCS0)circs.push({...c}); parkTh=null; mode='user'; didArrive=false; arT=0; readyPending=false; cordIn=false; breakaways=0; alertT=0; bat.r=80; bat.a=70; charging=false; lowT=0; trail=[]; retreats=0; waitT=0; lowSpeedT=0; patientUntil=0; lastRecEnd=-99;
  estop=false; deskH=userH=1.05;
  $('hs').value=105; $('hv').textContent='105';
  path=[]; goal=null; active=false; contacts=0; minGap=1e9; peakA=0;
  user.x=700; user.y=430;
  stuckTime=0; recovery=null; recoveryT=0; safetyHolds=0; holdLatch=false; squeeze=false; squeezeUntil=0;
  headTarget=T.th; headState='SETTLED';
  setSt('Idle',''); $('rc').textContent='–'; $('hg').textContent='–'; $('sl').textContent='–'; $('pp').textContent='–';
}

const ang=(a,b)=>{let d=a-b;while(d>Math.PI)d-=2*Math.PI;while(d<-Math.PI)d+=2*Math.PI;return d};

function hitTestInfo(x,y){
  if(x<0||y<0||x>W||y>H)return {type:'wall'};
  for(const r of rects)if(x>=r.x&&x<=r.x+r.w&&y>=r.y&&y<=r.y+r.h)return {type:'rect'};
  for(const c of circs)if(Math.hypot(x-c.x,y-c.y)<c.r)return {type:'circ'};
  for(const p of people)if(Math.hypot(x-p.x,y-p.y)<p.r)return {type:'person',person:p};
  if(Math.hypot(x-user.x,y-user.y)<user.r)return {type:'user'};
  return null;
}
function hitTest(x,y){return hitTestInfo(x,y)!==null}

function clearance(x,y){let m=1e9;
for(const r of rects){const cx=Math.max(r.x,Math.min(x,r.x+r.w)),cy=Math.max(r.y,Math.min(y,r.y+r.h));m=Math.min(m,Math.hypot(x-cx,y-cy))}
for(const c of circs)m=Math.min(m,Math.hypot(x-c.x,y-c.y)-c.r);
for(const p of people)m=Math.min(m,Math.hypot(x-p.x,y-p.y)-p.r);
return Math.min(m,Math.hypot(x-user.x,y-user.y)-user.r)}

function rayExitBox(ox,oy,dx,dy,hx,hy,lx,ly){
  // distance along (dx,dy) from (ox,oy) to leave the box [lx,hx]x[ly,hy]; default box is centred: [-hx,hx]x[-hy,hy]
  if(lx===undefined){lx=-hx;ly=-hy}
  let t=1e9;
  if(dx>1e-6) t=Math.min(t,(hx-ox)/dx); else if(dx<-1e-6) t=Math.min(t,(lx-ox)/dx);
  if(dy>1e-6) t=Math.min(t,(hy-oy)/dy); else if(dy<-1e-6) t=Math.min(t,(ly-oy)/dy);
  return t;
}

// Analytic lidar: exact ray/shape intersections (the old version stepped 3 px at a time through every object).
function spanCircle(ox,oy,c,s,cx,cy,r){
  const dx=cx-ox,dy=cy-oy,b=dx*c+dy*s,cc=dx*dx+dy*dy-r*r,disc=b*b-cc;
  if(disc<0)return null;const q=Math.sqrt(disc);return [b-q,b+q];
}
function spanRect(ox,oy,c,s,r){
  let t0=-1e9,t1=1e9;
  if(Math.abs(c)<1e-9){if(ox<r.x||ox>r.x+r.w)return null}
  else{let a=(r.x-ox)/c,b=(r.x+r.w-ox)/c;if(a>b){const q=a;a=b;b=q}t0=Math.max(t0,a);t1=Math.min(t1,b)}
  if(Math.abs(s)<1e-9){if(oy<r.y||oy>r.y+r.h)return null}
  else{let a=(r.y-oy)/s,b=(r.y+r.h-oy)/s;if(a>b){const q=a;a=b;b=q}t0=Math.max(t0,a);t1=Math.min(t1,b)}
  return t0>t1?null:[t0,t1];
}
function mapUpdate(){
  for(const[ox,oy,c,s,d0,d1]of clrRays)for(let d=d0;d<d1;d+=5){
    const gx=(ox+c*d)/CS|0,gy=(oy+s*d)/CS|0;
    if(gx>=0&&gx<GW&&gy>=0&&gy<GH){const k=gy*GW+gx;if(smap[k]){smap[k]=0;mapChanged=true}if(!seen[k]){seen[k]=1;mapChanged=true}}
  }
  for(const q of hits){
    const gx=q.x/CS|0,gy=q.y/CS|0;
    if(!(gx>=0&&gx<GW&&gy>=0&&gy<GH))continue;
    const k=gy*GW+gx;
    if(!seen[k]){seen[k]=1;mapChanged=true}
    if(q.type==='person'){traffic[k]=Math.min(40,traffic[k]+.01);continue}   // people are tracked separately, never stored as furniture
    if(q.type==='user')continue;
    if(time-since[k]>=1){if(!smap[k])mapChanged=true;smap[k]=time}
  }
  if((++mapCalls)%60===0)for(let i=0;i<GW*GH;i++)if(traffic[i]>0)traffic[i]*=.9982;   // busy spots fade slowly (~9 min)
  if(time-lastSaveT>10){lastSaveT=time;if(mapChanged){saveMap();mapChanged=false}}
}
function scan(){
  hits=[]; peopleInPathCount=0; clrRays=[];
  const R=260;
  const c0=Math.cos(T.th), s0=Math.sin(T.th);
  for(const L of LIDARS){
    const oxw=T.x+c0*L[0]-s0*L[1];
    const oyw=T.y+s0*L[0]+c0*L[1];
    for(let i=0;i<LIDAR_RAYS;i++){
      const aw=T.th+i/LIDAR_RAYS*6.2832;
      const c=Math.cos(aw), s=Math.sin(aw);
      const al=aw-T.th;
      const tExit=rayExitBox(L[0],L[1],Math.cos(al),Math.sin(al),HW,HH);
      const d0=Math.max(tExit+2,3);          // rays start outside the desk's own body
      let best=1e9,info=null;
      const consider=(sp,inf)=>{ if(!sp||sp[1]<d0)return; const t=Math.max(sp[0],d0); if(t<best){best=t;info=inf} };
      for(const r of rects)consider(spanRect(oxw,oyw,c,s,r),{type:'rect'});
      for(const q of circs)consider(spanCircle(oxw,oyw,c,s,q.x,q.y,q.r),{type:'circ'});
      for(const p of people)consider(spanCircle(oxw,oyw,c,s,p.x,p.y,p.r),{type:'person',person:p});
      consider(spanCircle(oxw,oyw,c,s,user.x,user.y,user.r),{type:'user'});
      const tw=rayExitBox(oxw,oyw,c,s,W,H,0,0);   // map border
      if(tw>=d0&&tw<best){best=tw+0.5;info={type:'wall'}}
      clrRays.push([oxw,oyw,c,s,d0,(info&&best<R)?best-12:R]);   // free space up to just before the hit
      if(!info||best>=R)continue;
      const d=best,x=oxw+c*d,y=oyw+s*d;
      const isPerson=info.type==='person';
      const settled=isPerson&&info.person.settled;
      hits.push({a:aw,d,hit:true,x,y,ox:oxw,oy:oyw,dyn:isPerson&&!settled,settledPerson:settled,type:info.type});
      const gx=x/CS|0, gy=y/CS|0;
      if(gx>=0&&gx<GW&&gy>=0&&gy<GH){
        const k=gy*GW+gx;
        if(time-last[k]>.4) since[k]=time;
        last[k]=time;
      }
    }
  }
}

function astar(blk,s,g){const N=GW*GH,G=new Float32Array(N).fill(1e9),P=new Int32Array(N).fill(-1),C=new Uint8Array(N),Hp=[];
const hf=i=>{const dx=Math.abs(i%GW-g%GW),dy=Math.abs((i/GW|0)-(g/GW|0));return Math.max(dx,dy)+0.414*Math.min(dx,dy)};
const push=(f,i)=>{Hp.push([f,i]);let k=Hp.length-1;while(k>0){const p=(k-1)>>1;if(Hp[p][0]<=Hp[k][0])break;[Hp[p],Hp[k]]=[Hp[k],Hp[p]];k=p}};
const pop=()=>{const t=Hp[0],l=Hp.pop();if(Hp.length){Hp[0]=l;let k=0;for(;;){const a=2*k+1,b=a+1;let m=k;if(a<Hp.length&&Hp[a][0]<Hp[m][0])m=a;if(b<Hp.length&&Hp[b][0]<Hp[m][0])m=b;if(m===k)break;[Hp[m],Hp[k]]=[Hp[k],Hp[m]];k=m}}return t};
G[s]=0;push(hf(s),s);
while(Hp.length){const i=pop()[1];if(C[i])continue;C[i]=1;if(i===g)break;const x=i%GW,y=i/GW|0;
for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){if(!dx&&!dy)continue;const nx=x+dx,ny=y+dy;if(nx<0||ny<0||nx>=GW||ny>=GH)continue;
const j=ny*GW+nx;if(blk[j]||C[j])continue;if(dx&&dy&&(blk[y*GW+nx]||blk[ny*GW+x]))continue;
const ng=G[i]+(dx&&dy?1.414:1);if(ng<G[j]){G[j]=ng;P[j]=i;push(ng+hf(j),j)}}}
if(!C[g])return null;const out=[];for(let i=g;i!==-1;i=P[i])out.push({x:(i%GW+.5)*CS,y:((i/GW|0)+.5)*CS});return out.reverse()}

// ===== Orientation-aware planning =====
// State = (cell, orientation). The desk is a 2*HW x 2*HH rectangle, so what fits through a gap depends on
// which way it faces. Orientation k means the long axis points at k*22.5 deg (the desk is symmetric, so mod 180).
// Each orientation gets its own blocked grid: every obstacle cell is grown by the real rotated footprint
// (plus a margin). A* then moves in 8 directions and can also rotate in place +-22.5 deg wherever both
// orientations are free, so it inserts a turn exactly where there is room for one.
const NKF=8, NK=16, DK=Math.PI/NKF, ALIGN_PEN=0.25, ROT_COST=1.0, ROT_MOVE=0.5, MG_STRICT=7, MG_TIGHT=2;
const DIRS=[[1,0],[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]];
const STEPC=[];for(let k=0;k<NK;k++)STEPC.push(DIRS.map(([dx,dy])=>Math.hypot(dx,dy)*(1+ALIGN_PEN*Math.abs(Math.sin(Math.atan2(dy,dx)-k*DK)))));
function makeKernels(mg){
  const out=[];
  for(let k=0;k<NKF;k++){
    const c=Math.cos(k*DK),s=Math.sin(k*DK),ac=Math.abs(c),as=Math.abs(s),hc=CS/2;
    const ex=(HW+mg)*ac+(HH+mg)*as+hc, ey=(HW+mg)*as+(HH+mg)*ac+hc;
    const hu=HW+mg+hc*(ac+as), hv=HH+mg+hc*(ac+as), R=Math.ceil(Math.max(ex,ey)/CS)+1, list=[];
    for(let b=-R;b<=R;b++)for(let a=-R;a<=R;a++){
      const dx=a*CS,dy=b*CS;
      if(Math.abs(dx)>ex||Math.abs(dy)>ey)continue;
      if(Math.abs(dx*c+dy*s)>hu||Math.abs(-dx*s+dy*c)>hv)continue;
      list.push([a,b]);
    }
    out.push(list);
  }
  return out;
}
const KERN_STRICT=makeKernels(MG_STRICT), KERN_TIGHT=makeKernels(MG_TIGHT);
function angleNear(thPlan,thCur){return thPlan+Math.round((thCur-thPlan)/(2*Math.PI))*2*Math.PI}   // full turn: the side that faces you matters

function buildOccupancy(){
  const occ=new Uint8Array(GW*GH);
  for(let i=0;i<GW*GH;i++)if((time-last[i]<1.2&&last[i]-since[i]>=1)||smap[i]>0)occ[i]=1;
  // Moving people are waited for, not routed around: drop only THEIR own cells (not the furniture next to them).
  for(const p of people){
    if(p.settled||(time<patientUntil&&Math.hypot(p.x-T.x,p.y-T.y)<150))continue;
    const cx=p.x/CS|0,cy=p.y/CS|0,rr=p.r+3,r=Math.ceil(rr/CS);
    for(let dy=-r;dy<=r;dy++)for(let dx=-r;dx<=r;dx++){
      if(Math.hypot(dx*CS,dy*CS)>rr)continue;const x=cx+dx,y=cy+dy;
      if(x>=0&&x<GW&&y>=0&&y<GH)occ[y*GW+x]=0;
    }
  }
  // Settled people are obstacles, with a comfort margin.
  for(const p of people){
    if(!(p.settled||(time<patientUntil&&Math.hypot(p.x-T.x,p.y-T.y)<150)))continue;
    const cx=p.x/CS|0,cy=p.y/CS|0,rr=p.r+8,r=Math.ceil(rr/CS);
    for(let dy=-r;dy<=r;dy++)for(let dx=-r;dx<=r;dx++){
      if(Math.hypot(dx*CS,dy*CS)>rr)continue;const x=cx+dx,y=cy+dy;
      if(x>=0&&x<GW&&y>=0&&y<GH)occ[y*GW+x]=1;
    }
  }
  return occ;
}
function buildBlocked(occ,kern,mg){
  const NC=GW*GH,B=[];for(let k=0;k<NKF;k++)B.push(new Uint8Array(NC));
  for(let i=0;i<NC;i++){
    if(!occ[i])continue;const cx=i%GW,cy=i/GW|0;
    for(let k=0;k<NKF;k++){const bk=B[k];for(const[a,b]of kern[k]){const x=cx+a,y=cy+b;if(x>=0&&x<GW&&y>=0&&y<GH)bk[y*GW+x]=1}}
  }
  for(let k=0;k<NKF;k++){
    const ac=Math.abs(Math.cos(k*DK)),as=Math.abs(Math.sin(k*DK)),ex=HW*ac+HH*as+mg,ey=HW*as+HH*ac+mg;
    for(let y=0;y<GH;y++)for(let x=0;x<GW;x++){const px=(x+.5)*CS,py=(y+.5)*CS;if(px<ex||px>W-ex||py<ey||py>H-ey)B[k][y*GW+x]=1}
  }
  const sx=T.x/CS|0,sy=T.y/CS|0;
  for(let k=0;k<NKF;k++)for(let a=-1;a<=1;a++)for(let b=-1;b<=1;b++){const x=sx+a,y=sy+b;if(x>=0&&x<GW&&y>=0&&y<GH)B[k][y*GW+x]=0}
  return B;
}
const _NS=GW*GH*NK,_G=new Float32Array(_NS),_P=new Int32Array(_NS),_Cl=new Uint8Array(_NS),_HF=new Float32Array(_NS*10),_HI=new Int32Array(_NS*10);
const HW_W=1.5, SECTOR_PEN=45, MAX_POPS=64000;   // heuristic weight, cost of NOT parking above/below you, search cap
function planOri(B,Bt){
  const NC=GW*GH,G=_G,P=_P,Cl=_Cl;G.fill(1e9);P.fill(-1);Cl.fill(0);
  const goHome=mode==='home'||mode==='tour',dd=dest(),gx=dd.x,gy=dd.y;
  // goal = (cell, orientation). Near you: a ring cell AND the parking orientation for that cell, so the turn to face
  // you is planned (and footprint-checked) on the way in. Cells directly above/below you (table horizontal) are free;
  // other ring cells carry a penalty, so the desk takes them only when the horizontal spots mean a big detour.
  const ring=new Uint8Array(NC),kPk=new Int8Array(NC).fill(-1),gpen=new Float32Array(NC),hfa=new Float32Array(NC);
  for(let c=0;c<NC;c++)hfa[c]=HW_W*Math.max(0,(Math.hypot((c%GW+.5)*CS-gx,((c/GW|0)+.5)*CS-gy)-(goHome?0:76))/CS);
  if(goHome){const hc=(Math.max(0,Math.min(GH-1,gy/CS|0)))*GW+(Math.max(0,Math.min(GW-1,gx/CS|0)));ring[hc]=1;kPk[hc]=mode==='tour'?-2:0}
  else for(let c=0;c<NC;c++){
    const cx=(c%GW+.5)*CS,cy=((c/GW|0)+.5)*CS,d=Math.hypot(cx-user.x,cy-user.y);
    if(Math.abs(d-70)<6){
      ring[c]=1;gpen[c]=Math.abs(cx-user.x)<.7*Math.abs(cy-user.y)?0:SECTOR_PEN;
      const u=Math.atan2(user.y-cy,user.x-cx),thP=u-Math.PI/2,thH=Math.sin(u)>0?0:Math.PI,t=Math.abs(ang(thP,thH))<=.65?thH:thP;
      kPk[c]=((Math.round(t/DK)%NK)+NK)%NK;
    }
  }
  let hn=0;
  const push=(f,i)=>{let k=hn++;while(k>0){const p=(k-1)>>1;if(_HF[p]<=f)break;_HF[k]=_HF[p];_HI[k]=_HI[p];k=p}_HF[k]=f;_HI[k]=i};
  const pop=()=>{const ri=_HI[0],lf=_HF[--hn],li=_HI[hn];if(hn>0){let k=0;for(;;){const a=2*k+1;if(a>=hn)break;let m=a;if(a+1<hn&&_HF[a+1]<_HF[a])m=a+1;if(_HF[m]>=lf)break;_HF[k]=_HF[m];_HI[k]=_HI[m];k=m}_HF[k]=lf;_HI[k]=li}return ri};
  const sx=Math.max(0,Math.min(GW-1,T.x/CS|0)),sy=Math.max(0,Math.min(GH-1,T.y/CS|0));
  const k0=((Math.round(T.th/DK)%NK)+NK)%NK;
  const s0=k0*NC+sy*GW+sx;G[s0]=0;push(hfa[s0%NC],s0);
  let goalS=-1,pops=0,bestS=-1,bestC=1e18;
  while(hn>0){
    const e=pop();
    if(e<0){goalS=-e-1;break}                       // a goal entry reached the top: it is the cheapest total
    const s=e;if(Cl[s])continue;Cl[s]=1;
    if(++pops>MAX_POPS)break;
    const c=s%NC,k=(s/NC)|0;
    if(ring[c]&&(kPk[c]===-2||k===kPk[c])){const tc=G[s]+gpen[c];if(tc<bestC){bestC=tc;bestS=s}push(tc,-(s+1));continue}
    const x=c%GW,y=c/GW|0,bk=B[k&7];
    for(let d=0;d<8;d++){
      const dx=DIRS[d][0],dy=DIRS[d][1],nx=x+dx,ny=y+dy;
      if(nx<0||ny<0||nx>=GW||ny>=GH)continue;
      const nc=ny*GW+nx;
      for(let dk=-1;dk<=1;dk++){
        const nk=(k+dk+NK)%NK,gn=ring[nc]===1&&(kPk[nc]===-2||nk===kPk[nc])&&!Bt[nk&7][nc];   // the final parking spot may be tighter than a travel lane
        const bn=gn?Bt[nk&7]:B[nk&7],bo=gn?Bt[k&7]:bk;
        if(bn[nc])continue;
        if(dk!==0&&(ROOMY[c]||ROOMY[nc]))continue;   // moving turns only where there's lots of room (see buildRoomy)
        if(dk!==0&&(bo[nc]||bn[c]))continue;     // turning while moving: the mixed old/new footprints must be free too
        if(dx&&dy&&(bn[y*GW+nx]||bn[ny*GW+x]||bo[y*GW+nx]||bo[ny*GW+x]))continue;
        const ns=nk*NC+nc;if(Cl[ns])continue;
        const ng=G[s]+STEPC[nk][d]+(dk?ROT_MOVE:0)+RISK[nc];
        if(ng<G[ns]){G[ns]=ng;P[ns]=s;push(ng+hfa[nc],ns)}
      }
    }
    for(const dk of[1,NK-1]){   // pure in-place turn: only when no combined move gets there
      const nk=(k+dk)%NK;if(B[nk&7][c]||ROTOK[c])continue;   // in-place turns only where it could turn to ANY angle with margin (no exception for the start cell)
      const ns=nk*NC+c;if(Cl[ns])continue;
      const ng=G[s]+ROT_COST;
      if(ng<G[ns]){G[ns]=ng;P[ns]=s;push(ng+hfa[c],ns)}
    }
  }
  if(goalS<0)goalS=bestS;                          // search was cut short: use the best goal found so far
  if(goalS<0)return [];
  const out=[];
  for(let s=goalS;s!==-1;s=P[s]){const c=s%NC,k=(s/NC)|0;out.push({x:(c%GW+.5)*CS,y:((c/GW|0)+.5)*CS,th:k*DK,k})}
  return out.reverse();
}
// Try the spot above/below you first (table ends up horizontal); use it unless it means a big detour.
function planBest(B){return planOri(B)}
// A cell is "roomy" if nothing is within the desk's circumscribed radius + 22 px, in ANY orientation. Only there may the plan
// combine a turn with a move: the planner knows nothing about yaw rate, and near walls/gaps a turn must be a deliberate in-place one.
const ROOMY_R=Math.hypot(HW,HH)+22, ROT_R=Math.hypot(HW,HH)+8, roomOffs=[], rotOffs=[];
for(let a=-8;a<=8;a++)for(let b=-8;b<=8;b++){const d=Math.hypot(a*CS,b*CS);if(d<=ROOMY_R+CS/2)roomOffs.push([a,b]);if(d<=ROT_R+CS/2)rotOffs.push([a,b])}
let ROOMY=new Uint8Array(GW*GH),ROTOK=new Uint8Array(GW*GH);   // 1 = NOT roomy / NOT safe to rotate here
function buildRoomy(occ,offs,rad){
  const R=new Uint8Array(GW*GH);
  for(let i=0;i<GW*GH;i++){
    if(!occ[i])continue;const cx=i%GW,cy=i/GW|0;
    for(const[a,b]of offs){const x=cx+a,y=cy+b;if(x>=0&&x<GW&&y>=0&&y<GH)R[y*GW+x]=1}
  }
  for(let y=0;y<GH;y++)for(let x=0;x<GW;x++){const px=(x+.5)*CS,py=(y+.5)*CS;if(px<rad||px>W-rad||py<rad||py>H-rad)R[y*GW+x]=1}
  return R;
}
function plan(){
  const occ=buildOccupancy();
  for(let i=0;i<GW*GH;i++)RISK[i]=(seen[i]?0:UNK_PEN)+Math.min(TRAF_MAX,traffic[i]*TRAF_K);ROOMY=buildRoomy(occ,roomOffs,ROOMY_R);ROTOK=buildRoomy(occ,rotOffs,ROT_R);
  const Bt=buildBlocked(occ,KERN_TIGHT,MG_TIGHT);
  let r;
  if(time<squeezeUntil){   // flipping between a strict route and a tight one every replan made the desk go back and forth
    r=planOri(Bt,Bt);squeeze=true;
    if(!r.length){r=planOri(buildBlocked(occ,KERN_STRICT,MG_STRICT),Bt);squeeze=false}
  }else{
    r=planOri(buildBlocked(occ,KERN_STRICT,MG_STRICT),Bt);squeeze=false;
    if(!r.length){   // nothing fits with the full margin: retry tighter (the supervisor still guards contact)
      const r2=planOri(Bt,Bt);
      if(r2.length){r=r2;squeeze=true;squeezeUntil=time+3}
    }
  }
  path=r;goal=r.length?{x:r[r.length-1].x,y:r[r.length-1].y}:null;
}

function pushOut(p){
  for(const r of rects){const cx=Math.max(r.x,Math.min(p.x,r.x+r.w)),cy=Math.max(r.y,Math.min(p.y,r.y+r.h));let dx=p.x-cx,dy=p.y-cy,d=Math.hypot(dx,dy);
  if(d<p.r){if(d<1e-6){dx=0;dy=-1;d=1}p.x=cx+dx/d*p.r;p.y=cy+dy/d*p.r}}
  for(const c of circs){const dx=p.x-c.x,dy=p.y-c.y,d=Math.hypot(dx,dy)||1,m=c.r+p.r;if(d<m){p.x=c.x+dx/d*m;p.y=c.y+dy/d*m}}
  p.x=Math.max(p.r,Math.min(W-p.r,p.x));p.y=Math.max(p.r,Math.min(H-p.r,p.y));
}

function pushFromDesk(p){
  const c=Math.cos(-T.th),s=Math.sin(-T.th),dx=p.x-T.x,dy=p.y-T.y;
  const lx=c*dx-s*dy,ly=s*dx+c*dy,qx=Math.max(-HW,Math.min(HW,lx)),qy=Math.max(-HH,Math.min(HH,ly));
  let ox=lx-qx,oy=ly-qy,d=Math.hypot(ox,oy);
  if(d>=p.r)return;
  if(d<1e-6){ // centre is inside the footprint: leave through the nearest face
    if(HW-Math.abs(lx)<HH-Math.abs(ly)){ox=Math.sign(lx)||1;oy=0}else{ox=0;oy=Math.sign(ly)||1}
    d=1;const px=qx+(ox>0?HW-qx:ox<0?-HW-qx:0),py=qy+(oy>0?HH-qy:oy<0?-HH-qy:0);
    ox=px+ox*p.r;oy=py+oy*p.r;
  }else{ox=qx+ox/d*p.r;oy=qy+oy/d*p.r}
  const cb=Math.cos(T.th),sb=Math.sin(T.th);
  p.x=T.x+cb*ox-sb*oy;p.y=T.y+sb*ox+cb*oy;
}

// ---- true-footprint helpers (desk is a 2*HW x 2*HH rectangle, not a circle) ----
function deskPoly(){const c=Math.cos(T.th),s=Math.sin(T.th);return[[-HW,-HH],[HW,-HH],[HW,HH],[-HW,HH]].map(([x,y])=>[T.x+c*x-s*y,T.y+s*x+c*y])}
function satOverlap(A,B){for(const P of[A,B])for(let i=0;i<P.length;i++){const a=P[i],b=P[(i+1)%P.length],nx=-(b[1]-a[1]),ny=b[0]-a[0];
  let a0=1e9,a1=-1e9,b0=1e9,b1=-1e9;for(const p of A){const d=p[0]*nx+p[1]*ny;a0=Math.min(a0,d);a1=Math.max(a1,d)}
  for(const p of B){const d=p[0]*nx+p[1]*ny;b0=Math.min(b0,d);b1=Math.max(b1,d)}if(a1<b0||b1<a0)return false}return true}
function deskTouchesCircle(cx,cy,r){const c=Math.cos(-T.th),s=Math.sin(-T.th),dx=cx-T.x,dy=cy-T.y,lx=c*dx-s*dy,ly=s*dx+c*dy;
  return Math.hypot(lx-Math.max(-HW,Math.min(HW,lx)),ly-Math.max(-HH,Math.min(HH,ly)))<r}
function footprintHit(){const d=deskPoly();
  for(const r of rects)if(satOverlap(d,[[r.x,r.y],[r.x+r.w,r.y],[r.x+r.w,r.y+r.h],[r.x,r.y+r.h]]))return true;
  for(const c of circs)if(deskTouchesCircle(c.x,c.y,c.r))return true;
  for(const p of people)if(deskTouchesCircle(p.x,p.y,p.r))return true;
  return false}

// ---- Safety supervisor: last stage before the wheels. Uses only what the lidar can see. ----
// Translation: sweep the real rectangular footprint along the commanded AND the actual velocity,
// and cap speed so the desk can always stop (at SAFE_DEC, below its real 60 px/s^2) before contact.
// Rotation: refuse yaw that would swing a corner into a lidar point.
function sweepFree(pts,ux,uy,m){
  const co=Math.cos(T.th),si=Math.sin(T.th),udx=co*ux+si*uy,udy=-si*ux+co*uy,hx=HW+m,hy=HH+m;
  let free=1e9;
  for(const p of pts){
    const qx=p[2],qy=p[3];
    if(p[0]*ux+p[1]*uy<=0&&Math.abs(qx)<=hx&&Math.abs(qy)<=hy)continue;   // touching but moving away: allowed
    let t0=0,t1=1e9,ok=true;
    for(const[q,u,hh]of[[qx,udx,hx],[qy,udy,hy]]){
      if(Math.abs(u)<1e-9){if(Math.abs(q)>hh){ok=false;break}}
      else{let a=(q-hh)/u,b=(q+hh)/u;if(a>b){const t=a;a=b;b=t}t0=Math.max(t0,a);t1=Math.min(t1,b)}
    }
    if(ok&&t0<=t1&&t1>=0)free=Math.min(free,Math.max(0,t0));
  }
  return free;
}
function supervise(cx,cy,w){
  const co=Math.cos(T.th),si=Math.sin(T.th),pts=[];
  for(const h of hits){const dx=h.x-T.x,dy=h.y-T.y;if(dx*dx+dy*dy>180*180)continue;pts.push([dx,dy,co*dx+si*dy,-si*dx+co*dy])}
  let held=false;
  const v=Math.hypot(cx,cy),va=Math.hypot(T.vx,T.vy);
  if(v>1||va>3){
    const m=SAFE_M+Math.min(8,Math.abs(w)*10);
    if(v>1){
      const f=sweepFree(pts,cx/v,cy/v,m),vAllow=Math.sqrt(2*SAFE_DEC*Math.max(0,f-STOP_GAP));
      if(vAllow<v){const k=vAllow/v;cx*=k;cy*=k;if(k<.05)held=true}
    }
    if(va>3){   // the desk is still coasting along its real velocity: brake if that path is short
      const f=sweepFree(pts,T.vx/va,T.vy/va,m),vAllow=Math.sqrt(2*SAFE_DEC*Math.max(0,f-STOP_GAP));
      if(va>vAllow){cx=0;cy=0;held=true}
    }
  }
  if(Math.abs(w)>.02){
    const hx=HW+3,hy=HH+3,dir=Math.sign(w),look=.06+Math.abs(w)*.3;
    const pen=a=>{const c=Math.cos(T.th+a),s=Math.sin(T.th+a);let b=0;for(const p of pts){const qx=c*p[0]+s*p[1],qy=-s*p[0]+c*p[1];
      const d=Math.min(hx-Math.abs(qx),hy-Math.abs(qy));if(d>b)b=d}return b};
    const p0=pen(0);let worst=p0;for(let k=1;k<=4;k++)worst=Math.max(worst,pen(dir*look*k/4));
    if(worst>0&&worst>p0+.5){w=0;held=true}
  }
  if(held&&!holdLatch&&(v>8||Math.abs(w)>.1))safetyHolds++;
  holdLatch=held;
  return [cx,cy,w,held];
}

// The desk's forward is the BACK OF THE MONITOR (local -y); you stand on the keyboard side (local +y).
// Parking turns the desk to face you with its keyboard side. When you're roughly above or below it, that is
// exactly horizontal on screen (long axis along x), so snap to it.
// ---- batteries + dock ----
// Robot battery (12 V SLA): drive.  UPS battery: computer, lidars, desk power, peripherals. A UPS charges itself whenever it has wall power.
// Charging starts BY ITSELF when the desk is parked on the dock pad (the start spot), stationary, facing forward.
// Low battery while idle sends it back to the dock on its own.  (Rates are sim-accelerated, %/s of sim time.)
// ---- Charging station: manual cord ----
// One AC cord with a magnetic breakaway connector. Two independent layers protect a forgotten cord:
//  1) INTERLOCK: if the cord is plugged in, the desk refuses to drive and raises an alert.
//  2) BREAKAWAY: if it is driven anyway (interlock off or failed), the connector lets go at ~0.9 m instead of dragging the plug.
function manualDock(){return $('dm').value==='manual'}
function cordLock(){return manualDock()&&cordIn&&$('il').checked}   // true = the wheels must not be commanded
function cordBlocks(){
  if(manualDock()&&cordIn&&$('il').checked){setSt('Cord is still plugged in: unplug it first (the desk will not drive)','bad');alertT=4;return true}
  return false;
}
function dockedNow(){
  return Math.hypot(T.x-HOME.x,T.y-HOME.y)<=4&&!active&&parkTh===null&&Math.hypot(T.vx,T.vy)<2&&Math.abs(ang(0,T.th))<.25;
}
function batStep(dt){
  const sp=Math.hypot(T.vx,T.vy),lifting=Math.abs(deskH-prevDeskH)>1e-7;prevDeskH=deskH;
  charging=manualDock()?cordIn:dockedNow();   // with a manual cord, plugged in IS charging (the cord only reaches the dock)
  if(alertT>0)alertT-=dt;
  if(cordIn&&Math.hypot(T.x-HOME.x,T.y-HOME.y)>45){cordIn=false;breakaways++;setSt('Cord pulled out: the breakaway connector released safely','warn')}
  if(charging&&!wasCharging&&!active&&!readyPending&&$('st').textContent.startsWith('Ready - back'))setSt('Ready - docked and charging, desk at '+Math.round(deskH*100)+' cm','ok');
  wasCharging=charging;
  if(charging){bat.r+=.9*dt;bat.a+=.6*dt}
  else{bat.r-=(.03+.35*sp/VMAX+.08*Math.abs(T.w))*dt;bat.a-=(.04+(lifting?.5:0))*dt}
  bat.r=Math.max(0,Math.min(100,bat.r));bat.a=Math.max(0,Math.min(100,bat.a));
  if(!active&&!estop&&!charging&&mode==='user'&&parkTh===null&&(bat.r<25||bat.a<20)){
    lowT+=dt;if(lowT>=3){startReturn();setSt('Low battery - returning to the dock to charge','warn')}
  }else lowT=0;
}
function startTour(){
  manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;
  if(cordBlocks())return;
  if(estop){setSt('E-stop engaged - release it first','bad');return}
  mode='tour';active=true;parkTh=null;didArrive=false;arT=0;tourI=0;tourT=0;trail=[];retreats=0;waitT=0;peakA=0;rp=0;minGap=1e9;
  stuckTime=0;recovery=null;recoveryT=0;headTarget=T.th;headState='SETTLED';setSt('Learning the room: point 1 of '+tourPts().length,'');
}
function startReturn(){
  manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;
  if(cordBlocks())return;
  if(estop){setSt('E-stop engaged - release it first','bad');return}
  mode='home';active=true;parkTh=null;didArrive=false;arT=0;trail=[];retreats=0;waitT=0;peakA=0;rp=0;minGap=1e9;stuckTime=0;recovery=null;recoveryT=0;
  headTarget=T.th;headState='SETTLED';setSt('Returning to start…','');
}
function startParking(){
  const u=Math.atan2(user.y-T.y,user.x-T.x);
  const thP=u-Math.PI/2;                                  // local +y points at you
  const thH=Math.sin(u)>0?0:Math.PI;                      // horizontal table: you below -> 0, you above -> PI
  const t=Math.abs(ang(thP,thH))<=.65?thH:thP;
  parkTh=t+Math.round((T.th-t)/(2*Math.PI))*2*Math.PI;parkT=8;   // mod 2*PI here: which SIDE faces you matters
}
function update(dt){time+=dt;

// ============ PEOPLE ============
for(const p of people){
  if(p.type==='sitter'){
    // Stationary from the start; already settled. Occasionally shift slightly
    // to look alive, but never enough to reset the still timer.
    p.stillTime += dt;
    p.settled = true;
    continue;
  }
  // Update target by type
  if(p.type==='talker'){
    p.tx = p.anchorX + Math.sin(time*0.35 + p.jitterPhase)*1.8;
    p.ty = p.anchorY + Math.cos(time*0.45 + p.jitterPhase)*1.8;
    p.sp = 2.5;
  } else { // walker
    p.t -= dt;
    if(p.t<=0 || Math.hypot(p.tx-p.x, p.ty-p.y)<8){
      if(p.convertOnArrival){
        p.type='sitter';
        p.sp=0; p.t=1e9;
        p.stillTime=0; p.settled=false;
        p.convertOnArrival=false;
      } else {
        p.tx=40+Math.random()*(W-80);
        p.ty=40+Math.random()*(H-80);
        p.t=4+Math.random()*4;
      }
    }
  }
  // Velocity command
  const dx=p.tx-p.x, dy=p.ty-p.y, d=Math.hypot(dx,dy)||1;
  let vx=dx/d*p.sp, vy=dy/d*p.sp;
  // Flee from the bot — only for walkers
  if(p.type==='walker'){
    const tx=p.x-T.x, ty=p.y-T.y, td=Math.hypot(tx,ty)||1;
    if(td<80){vx+=tx/td*35; vy+=ty/td*35}
  }
  p.x += vx*dt;
  p.y += vy*dt;
  pushOut(p);
  pushFromDesk(p);
  // Motion classification
  const moved = Math.hypot(p.x-p.lastX, p.y-p.lastY)/Math.max(dt,1e-3);
  if(moved > MOVE_THRESH){
    p.stillTime = 0;
    p.settled = false;
  } else {
    p.stillTime += dt;
    if(p.stillTime > STAND_WAIT) p.settled = true;
  }
  p.lastX = p.x; p.lastY = p.y;
}

scan();mapUpdate();
rp-=dt; if(active&&rp<=0){const t0=performance.now();plan();const pd=(performance.now()-t0)/1000;rp=path.length?Math.min(.5,Math.max(.2,10*pd)):.7}

let cx=0,cy=0,w=0;
const tgt=dest(),du=Math.hypot(tgt.x-T.x,tgt.y-T.y);
const low=true;   // the desk no longer lowers before driving: it stays at the user's height the whole trip
if(estop){active=false;path=[];T.vx=T.vy=T.w=0;manualDrive={...manualDrive,x:0,y:0,turn:0};setSt('E-STOP engaged - click the red button to release','bad')}
let thTarget=null,rotHere=false;
if(active){
  if(mode==='tour'?du<=8:mode==='home'?du<=3:(du<=80&&(!goal||Math.hypot(T.x-goal.x,T.y-goal.y)<=22||du<=70))){
   if(mode==='tour'){
     tourI++;rp=0;path=[];trail=[];
     if(tourI>=tourPts().length){startReturn();setSt('Room mapped - returning to the dock','ok')}
     else setSt('Learning the room: point '+(tourI+1)+' of '+tourPts().length,'');
   }else{
    active=false;
    if(mode==='home'){parkTh=Math.round(T.th/(2*Math.PI))*2*Math.PI;parkT=8;didArrive=false;readyPending=true;setSt('Arrived – back at start, turning to park','ok')}
    else{startParking();didArrive=true;arT=0;readyPending=true;setSt('Arrived – turning to face you','ok')}
   }
  }
  else if(!path.length)setSt('No safe path – waiting','warn')
  else{
    let bi=0,bd=1e9;
    for(let i=0;i<path.length;i++){const d=Math.hypot(path[i].x-T.x,path[i].y-T.y);if(d<bd){bd=d;bi=i}}
    // ---- orientation-aware following ----
    // Swerve: the desk turns WHILE it drives. It only stops for a turn if the plan has an in-place one (same spot,
    // new orientation), which A* now uses only where no moving turn fits. Otherwise the yaw target is simply the
    // orientation planned for the next waypoint, and speed drops only if yaw falls well behind.
    let tiLimit=path.length-1,dRcur=1e9,hasR=false;
    for(;;){
      let Rj=-1;
      for(let j=bi+1;j<path.length;j++){if(path[j].k!==path[j-1].k&&Math.hypot(path[j].x-path[j-1].x,path[j].y-path[j-1].y)<1){Rj=j-1;break}}
      if(Rj<0)break;
      const R=path[Rj],dR=Math.hypot(R.x-T.x,R.y-T.y);
      if(dR>7){tiLimit=Rj;dRcur=dR;hasR=true;break}
      let e=Rj;while(e+1<path.length&&Math.hypot(path[e+1].x-R.x,path[e+1].y-R.y)<1)e++;
      const thT=angleNear(path[e].th,T.th);
      if(Math.abs(ang(thT,T.th))<.12){bi=e;continue}
      rotHere=true;thTarget=thT;break;
    }
    if(!rotHere)thTarget=angleNear(path[bi].th,T.th);
    let ti=bi;
    for(let i=bi;i<=tiLimit;i++){ti=i;if(Math.hypot(path[i].x-T.x,path[i].y-T.y)>38)break}
    const dx=path[ti].x-T.x,dy=path[ti].y-T.y,d=Math.hypot(dx,dy)||1,da=Math.atan2(dy,dx);
    const ca=Math.cos(da),sa=Math.sin(da);
    let sp=(mode==='home'||mode==='tour')?Math.min(VMAX,8+du*.5):Math.min(VMAX,15+(du-80)*.5),ms=1e9,md=1e9;
    let movingPeopleSeen=0, stillPeopleSeen=0;
    for(const h of hits){
      if(!h.hit)continue;
      if(Math.hypot(h.x-user.x,h.y-user.y)<user.r+5)continue;
      const a2=Math.abs(ang(h.a,da));
      if(h.dyn&&time>=patientUntil){ if(a2<1.05){md=Math.min(md,h.d); if(a2<0.6) movingPeopleSeen++; } }
      else     { if(a2<0.45) ms=Math.min(ms,h.d); }
    }
    peopleInPathCount = movingPeopleSeen;
    let vSafe=VMAX;
    for(const h of hits){
      if(!h.hit)continue;
      if(Math.hypot(h.x-user.x,h.y-user.y)<user.r+5)continue;
      const a2=Math.abs(ang(h.a,da));
      const inCone = (h.dyn&&time>=patientUntil) ? (a2<1.05) : false;   // people keep a comfort gap; static obstacles are handled exactly by the swept-footprint supervisor
      if(!inCone)continue;
      const gap=Math.max(0,h.d-TR-8);
      if(gap<=0){vSafe=0;break}
      const vb=Math.sqrt(2*ABRAKE*gap); if(vb<vSafe)vSafe=vb;
    }
    sp=Math.min(sp,vSafe);
    if(hasR)sp=Math.min(sp,5+Math.sqrt(2*40*Math.max(0,dRcur-7)));      // arrive at the turn point slowly
    sp*=Math.max(0,Math.min(1,(.7-Math.abs(ang(rotHere?thTarget:angleNear(path[bi].th,T.th),T.th)))/.35));   // yaw must keep up with the orientation the plan assumes HERE (<=20 deg: full speed)   // don't drive while facing the wrong way
    if(recovery==='ALIGN'){
      const err=ang(da,T.th); cx=0;cy=0;w=Math.max(-.6,Math.min(.6,err*2.5));
      setSt('Aligning to gap','warn');
    } else if(recovery==='BACKOFF'){
      cx=-ca*VSLOW;cy=-sa*VSLOW;w=0; setSt('Backing off','warn');
    } else if(recovery==='SPIN'){
      cx=0;cy=0;w=0.5; setSt('Re-scanning','warn');
    } else {
      cx=dx/d*sp;cy=dy/d*sp;
      const sd=Math.max(0,Math.min(1,(md-TR-22)/70)),ss=1,sc=Math.min(sd,ss);
      if(sd===0&&sc>0)setSt('Waiting for person to move','warn');
      else if(ss===0&&sc>0)setSt('Blocked - re-routing','warn');
      else if(sc<.99)setSt(sd<ss?'Slowing - person nearby':'Slowing - obstacle ahead','warn');
      else setSt('Driving to you','');
    }
  }
  // RETREAT: reverse along the trail we came in on, matching the orientation we had at each crumb
  if(recovery==='RETREAT'){
    while(trail.length>1&&Math.hypot(T.x-trail[trail.length-1].x,T.y-trail[trail.length-1].y)<10)trail.pop();
    const tc=trail[trail.length-1];
    const gi=T.x/CS|0,gj=T.y/CS|0,roomy=gi>=0&&gi<GW&&gj>=0&&gj<GH&&!ROOMY[gj*GW+gi];
    if(!tc||(roomy&&recoveryT>1.5)||retreatD>=80||recoveryT>13){recovery=null;recoveryT=0;stuckTime=0;rp=0;waitT=1.2;trail=[];lastRecEnd=time}
    else{
      const dd=Math.hypot(tc.x-T.x,tc.y-T.y)||1;
      cx=(tc.x-T.x)/dd*22;cy=(tc.y-T.y)/dd*22;retreatD+=22*dt;
      w=Math.max(-1.8,Math.min(1.8,ang(angleNear(tc.th,T.th),T.th)*3));
      setSt('Stuck - backing out the way it came in','warn');
    }
  }
  if(waitT>0&&!recovery){waitT-=dt;cx=0;cy=0;w=0;thTarget=null;rotHere=false;setSt('Backed out - re-planning','warn')}
  // Heading controller
  if(rotHere&&!recovery){cx=0;cy=0}
  if(low&&!recovery&&thTarget!==null){
    const err=ang(thTarget,T.th);
    headTarget=thTarget;
    if(Math.abs(err)<.04){w=0;headState='SETTLED'}
    else{w=Math.max(-YAW_RATE_MAX,Math.min(YAW_RATE_MAX,err*YAW_GAIN));headState='ALIGNING'}
    if(rotHere)setSt('Rotating to fit through the gap','warn');
  }
  else if(low&&!recovery){
    const sCmd=Math.hypot(cx,cy);
    if(sCmd<SPEED_DRIVE_MIN){
      if(headState!=='SETTLED'){
        const err=ang(headTarget,T.th);
        if(Math.abs(err)<HEADING_SETTLE){w=0;headState='SETTLED'}
        else w=Math.max(-YAW_RATE_MAX,Math.min(YAW_RATE_MAX,err*YAW_GAIN));
      } else w=0;
    } else {
      const rawDir=Math.atan2(cy,cx);
      const errF=ang(rawDir,headTarget);
      headTarget+=errF*Math.min(1,dt/HEADING_FILTER_TC);
      const errT=ang(headTarget,T.th);
      if(headState==='SETTLED'&&Math.abs(errT)>HEADING_COMMIT) headState='ALIGNING';
      if(headState==='ALIGNING'&&Math.abs(errT)<HEADING_SETTLE) headState='SETTLED';
      if(headState==='ALIGNING') w=Math.max(-YAW_RATE_MAX,Math.min(YAW_RATE_MAX,errT*YAW_GAIN));
      else w=0;
    }
  }
}

// Parking turn (after arriving): rotate in place until the keyboard side faces you
if(!active&&!estop&&parkTh!==null){
  parkT-=dt;
  const err=ang(parkTh,T.th);
  if(Math.abs(err)<.04||parkT<=0){parkTh=null;w=0}
  else{w=Math.max(-.8,Math.min(.8,err*YAW_GAIN));setSt(mode==='home'?'Parking - turning to face forward at the start':'Parking - turning keyboard side toward you','')}
}

// Tour: skip a waypoint that has no route for 5 s
if(mode==='tour'&&active&&!estop&&!path.length){
  tourT+=dt;
  if(tourT>5){tourT=0;tourI++;rp=0;if(tourI>=tourPts().length){startReturn();setSt('Room mapped (some points were unreachable) - returning to the dock','warn')}}
}else tourT=0;

// Patience: if a MOVING person has held the desk up for ~8 s and still hasn't cleared, stop waiting for them. For 12 s they
// count as an obstacle for planning (route around them) and the swept-footprint supervisor alone keeps the gap.
{
  const sp0=Math.hypot(T.vx,T.vy);
  if(active&&!estop&&low&&path.length&&!recovery&&!rotHere&&sp0<3){
    if(people.some(p=>!p.settled&&Math.hypot(p.x-T.x,p.y-T.y)<150))lowSpeedT+=dt;else lowSpeedT=Math.max(0,lowSpeedT-dt);
  }else lowSpeedT=Math.max(0,lowSpeedT-2*dt);
  if(lowSpeedT>8){patientUntil=time+12;lowSpeedT=0}
}

// Breadcrumb trail: positions AND orientations the desk has actually driven through (those states were collision-free)
if(active&&!estop&&!recovery){
  const lc=trail[trail.length-1];
  if(!lc||Math.hypot(T.x-lc.x,T.y-lc.y)>=8){trail.push({x:T.x,y:T.y,th:T.th});if(trail.length>150)trail.shift()}
}

// Stuck detection
{
  const actualSpeed=Math.hypot(T.vx,T.vy);
  const cmdSpeed=Math.hypot(cx,cy);
  const shouldBeMoving = active && !estop && low && (cmdSpeed>8 || (thTarget!==null&&Math.abs(ang(thTarget,T.th))>.3&&Math.abs(T.w)<.05));
  if(shouldBeMoving && actualSpeed<8) stuckTime+=dt;
  else stuckTime=Math.max(0,stuckTime-dt*2);
  {const gi=T.x/CS|0,gj=T.y/CS|0,tight=gi>=0&&gi<GW&&gj>=0&&gj<GH&&ROOMY[gj*GW+gi];
   if(active&&!estop&&low&&!path.length&&!recovery&&tight)noPathT+=dt;else noPathT=0;
   if(noPathT>3&&!recovery&&trail.length>=3&&retreats<4){recovery='RETREAT';retreats++;retreatD=0;recoveryT=0;noPathT=0}}
  if(stuckTime>STUCK_TRIGGER && !recovery){
    // First try to back out the way we came in (known-good states). Spinning/aligning inside a tight gap can't work.
    if(trail.length>=3&&retreats<4&&time-lastRecEnd<8){recovery='RETREAT';retreats++;retreatD=0}else recovery='ALIGN';
    recoveryT=0;stuckTime=STUCK_TRIGGER}
  if(recovery){
    recoveryT+=dt;
    const lim = recovery==='RETREAT'?14:recovery==='SPIN'?REC_SPIN_T:(recovery==='ALIGN'?REC_ALIGN_T:REC_BACKOFF_T);
    if(recoveryT>lim){
      if(recovery==='ALIGN'){recovery='BACKOFF';recoveryT=0}
      else if(recovery==='BACKOFF'){recovery='SPIN';recoveryT=0}
      else{recovery=null;recoveryT=0;stuckTime=0;lastRecEnd=time}
    }
  } else if(!active||estop){stuckTime=0;recoveryT=0}
  $('rc').textContent=recovery?recovery:'–';
}

batStep(dt);

// Optional auto-return: 10 s after it has arrived, parked and finished raising
if($('ar').checked&&didArrive&&mode==='user'&&!active&&!estop&&parkTh===null){
  arT+=dt;
  if(arT>=10)startReturn();
  else setSt('Ready - returning to start in '+Math.ceil(10-arT)+' s','ok');
}else arT=0;

// Desk height: the robot never moves it any more. It only follows the height setting if YOU change it (manual desk).
{
  const dh=userH-deskH, still=Math.hypot(T.vx,T.vy)<3&&Math.abs(T.w)<.05;
  if(!estop&&Math.abs(dh)>1e-4&&still)deskH+=Math.sign(dh)*Math.min(Math.abs(dh),LIFT*dt);
  if(!active&&!estop&&readyPending){
    if(parkTh!==null)setSt(mode==='home'?'Parking - turning to face forward at the start':'Parking - turning keyboard side toward you','');
    else{setSt((mode==='home'?(charging?'Ready - docked and charging, desk at ':'Ready - back at start, desk at '):'Ready - parked facing you, desk at ')+Math.round(deskH*100)+' cm','ok');readyPending=false}
  }
}

// ---- Manual drive override (from the phone Drive sim tab) ----
// Drop held joystick input if phone/API updates disappear; a lost connection must not
// leave the simulator driving on the last command indefinitely. This is simulation-only.
if(manualDrive.enabled && manualDrive.lastCommandAt && Date.now()-manualDrive.lastCommandAt>750){
  manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;
  setSt('Manual drive timed out — input lost','warn');
}
// Robot-frame: joystick "up" moves the desk toward its FRONT arrow (local -y).
// Runs before the supervisor, so swept-footprint and yaw checks still apply.
if(manualDrive.enabled && !estop && !active && !recovery && parkTh===null && !cordLock()){
  const mspd=VMAX*Math.max(0,Math.min(1,manualDrive.speed));
  const ix=Math.max(-1,Math.min(1,manualDrive.x));
  const iy=Math.max(-1,Math.min(1,manualDrive.y));
  const cosT=Math.cos(T.th), sinT=Math.sin(T.th);
  cx=(cosT*ix + sinT*iy)*mspd;
  cy=(sinT*ix - cosT*iy)*mspd;
  w =Math.max(-1,Math.min(1,manualDrive.turn))*YAW_RATE_MAX*0.6;
  if(Math.abs(cx)>.5||Math.abs(cy)>.5||Math.abs(w)>.05)setSt('Manual drive (simulation)','');
}

// Safety supervisor (swept footprint vs live lidar points)
let held=false;
[cx,cy,w,held]=supervise(cx,cy,w);
if(held&&active&&!estop&&!recovery&&low&&du>(mode==='user'?80:10))setSt('Safety hold - something in the swept path','warn');

// Charging-cord lock: while plugged in (interlock on) the wheels get NO command, every frame, whatever else asked for motion.
// A trip that is under way when the cord goes in is cancelled and the desk brakes hard (the cord is attached).
if(cordLock()){
  cx=0;cy=0;w=0;held=true;parkTh=null;
  if(active){active=false;path=[];setSt('Cord plugged in: drive disabled, trip cancelled','bad');alertT=4}
}

// Integrate swerve
const a=(Math.hypot(cx,cy)<Math.hypot(T.vx,T.vy)?(held?EMERG_DEC:60):25)*dt,ddx=cx-T.vx,ddy=cy-T.vy,dd=Math.hypot(ddx,ddy);
if(dd>a&&dd>1e-6){T.vx+=ddx/dd*a;T.vy+=ddy/dd*a}else{T.vx=cx;T.vy=cy}
T.w+=(w-T.w)*Math.min(1,dt*8);
{const ex=HW*Math.abs(Math.cos(T.th))+HH*Math.abs(Math.sin(T.th)),ey=HW*Math.abs(Math.sin(T.th))+HH*Math.abs(Math.cos(T.th));
 T.x=Math.max(ex,Math.min(W-ex,T.x+T.vx*dt));
 T.y=Math.max(ey,Math.min(H-ey,T.y+T.vy*dt));}
T.th+=T.w*dt;
{const ax=Math.hypot(T.vx-(T._px||0),T.vy-(T._py||0))/Math.max(dt,1e-3)/50;T._px=T.vx;T._py=T.vy;if(ax>peakA&&ax<20)peakA=ax}

const gap=clearance(T.x,T.y)-TR;
if(gap<minGap)minGap=gap;
const fh=footprintHit();
if(fh&&!inC){contacts++;inC=true}else if(!fh)inC=false;   // real rectangle, not the old 34 px circle

$('pa').textContent=peakA.toFixed(2);
const slipRatio=SLIP_A/Math.max(peakA,0.01);
$('sl').textContent=slipRatio.toFixed(1)+'×';
$('sl').className = slipRatio>3?'ok':(slipRatio>1.5?'warn':'bad');
$('es').textContent=estop?'Release E-stop':'E-STOP';
$('dh').textContent=Math.round(deskH*100)+' cm';
$('ct').textContent=contacts;$('sh').textContent=safetyHolds;
$('rb').textContent=bat.r.toFixed(0)+'%';$('rb').className=bat.r<15?'bad':bat.r<30?'warn':'ok';
$('ab').textContent=bat.a.toFixed(0)+'%';$('ab').className=bat.a<15?'bad':bat.a<30?'warn':'ok';
{let ns=0,no=0;for(let i=0;i<GW*GH;i++){if(seen[i])ns++;if(smap[i]>0)no++}$('ex').textContent=Math.round(100*ns/(GW*GH))+'%';$('rm').textContent=no+' cells'}
$('cord').style.display=manualDock()?'':'none';$('cord').textContent=cordIn?'Unplug cord':'Plug in cord';
$('cs').textContent=manualDock()?((cordIn?'plugged in':'unplugged')+(breakaways?' ('+breakaways+' breakaway)':'')):'n/a (auto contacts)';
const needPlug=manualDock()&&dockedNow()&&!cordIn;
$('dk').textContent=needPlug?'docked: PLUG ME IN':charging?((bat.r>=99.5&&bat.a>=99.5)?'docked, full':'⚡ charging'):'off the dock';$('dk').className=needPlug?'warn':charging?'ok':'';$('ct').className=contacts?'bad':'ok';
$('gp').textContent=minGap>1e8?'–':Math.round(minGap*2)+' cm';
$('hg').textContent=(Math.round((headTarget*180/Math.PI+360)%360)+'°'+(headState==='SETTLED'?' ✓':' →'));
$('pp').textContent = peopleInPathCount>0 ? peopleInPathCount+' (waiting)' : '0';
}

function draw(){
  const S=getComputedStyle(document.documentElement),g=n=>S.getPropertyValue(n).trim();
  const obs=g('--obs'),hitc=g('--hit'),acc=g('--acc'),tbl=g('--tbl'),fg=g('--fg');
  ctx.setTransform(2,0,0,2,0,0);ctx.clearRect(0,0,W,H);
  const floors={office:'#e8e5dc',wood:'#c69a68',tile:'#f4f2e9',warehouse:'#b9b8b2',dark:'#30343b'};
  const floor=floors[roomStyle]||floors.office;ctx.fillStyle=floor;ctx.fillRect(0,0,W,H);
  // Room finish / subtle floor pattern; purely visual, never part of collision geometry.
  if(roomStyle==='tile'||roomStyle==='warehouse'){
    ctx.strokeStyle=roomStyle==='tile'?'rgba(80,80,80,.12)':'rgba(40,40,40,.09)';ctx.lineWidth=.7;ctx.beginPath();
    const step=roomStyle==='tile'?50:80;for(let x=0;x<=W;x+=step){ctx.moveTo(x,0);ctx.lineTo(x,H)}for(let y=0;y<=H;y+=step){ctx.moveTo(0,y);ctx.lineTo(W,y)}ctx.stroke();
  } else if(roomStyle==='wood'){
    ctx.strokeStyle='rgba(80,45,20,.12)';ctx.lineWidth=1;for(let y=0;y<H;y+=28){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke()}
  }
  ctx.strokeStyle=roomStyle==='dark'?'#707985':'rgba(70,70,70,.45)';ctx.lineWidth=5;ctx.strokeRect(2.5,2.5,W-5,H-5);
  // Obstacles. Custom blocks are visually distinct and selectable in editor mode.
  for(const r of rects){ctx.fillStyle=r._custom?(r===selectedBlock?'#f59e0b':'#d97706'):obs;ctx.fillRect(r.x,r.y,r.w,r.h);if(r._custom){ctx.strokeStyle=r===selectedBlock?'#7c2d12':'rgba(70,35,0,.8)';ctx.lineWidth=1.5;ctx.strokeRect(r.x+.5,r.y+.5,r.w-1,r.h-1)}}
  for(const c of circs){ctx.beginPath();ctx.arc(c.x,c.y,c.r,0,7);ctx.fill()}
  // Costmap
  if($('cost').checked){   // what it knows: grey = never seen, orange = busy (people), purple = remembered obstacle, red = lidar right now
    ctx.globalAlpha=.12;ctx.fillStyle='#808898';for(let i=0;i<GW*GH;i++)if(!seen[i])ctx.fillRect(i%GW*CS,(i/GW|0)*CS,CS,CS);
    ctx.fillStyle='#f28c28';for(let i=0;i<GW*GH;i++)if(traffic[i]>.4){ctx.globalAlpha=Math.min(.5,traffic[i]/25);ctx.fillRect(i%GW*CS,(i/GW|0)*CS,CS,CS)}
    ctx.globalAlpha=.4;ctx.fillStyle='#9b59d6';for(let i=0;i<GW*GH;i++)if(smap[i]>0)ctx.fillRect(i%GW*CS,(i/GW|0)*CS,CS,CS);
    ctx.globalAlpha=.35;ctx.fillStyle=hitc;for(let i=0;i<GW*GH;i++)if(time-last[i]<1.2)ctx.fillRect(i%GW*CS,(i/GW|0)*CS,CS,CS);ctx.globalAlpha=1}
  // Lidar rays
  if($('ray').checked){
    ctx.strokeStyle=acc;ctx.globalAlpha=.13;ctx.lineWidth=1;
    ctx.beginPath();for(const h of hits){ctx.moveTo(h.ox,h.oy);ctx.lineTo(h.x,h.y)}ctx.stroke();
    ctx.globalAlpha=1;
    for(const h of hits)if(h.hit){
      ctx.fillStyle = h.dyn ? '#d62fd6' : (h.settledPerson ? '#f5c400' : hitc);
      ctx.fillRect(h.x-1.5,h.y-1.5,3,3);
    }
  }
  // People: shape by type, color by state
  for(const p of people){
    if(p.type==='sitter'){
      // Chair hint
      ctx.fillStyle='rgba(140,110,80,0.30)';
      ctx.beginPath();ctx.roundRect(p.x-18,p.y-18,36,36,4);ctx.fill();
    }
    const col = p.settled ? '#e0561f' : (p.stillTime>1 ? '#f5c400' : '#d62fd6');
    ctx.fillStyle = col;
    ctx.beginPath();
    if(p.type==='sitter'){
      ctx.roundRect(p.x-p.r*0.75, p.y-p.r*0.75, p.r*1.5, p.r*1.5, 4);
    } else {
      ctx.arc(p.x, p.y, p.r, 0, 7);
    }
    ctx.fill();
    // Settling ring for non-settled non-moving people
    if(!p.settled && p.stillTime>0 && p.stillTime<STAND_WAIT){
      ctx.strokeStyle='#f5c400';ctx.lineWidth=2;
      ctx.beginPath();ctx.arc(p.x,p.y,p.r+3,0,7*Math.min(1,p.stillTime/STAND_WAIT));ctx.stroke();
    }
    // Small label
    ctx.fillStyle=fg;ctx.font='9px system-ui';ctx.textAlign='center';
    ctx.fillText(p.type==='sitter'?'sit':(p.type==='talker'?'talk':'walk'), p.x, p.y - p.r - 4);
  }
  // Path
  if(path.length&&active){ctx.strokeStyle=acc;ctx.lineWidth=2;ctx.setLineDash([6,5]);ctx.beginPath();path.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.stroke();ctx.setLineDash([]);
    // ghost desk where the plan rotates it
    ctx.lineWidth=1.5;ctx.strokeStyle=acc;ctx.globalAlpha=.55;
    for(let j=1;j<path.length;j++){
      const nxt=path[j+1];
      if(path[j].k!==path[j-1].k&&!(nxt&&nxt.k!==path[j].k)){
        ctx.save();ctx.translate(path[j].x,path[j].y);ctx.rotate(path[j].th);ctx.beginPath();ctx.roundRect(-HW,-HH,2*HW,2*HH,5);ctx.stroke();ctx.restore();
      }
    }
    ctx.globalAlpha=1}
  // User
  if(mode==='tour'&&active){tourPts().forEach((p,i)=>{ctx.beginPath();ctx.arc(p[0],p[1],i===tourI?7:4,0,7);ctx.strokeStyle=acc;ctx.lineWidth=i===tourI?2.5:1.5;ctx.stroke()})}
  // charging station: wall socket + cord with breakaway connector
  if(manualDock()){
    ctx.save();ctx.fillStyle=fg;ctx.globalAlpha=.7;ctx.fillRect(0,58,7,26);ctx.restore();
    if(cordIn){
      const dxc=T.x-6,dyc=T.y-71,dc=Math.hypot(dxc,dyc),sag=Math.max(0,28-dc*.5);
      ctx.save();ctx.strokeStyle=dc>38?'#d64545':'#222';ctx.lineWidth=2.5;ctx.beginPath();ctx.moveTo(7,71);
      ctx.quadraticCurveTo((7+T.x)/2,(71+T.y)/2+sag,T.x,T.y);ctx.stroke();ctx.restore();
    }
  }
  if(alertT>0){ctx.save();ctx.strokeStyle='#f0a000';ctx.lineWidth=4;ctx.globalAlpha=.4+.5*Math.abs(Math.sin(time*8));ctx.beginPath();ctx.arc(T.x,T.y,HW+14,0,7);ctx.stroke();ctx.restore()}
  // start spot: dashed desk outline with the forward arrow (monitor back)
  ctx.save();ctx.translate(HOME.x,HOME.y);ctx.globalAlpha=.5;ctx.strokeStyle=fg;ctx.lineWidth=1.5;ctx.setLineDash([5,4]);
  ctx.beginPath();ctx.roundRect(-HW,-HH,2*HW,2*HH,5);ctx.stroke();ctx.setLineDash([]);
  ctx.fillStyle=fg;ctx.beginPath();ctx.moveTo(0,-HH+1.5);ctx.lineTo(-5.5,-HH+9.5);ctx.lineTo(5.5,-HH+9.5);ctx.fill();
  ctx.font='10px system-ui';ctx.textAlign='center';ctx.fillText('start / dock',0,HH+12);ctx.restore();
  // dock pad: charging coil/contacts under the start spot; pulses while charging
  {const okc=getComputedStyle(document.documentElement).getPropertyValue('--ok').trim()||'#1f9d55';
   ctx.save();ctx.translate(HOME.x,HOME.y);
   ctx.globalAlpha=charging?.3+.2*Math.sin(time*5):.1;ctx.fillStyle=okc;ctx.beginPath();ctx.roundRect(-HW+4,-HH+4,2*HW-8,2*HH-8,5);ctx.fill();
   if(charging){ctx.globalAlpha=.35+.3*Math.sin(time*5);ctx.strokeStyle=okc;ctx.lineWidth=3;ctx.beginPath();ctx.arc(0,0,HW+9,0,7);ctx.stroke()}
   ctx.globalAlpha=.9;ctx.fillStyle=charging?okc:fg;
   ctx.beginPath();ctx.moveTo(14,-9);ctx.lineTo(8,2);ctx.lineTo(12,2);ctx.lineTo(10,11);ctx.lineTo(18,-2);ctx.lineTo(14,-2);ctx.closePath();ctx.fill();
   ctx.restore()}
  ctx.fillStyle='#f28c28';ctx.beginPath();ctx.arc(user.x,user.y,user.r,0,7);ctx.fill();
  ctx.fillStyle=fg;ctx.font='11px system-ui';ctx.textAlign='center';ctx.fillText('you',user.x,user.y-19);
  // Robot
  ctx.save();ctx.translate(T.x,T.y);ctx.rotate(T.th);
  ctx.fillStyle=recovery?g('--warn')||'#b07800':tbl;
  ctx.strokeStyle=fg;ctx.lineWidth=1.5;
  ctx.beginPath();ctx.roundRect(-HW,-HH,2*HW,2*HH,5);ctx.fill();ctx.stroke();
  // Forward = back of the monitor (local -y, up when th=0). The keyboard side (+y) is where you stand.
  ctx.fillStyle='rgba(20,22,26,.9)';ctx.fillRect(-3,-9,26,4.5);            // monitor seen from above
  ctx.fillStyle='rgba(255,255,255,.6)';ctx.fillRect(7,6,17,6);             // keyboard
  ctx.fillStyle=fg;ctx.beginPath();ctx.moveTo(0,-HH+1.5);ctx.lineTo(-5.5,-HH+9.5);ctx.lineTo(5.5,-HH+9.5);ctx.fill();   // forward arrow
  ctx.fillStyle='#f5c400';ctx.beginPath();ctx.arc(ESX,0,8,0,7);ctx.fill();
  ctx.fillStyle=estop?'#7a0f0f':'#e5221f';ctx.beginPath();ctx.arc(ESX,0,estop?4.5:5.5,0,7);ctx.fill();
  if(estop){ctx.strokeStyle='#ff2d2d';ctx.lineWidth=2.5;ctx.beginPath();ctx.roundRect(-HW-3,-HH-3,2*HW+6,2*HH+6,7);ctx.stroke()}
  ctx.fillStyle='#30d0ff';for(const L of LIDARS){ctx.beginPath();ctx.arc(L[0],L[1],3.2,0,7);ctx.fill()}
  ctx.restore();
  // Swerve modules
  const co=Math.cos(T.th),si=Math.sin(T.th);let k=0;
  for(const[lx,ly]of[[22,13],[22,-13],[-22,13],[-22,-13]]){
    const rx=co*lx-si*ly,ry=si*lx+co*ly,mx=T.x+rx,my=T.y+ry;
    const vx=T.vx-T.w*ry,vy=T.vy+T.w*rx,sp=Math.hypot(vx,vy);
    if(sp>3)modAng[k]=Math.atan2(vy,vx);
    ctx.save();ctx.translate(mx,my);ctx.rotate(modAng[k]);
    ctx.fillStyle='#222';ctx.strokeStyle=acc;ctx.lineWidth=1.5;
    ctx.beginPath();ctx.roundRect(-7,-3,14,6,2);ctx.fill();ctx.stroke();
    if(sp>3){ctx.strokeStyle='#3ddc84';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(sp*.25,0);ctx.stroke()}
    ctx.restore();k++;
  }
  ctx.fillStyle=fg;ctx.font='11px system-ui';ctx.textAlign='center';
  ctx.fillText(Math.round(deskH*100)+' cm',T.x,T.y+TR+14);
}

let lt=performance.now();
function loop(n){const dt=Math.max(0,Math.min(.05,(n-lt)/1000));lt=n;let rem=dt*warp;while(rem>1e-6){const s=Math.min(rem,1/60);update(s);rem-=s}draw();requestAnimationFrame(loop)}
function pos(e){const r=cv.getBoundingClientRect();return{x:(e.clientX-r.left)/r.width*W,y:(e.clientY-r.top)/r.height*H}}
cv.addEventListener('pointerdown',e=>{
  const p=pos(e);
  if(placeBlocks){
    const w=Math.max(10,Math.min(300,Number($('blockW').value)||60)),h=Math.max(10,Math.min(300,Number($('blockH').value)||40));
    const b={x:Math.max(0,Math.min(W-w,p.x-w/2)),y:Math.max(0,Math.min(H-h,p.y-h/2)),w,h,_custom:true};rects.push(b);selectedBlock=b;cv.setPointerCapture(e.pointerId);$('editHint').textContent='Block added. Click Place blocks to stop adding, or add more blocks.';return;
  }
  if(Math.hypot(p.x-T.x-Math.cos(T.th)*ESX,p.y-T.y-Math.sin(T.th)*ESX)<13){toggleEstop();return}
  const b=[...rects].reverse().find(r=>r._custom&&p.x>=r.x&&p.x<=r.x+r.w&&p.y>=r.y&&p.y<=r.y+r.h);
  if(b){selectedBlock=b;blockDrag={b,dx:p.x-b.x,dy:p.y-b.y};cv.setPointerCapture(e.pointerId);return}
  if(Math.hypot(p.x-user.x,p.y-user.y)<32){drag=true;cv.setPointerCapture(e.pointerId)}
});
cv.addEventListener('pointermove',e=>{const p=pos(e);if(blockDrag){const b=blockDrag.b;b.x=Math.max(0,Math.min(W-b.w,p.x-blockDrag.dx));b.y=Math.max(0,Math.min(H-b.h,p.y-blockDrag.dy));return}if(!drag)return;user.x=Math.max(20,Math.min(W-20,p.x));user.y=Math.max(20,Math.min(H-20,p.y))});
function finishCanvasDrag(){drag=false;blockDrag=null}
cv.addEventListener('pointerup',finishCanvasDrag);cv.addEventListener('pointercancel',finishCanvasDrag);

$('go').onclick=()=>{manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;if(cordBlocks())return;if(bat.r<8&&!estop){startReturn();setSt('Battery too low - returning to the dock instead','warn');return}parkTh=null;mode='user';didArrive=false;arT=0;trail=[];retreats=0;waitT=0;if(estop){setSt('E-stop engaged - release it first','bad');return}active=true;peakA=0;rp=0;minGap=1e9;stuckTime=0;recovery=null;recoveryT=0;headTarget=T.th;headState='SETTLED';setSt('Request sent…','')};
$('back').onclick=startReturn;
$('stop').onclick=()=>{manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;didArrive=false;arT=0;active=false;path=[];recovery=null;stuckTime=0;setSt('Stopped','warn')};
$('rst').onclick=()=>{forgetMap();sceneKey='default';reset();clearMapArrays();loadMap()};
$('learn').onclick=startTour;
$('cord').onclick=()=>{
  if(!cordIn){if(Math.hypot(T.x-HOME.x,T.y-HOME.y)>45){setSt('The cord only reaches the dock: bring the desk home first','warn');return}cordIn=true;setSt('Cord plugged in','ok')}
  else{cordIn=false;setSt('Cord unplugged','ok')}
};
$('forget').onclick=()=>{forgetMap();setSt('Map forgotten: it will plan through unknown space again until it relearns','')};
// Slot test: a full-width wall with one 70 px slot (the desk is 56 x 36 px). The only way through is to turn the
// desk so its long axis points through the slot.
function buildSlotTest(){
  const gap=Math.max(56,Math.min(160,Number($('slotGap').value)||70)),dir=$('slotDirection').value;
  reset();sceneKey='slot';clearMapArrays();loadMap();rects.length=0;
  if(dir==='horizontal'){const gx=(W-gap)/2;rects.push({x:0,y:240,w:gx,h:20},{x:gx+gap,y:240,w:W-gx-gap,h:20});T.x=400;T.y=90;user.x=700;user.y=430;}
  else{const gy=(H-gap)/2;rects.push({x:380,y:0,w:20,h:gy},{x:380,y:gy+gap,w:20,h:H-gy-gap});T.x=100;T.y=250;user.x=700;user.y=250;}
  circs.length=0;people=[];setSt('Slot test ('+gap+' px, '+dir+'): press Come to me. Watch the desk orientation as it crosses the gap.','');
}
$('slot').onclick=buildSlotTest;$('runSlot').onclick=buildSlotTest;
$('roomStyle').onchange=()=>{roomStyle=$('roomStyle').value};
$('placeMode').onclick=()=>{placeBlocks=!placeBlocks;blockDrag=null;$('placeMode').textContent='Place blocks: '+(placeBlocks?'on':'off');$('editHint').textContent=placeBlocks?'Click anywhere in the room to add a block using the width and height above.':'Click and drag an orange block to reposition it. Added blocks affect route planning and collision checks.'};
$('deleteBlock').onclick=()=>{if(!selectedBlock){$('editHint').textContent='Select an orange added block by clicking or dragging it first.';return}const i=rects.indexOf(selectedBlock);if(i>=0)rects.splice(i,1);selectedBlock=null;blockDrag=null};
$('clearBlocks').onclick=()=>{for(let i=rects.length-1;i>=0;i--)if(rects[i]._custom)rects.splice(i,1);selectedBlock=null;blockDrag=null;$('editHint').textContent='All added blocks removed. Built-in room obstacles remain.'};

function toggleEstop(){estop=!estop;parkTh=null;didArrive=false;if(estop){active=false;path=[];T.vx=T.vy=T.w=0;manualDrive={...manualDrive,x:0,y:0,turn:0};recovery=null;stuckTime=0}else setSt('E-stop released - press Come to me','')}
addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();if(document.activeElement&&document.activeElement.blur)document.activeElement.blur();if(!estop)toggleEstop()}});
function ahead(dd){for(const q of path)if(Math.hypot(q.x-T.x,q.y-T.y)>=dd)return q;return{x:T.x+Math.cos(T.th)*dd,y:T.y+Math.sin(T.th)*dd}}
$('es').onclick=toggleEstop;
$('wp').onchange=()=>warp=+$('wp').value;
$('hs').oninput=()=>{userH=$('hs').value/100;$('hv').textContent=$('hs').value};

$('add').onclick=()=>{
  for(let i=0;i<50;i++){
    const x=40+Math.random()*(W-80),y=40+Math.random()*(H-80);
    if(!hitTest(x,y)&&Math.hypot(x-T.x,y-T.y)>120){people.push(mkWalker(x,y));break}
  }
};
$('obs').onclick=()=>{
  for(const dd of[130,110,150,170]){
    const q=ahead(dd);
    if(q.x>50&&q.x<W-50&&q.y>50&&q.y<H-50&&clearance(q.x,q.y)>20){circs.push({x:q.x,y:q.y,r:16});return}
  }
};
$('walk').onclick=()=>{
  const q=ahead(130),da=Math.atan2(q.y-T.y,q.x-T.x),nx=-Math.sin(da),ny=Math.cos(da);
  const cl=(v,m)=>Math.max(30,Math.min(m-30,v));
  const p=mkWalker(cl(q.x+nx*60,W),cl(q.y+ny*60,H));
  p.tx=cl(q.x-nx*170,W); p.ty=cl(q.y-ny*170,H);
  p.sp=14; p.t=12;
  people.push(p);
};
$('sit').onclick=()=>{
  const q=ahead(130),da=Math.atan2(q.y-T.y,q.x-T.x),nx=-Math.sin(da),ny=Math.cos(da);
  const cl=(v,m)=>Math.max(30,Math.min(m-30,v));
  const p=mkWalker(cl(q.x+nx*70,W),cl(q.y+ny*70,H));
  p.tx=q.x; p.ty=q.y; p.sp=18; p.t=999;
  p.convertOnArrival=true;   // becomes a sitter on arrival, then waits 5 s
  people.push(p);
};
$('talk').onclick=()=>{
  const q=ahead(150),da=Math.atan2(q.y-T.y,q.x-T.x),nx=-Math.sin(da),ny=Math.cos(da);
  const cl=(v,m)=>Math.max(30,Math.min(m-30,v));
  const cx1=cl(q.x+nx*20,W),cy1=cl(q.y+ny*20,H);
  const cx2=cl(q.x-nx*20,W),cy2=cl(q.y-ny*20,H);
  people.push(mkTalker(cx1,cy1));
  people.push(mkTalker(cx2,cy2));
};

reset();loadMap();requestAnimationFrame(loop);

