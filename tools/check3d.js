// Connectivity check for the 3D concept model: builds it with a mock THREE, then verifies every part touches
// another part and that the whole assembly is connected to the ground, at several desk heights.
const fs=require('fs');
const html=fs.readFileSync(process.argv[2],'utf8');
const a=html.indexOf("try{const el=$('m')"),b=html.indexOf("}catch(err){$('m').textContent='3D preview needs WebGL.'}",a);
const block=html.slice(a,b+"}catch(err){$('m').textContent='3D preview needs WebGL.'}".length);
class Vec{constructor(){this.x=0;this.y=0;this.z=0}set(x,y,z){this.x=x;this.y=y;this.z=z;return this}}
class Obj{constructor(){this.position=new Vec();this.rotation=new Vec();this.scale={x:1,y:1,z:1};this.children=[];this.parent=null}add(...c){for(const o of c){o.parent=this;this.children.push(o)}}}
const THREE={Scene:class extends Obj{},Group:class extends Obj{},Mesh:class extends Obj{constructor(g,m){super();this.geometry=g;this.material=m}},
 BoxGeometry:class{constructor(w,h,d){this.type='box';this.w=w;this.h=h;this.d=d}},CylinderGeometry:class{constructor(rt,rb,h){this.type='cyl';this.r=Math.max(rt,rb);this.h=h}},
 MeshStandardMaterial:class{constructor(o){this.o=o}},AmbientLight:class extends Obj{},DirectionalLight:class extends Obj{},
 PerspectiveCamera:class extends Obj{updateProjectionMatrix(){}lookAt(){}},WebGLRenderer:class{constructor(){this.domElement={style:{},setPointerCapture(){}}}setPixelRatio(){}setSize(){}render(){}}};
let raf=null,scene=null;
const origScene=THREE.Scene;THREE.Scene=class extends origScene{constructor(){super();scene=this}};
const $=id=>({appendChild(){},style:{},clientWidth:800,clientHeight:400,textContent:''});
const state={deskH:1.05};
const run=new Function('THREE','$','state','devicePixelRatio','addEventListener','requestAnimationFrame',
  'let deskH=state.deskH;state.set=v=>{deskH=v};\n'+block);
run(THREE,$,state,1,()=>{},f=>{raf=f});
function boxOf(m){
  const g=m.geometry;let hx,hy,hz;
  if(g.type==='box'){hx=g.w/2*m.scale.x;hy=g.h/2*m.scale.y;hz=g.d/2*m.scale.z}
  else{hx=g.r;hy=g.h/2*m.scale.y;hz=g.r; if(Math.abs(m.rotation.z-Math.PI/2)<0.01){[hx,hy]=[hy,hx]}}
  let x=m.position.x,y=m.position.y,z=m.position.z;
  for(let p=m.parent;p&&p.geometry;p=p.parent){x+=p.position.x;y+=p.position.y;z+=p.position.z}   // children of the desktop
  return [x-hx,y-hy,z-hz,x+hx,y+hy,z+hz];
}
function meshes(o,acc=[]){for(const c of o.children){if(c.geometry)acc.push(c);meshes(c,acc)}return acc}
const TOL=0.006;let anyBad=false;
for(const dh of [0.63,0.85,1.05,1.28]){
  state.set(dh);raf();
  const ms=meshes(scene).filter(m=>!(m.geometry.type==='cyl'&&m.geometry.r>1)); // ignore the ground disc
  const bx=ms.map(boxOf),n=ms.length,adj=Array.from({length:n},()=>[]);
  for(let i=0;i<n;i++)for(let j=i+1;j<n;j++){
    const A=bx[i],B=bx[j];
    if(A[0]-TOL<=B[3]&&B[0]-TOL<=A[3]&&A[1]-TOL<=B[4]&&B[1]-TOL<=A[4]&&A[2]-TOL<=B[5]&&B[2]-TOL<=A[5]){adj[i].push(j);adj[j].push(i)}
  }
  const seen=new Array(n).fill(false),q=[];
  for(let i=0;i<n;i++)if(bx[i][1]<=TOL){seen[i]=true;q.push(i)}      // touches the floor
  while(q.length){const i=q.pop();for(const j of adj[i])if(!seen[j]){seen[j]=true;q.push(j)}}
  const bad=[];for(let i=0;i<n;i++)if(!seen[i]){const g=ms[i].geometry;bad.push((g.type==='box'?'box '+[g.w,g.h,g.d].map(v=>v.toFixed(3)).join('x'):'cyl r'+g.r.toFixed(3)+' h'+g.h.toFixed(3))+' @y'+((bx[i][1]+bx[i][4])/2).toFixed(2))}
  const desk=ms.find(m=>m.geometry.type==='box'&&Math.abs(m.geometry.w-1.1)<.001&&Math.abs(m.geometry.d-.7)<.001);
  console.log('deskH',dh.toFixed(2),'| desktop underside y',(boxOf(desk)[1]).toFixed(2),'| parts',n,'| NOT connected to the ground:',bad.length?bad.join(' ; '):'none');
  if(bad.length)anyBad=true;
}
console.log(anyBad?'RESULT: floating parts found':'RESULT: every part is connected at every height');
process.exit(anyBad?1:0);
