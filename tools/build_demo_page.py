"""Build one hostable page: the simulator plus the real phone app, wired together in the page.

The phone app (mobile/index.html) runs unchanged inside an iframe. Its fetch() calls to /v1/... are
posted to the parent page, where a small in-page bridge answers them from the live simulator using the
same rules as bridge/bridge_core.js (422 off-map, 409 cord interlock, 409 duplicate navigation).
Usage: python3 tools/build_demo_page.py . demo.html
"""
import base64, re, sys, pathlib

root, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
sim = (root / "robot/simulator/summon_table_sim.html").read_text(encoding="utf8")
app = (root / "mobile/index.html").read_text(encoding="utf8")

def sub1(s, old, new):
    assert s.count(old) == 1, old[:60]
    return s.replace(old, new)

# ---- simulator: strip the document wrapper (the host adds it) and its bridge-sync script ----
sim = sub1(sim, '<!doctype html>\n<html lang="en"><head><meta charset="utf-8">\n', "")
sim = re.sub(r'<meta name="viewport"[^>]*>\n', "", sim, count=1)
sim = sub1(sim, "<title>Summon Table – swerve + lidar simulator</title>", "<title>Summon Desk Simulator</title>")
i = sim.index("</script><script>\n// Summon Desk live bridge")
j = sim.index("</script>\n</body></html>")
sim = sim[:i] + "</script>\n" + "@@BRIDGE@@" + sim[j + len("</script>\n</body></html>"):]

# ---- layout: simulator on the left, phone on the right (stacked on narrow screens) ----
sim = sub1(sim, "</style></head><body><main>", """
.duo{display:flex;flex-wrap:wrap;gap:20px;align-items:flex-start;justify-content:center;max-width:1320px;margin:0 auto}
.duo>main{flex:1 1 560px;min-width:0;max-width:860px;margin:0}
.phone-col{flex:0 1 400px;min-width:0;padding:18px 14px 28px;position:sticky;top:env(safe-area-inset-top,0px)}
.phone-col h2{margin:0 0 4px}
.phone-col p{color:var(--mut);font-size:13px;margin:0 0 10px}
.bezel{border:10px solid var(--fg);border-radius:34px;overflow:hidden;background:var(--fg);box-shadow:0 12px 30px rgba(0,0,0,.18)}
.bezel iframe{display:block;width:100%;height:min(780px,82vh);border:0;background:#080f12}
@media (max-width:1000px){.phone-col{position:static;flex-basis:100%;max-width:440px}}
</style><div class="duo"><main>""")
sim = sub1(sim, "</main>", """</main>
<aside class="phone-col" aria-label="Phone app">
<h2>Phone app</h2>
<p>Your mobile controller (mobile/index.html), connected to the simulator on the left. Summon, Return, Stop, Drive sim and LiDAR 3D all act on that simulator.</p>
<div class="bezel"><iframe id="phoneApp" title="Summon Desk phone app"></iframe></div>
</aside></div>""")

# ---- phone app: drop the manifest, hide bridge-only links, route fetch() to the parent page ----
shim = """<script>
(()=>{let n=0;const waiting=new Map();
addEventListener('message',e=>{const m=e.data;if(!m||m.kind!=='bridge-reply'||!waiting.has(m.id))return;
  waiting.get(m.id)(new Response(JSON.stringify(m.body),{status:m.status,headers:{'Content-Type':'application/json'}}));waiting.delete(m.id)});
window.fetch=(path,opts={})=>new Promise(resolve=>{const id=++n;waiting.set(id,resolve);
  let body;try{body=opts.body?JSON.parse(opts.body):undefined}catch(_){body=null}
  parent.postMessage({kind:'bridge-request',id,method:(opts.method||'GET').toUpperCase(),path:String(path).split('?')[0],body},'*')});
})();
</script>
<style>.links,.settings-card{display:none!important}</style>"""
app = re.sub(r'<link rel="manifest"[^>]*>', "", app, count=1)
app = sub1(app, "<title>", shim + "<title>")
app = sub1(app, "Summon Desk · local network only · simulation mode", "Summon Desk · simulation mode")
app_b64 = base64.b64encode(app.encode("utf8")).decode("ascii")

bridge = """<script>
// In-page bridge: answers the phone app's /v1 requests from this simulator, with bridge_core.js's rules.
(()=>{
  'use strict';
  const PX=.02,EPOCH=1;let lastNav='',lastNavAt=0;
  function state(){let seenCount=0,remembered=0;for(let i=0;i<GW*GH;i++){if(seen[i])seenCount++;if(smap[i]>0)remembered++}
    return {time:+time.toFixed(1),mode,status:$('st').textContent,severity:($('st').className||'info'),active,estop,
      pose:{x:+(T.x*PX).toFixed(3),y:+(T.y*PX).toFixed(3),theta:+T.th.toFixed(3)},user:{x:+(user.x*PX).toFixed(3),y:+(user.y*PX).toFixed(3)},
      battery:{robot_pct:+bat.r.toFixed(1),ups_pct:+bat.a.toFixed(1)},charging,cord:{plugged:cordIn,interlock_on:!!$('il').checked,breakaways},
      dock_type:$('dm').value,desk_height_cm:Math.round(deskH*100),alert:alertT>0,explored_pct:Math.round(100*seenCount/(GW*GH)),remembered_cells:remembered,
      people:people.map(p=>({x:+(p.x*PX).toFixed(2),y:+(p.y*PX).toFixed(2),settled:!!p.settled})),
      lidar:hits.filter(h=>h&&h.hit&&Number.isFinite(h.d)&&h.d>0).map(h=>({angle:+(((h.a-T.th)*180/Math.PI+540)%360-180).toFixed(2),range:+(h.d*PX).toFixed(3),type:h.type||'obstacle'})),
      ...extraStateJSON(),simulator_connected:true,epoch:EPOCH}}
  function map(){const rows=[];for(let y=0;y<GH;y++){let row='';for(let x=0;x<GW;x++){const i=y*GW+x;row+=smap[i]>0?'#':traffic[i]>2?'+':seen[i]?'_':'.'}rows.push(row)}
    return {width_m:W*PX,height_m:H*PX,cell_m:CS*PX,gw:GW,gh:GH,rows,epoch:EPOCH}}
  function apply(c){
    if(c.type==='summon'){manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;user.x=c.x/PX;user.y=c.y/PX;$('go').click()}
    else if(c.type==='drive'){active=false;path=[];parkTh=null;mode='manual';manualDrive.enabled=true;manualDrive.lastCommandAt=Date.now();manualDrive.x=c.x;manualDrive.y=c.y;manualDrive.turn=c.turn;manualDrive.speed=c.speed;if(estop){manualDrive.x=manualDrive.y=manualDrive.turn=0;setSt('E-STOP engaged — manual drive blocked','bad')}}
    else if(c.type==='return'){manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;$('back').click()}
    else if(c.type==='learn')$('learn').click();
    else if(c.type==='stop'){manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;$('stop').click()}
    else if(c.type==='cord'&&!!c.plugged!==cordIn)$('cord').click();
    else if(c.type==='zones')setZonesM(c.zones);
  }
  function handle(method,path,b){
    if(method==='GET'){
      if(path==='/v1/health')return [200,{ok:true,name:'in-page simulator bridge',version:1,epoch:EPOCH}];
      if(path==='/v1/state')return [200,state()];
      if(path==='/v1/map')return [200,map()];
      return [404,{error:'not found'}];
    }
    if(method!=='POST')return [404,{error:'not found'}];
    if(b===null)return [400,{error:'bad json'}];b=b||{};
    let c;
    if(path==='/v1/summon'){const x=+b.x,y=+b.y;
      if(!(Number.isFinite(x)&&Number.isFinite(y)&&x>=0.5&&x<=W*PX-0.5&&y>=0.5&&y<=H*PX-0.5))return [422,{accepted:false,status:'Target outside the room'}];
      c={type:'summon',x,y}}
    else if(path==='/v1/drive'){const v=[b.x,b.y,b.turn,b.speed].map(Number);
      if(!v.every(Number.isFinite))return [422,{accepted:false,status:'Drive values must be numbers'}];
      const cl=(n,lo,hi)=>Math.max(lo,Math.min(hi,n));c={type:'drive',x:cl(v[0],-1,1),y:cl(v[1],-1,1),turn:cl(v[2],-1,1),speed:cl(v[3],.08,1)}}
    else if(path==='/v1/return')c={type:'return'};
    else if(path==='/v1/learn')c={type:'learn'};
    else if(path==='/v1/stop')c={type:'stop'};
    else if(path==='/v1/sim/cord')c={type:'cord',plugged:!!b.plugged};
    else if(path==='/v1/zones'){if(!Array.isArray(b.zones)||b.zones.length>20)return [422,{accepted:false,status:'zones must be a list of at most 20'}];c={type:'zones',zones:b.zones}}
    else return [404,{error:'not found'}];
    if(['summon','drive','return','learn'].includes(c.type)&&cordIn&&$('il').checked)
      return [409,{accepted:false,status:'Movement refused: desk cord is plugged in. Unplug the cord before moving.',severity:'bad'}];
    if(['summon','return','learn'].includes(c.type)){const sig=c.type==='summon'?JSON.stringify(c):c.type,now=Date.now();
      if(sig===lastNav&&now-lastNavAt<2000)return [409,{accepted:false,status:'Duplicate navigation command ignored. Wait for the current command to finish.',severity:'warn'}];
      lastNav=sig;lastNavAt=now}
    if(c.type==='stop')lastNav='';
    apply(c);
    return [200,{accepted:true,status:'Command sent to simulator',severity:'info'}];
  }
  const frame=document.getElementById('phoneApp');
  addEventListener('message',e=>{const m=e.data;if(!m||m.kind!=='bridge-request'||e.source!==frame.contentWindow)return;
    let r;try{r=handle(m.method,m.path,m.body)}catch(err){r=[500,{error:String(err&&err.message||err)}]}
    e.source.postMessage({kind:'bridge-reply',id:m.id,status:r[0],body:r[1]},'*')});
  const bytes=Uint8Array.from(atob('@@APP@@'),ch=>ch.charCodeAt(0));
  frame.srcdoc=new TextDecoder().decode(bytes);
})();
</script>
"""
sim = sub1(sim, "@@BRIDGE@@", bridge.replace("@@APP@@", app_b64))
for t in ["<html", "<head", "<body", "</body", "</html", "SUMMON_BRIDGE"]:
    assert t not in sim.split("<script>\n// In-page bridge")[0], t
out.write_text(sim, encoding="utf8")
print("wrote", out, len(sim), "bytes")
