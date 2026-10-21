// Runs inside the simulator's scope. Exposes the simulator API and a mobile-first web controller.
$('dm').value='manual'; $('il').checked=true;
const TOKEN=process.env.SUMMON_TOKEN||'dev-token', PORT=+process.env.PORT||8787, PX=0.02;
let lastT=Date.now();
// When the visual browser simulator is open, it is the authoritative simulation state.
let browserState=null,browserStateAt=0,browserMap=null,browserMapAt=0;
let pendingCommands=[],nextCommandId=1,browserLidar=null,browserLidarAt=0;
// Server-side duplicate suppression: the UI guard alone is not sufficient when stale tabs or repeated events exist.
let lastNavSignature='',lastNavAt=0,lastStopAt=0;
// Command epoch: bumped when the simulator was stale and has just come back.
// Commands may carry the client's last-seen epoch; a mismatch is rejected.
let epoch=1,lastBrowserStateAt=0;
setInterval(()=>{const n=Date.now();const dt=Math.min(.05,(n-lastT)/1000);lastT=n;drainHeadless();update(dt)},16);
function drainHeadless(){
  // Headless mode is allowed only before a browser simulator has connected.
  // Once a browser has connected, never replay its queued commands into the
  // internal simulation after the browser becomes stale.
  if(browserState)return;
  while(pendingCommands.length){
    const c=pendingCommands.shift();
    try{
      // Same path as the browser (user target, then Come to me): cord, low-battery redirect and E-stop checks apply.
      if(c.type==='summon'){manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;user.x=c.x/PX;user.y=c.y/PX;$('go').onclick();}
      else if(c.type==='return'){manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;startReturn();}
      else if(c.type==='stop'){manualDrive.enabled=false;manualDrive.x=manualDrive.y=manualDrive.turn=0;active=false;path=[];recovery=null;stuckTime=0;parkTh=null;setSt('Stopped','warn');}
      else if(c.type==='learn'){startTour();}
      else if(c.type==='cord'){cordIn=!!c.plugged;setSt(cordIn?'Cord plugged in':'Cord unplugged','');}
      else if(c.type==='zones'){setZonesM(c.zones);}
      else if(c.type==='drive'){active=false;path=[];parkTh=null;mode='manual';manualDrive.enabled=true;manualDrive.lastCommandAt=Date.now();manualDrive.x=c.x;manualDrive.y=c.y;manualDrive.turn=c.turn;manualDrive.speed=c.speed;}
    }catch(e){console.error('drainHeadless failed for',c.type,e.message)}
  }
}
function stateJSON(){
  let ns=0,no=0;for(let i=0;i<GW*GH;i++){if(seen[i])ns++;if(smap[i]>0)no++}
  return {time:+time.toFixed(1),mode,status:$('st').textContent,severity:($('st').className||'info')||'info',active,estop,
    pose:{x:+(T.x*PX).toFixed(3),y:+(T.y*PX).toFixed(3),theta:+T.th.toFixed(3)},user:{x:+(user.x*PX).toFixed(3),y:+(user.y*PX).toFixed(3)},
    battery:{robot_pct:+bat.r.toFixed(1),ups_pct:+bat.a.toFixed(1)},charging,cord:{plugged:cordIn,interlock_on:!!$('il').checked,breakaways},
    dock_type:$('dm').value,desk_height_cm:Math.round(deskH*100),alert:alertT>0,explored_pct:Math.round(100*ns/(GW*GH)),remembered_cells:no,
    people:people.map(p=>({x:+(p.x*PX).toFixed(2),y:+(p.y*PX).toFixed(2),settled:!!p.settled})),...extraStateJSON()};
}
function mapJSON(){const rows=[];for(let y=0;y<GH;y++){let r='';for(let x=0;x<GW;x++){const i=y*GW+x;r+=smap[i]>0?'#':traffic[i]>2?'+':seen[i]?'_':'.'}rows.push(r)}return {width_m:W*PX,height_m:H*PX,cell_m:CS*PX,gw:GW,gh:GH,rows}}
function send(res,code,obj){const b=JSON.stringify(obj);res.writeHead(code,{'Content-Type':'application/json; charset=utf-8','Content-Length':Buffer.byteLength(b),'Cache-Control':'no-store','Access-Control-Allow-Origin':'*'});res.end(b)}
// The bridge listens on 0.0.0.0: only a page loaded from this machine gets the token injected.
function isLoopback(a){return a==='127.0.0.1'||a==='::1'||a==='::ffff:127.0.0.1'}
function sendFile(res,file,type,req){try{let b=require('fs').readFileSync(require('path').join(__dirname,'..',file));if(file==='robot/simulator/summon_table_sim.html'&&req&&isLoopback(req.socket.remoteAddress)){let html=b.toString('utf8');html=html.replace('</head>','<script>window.SUMMON_BRIDGE_TOKEN='+JSON.stringify(TOKEN)+'</script></head>');b=Buffer.from(html)}res.writeHead(200,{'Content-Type':type,'Content-Length':b.length,'Cache-Control':'no-store'});res.end(b)}catch(e){send(res,404,{error:'file not found'})}}
function readBody(req,cb){let s='';req.on('data',c=>{s+=c;if(s.length>262144)req.destroy()});req.on('end',()=>{try{cb(s?JSON.parse(s):{})}catch(e){cb(null)}})}
const server=require('http').createServer((req,res)=>{
  const url=req.url.split('?')[0];
  if(req.method==='OPTIONS'){res.writeHead(204,{'Access-Control-Allow-Origin':'*','Access-Control-Allow-Headers':'Authorization, Content-Type','Access-Control-Allow-Methods':'GET, POST, OPTIONS'});return res.end()}
  // The web app and simulator are served by this same local server, so the phone only needs one URL.
  if(req.method==='GET'&&(url==='/'||url==='/app'||url==='/app/'))return sendFile(res,'mobile/index.html','text/html; charset=utf-8');
  if(req.method==='GET'&&url==='/manifest.webmanifest')return sendFile(res,'mobile/manifest.webmanifest','application/manifest+json; charset=utf-8');
  if(req.method==='GET'&&url==='/simulator')return sendFile(res,'robot/simulator/summon_table_sim.html','text/html; charset=utf-8',req);
  if(req.method==='GET'&&url==='/v1/health')return send(res,200,{ok:true,name:'summon-desk-bridge (simulator)',version:1,epoch:epoch});
  if((req.headers.authorization||'')!=='Bearer '+TOKEN)return send(res,401,{error:'unauthorized'});
  if(req.method==='GET'&&url==='/v1/state'){const live=!!browserState&&(Date.now()-browserStateAt<3000);return send(res,200,{...(live?browserState:stateJSON()),simulator_connected:live,epoch:epoch})}
  if(req.method==='GET'&&url==='/v1/map')return send(res,200,{...(browserMap&&(Date.now()-browserMapAt<5000)?browserMap:mapJSON()),epoch:epoch});
  if(req.method==='GET'&&url==='/v1/lidar'){const live=!!browserLidar&&(Date.now()-browserLidarAt<3000);const points=live?browserLidar:(hits||[]).filter(h=>h&&h.hit&&Number.isFinite(h.d)&&h.d>0).map(h=>({angle:+(((h.a-T.th)*180/Math.PI+540)%360-180).toFixed(2),range:+(h.d*PX).toFixed(3),type:h.type||'obstacle'}));return send(res,200,{source:live?'simulator-live':'bridge-sim-core',simulator_connected:live,max_range:5.2,frame:'robot',units:'meters',points,updated_at:Date.now(),epoch:epoch})}
  if(req.method==='GET'&&url==='/v1/sim/poll'){
    // Defense in depth: never deliver commands from an older session.
    const commands=pendingCommands.splice(0).filter(c=>c.epoch===epoch);
    return send(res,200,{commands,server_time:Date.now(),epoch:epoch});
  }
  if(req.method==='POST'&&(url==='/v1/sim/state'||url==='/v1/sim/map'))return readBody(req,body=>{
    if(body===null)return send(res,400,{error:'bad json'});
    if(url==='/v1/sim/state'){
      const hadBrowserSession=lastBrowserStateAt>0;
      const wasStale=!lastBrowserStateAt||(Date.now()-lastBrowserStateAt>=3000);
      browserState=body;browserStateAt=Date.now();lastBrowserStateAt=browserStateAt;
      if(wasStale){
        epoch++;
        // Discard commands belonging to the previous browser session.
        pendingCommands=[];
        // On reconnection, require a known stopped state before resuming.
        // Do not inject a Stop on the simulator's initial connection.
        if(hadBrowserSession){
          pendingCommands.push({id:nextCommandId++,type:'stop',created_at:Date.now(),epoch:epoch});
        }
        console.log('simulator '+(hadBrowserSession?'reconnected':'connected')+', epoch='+epoch);
      }
      if(Array.isArray(body.lidar)){browserLidar=body.lidar;browserLidarAt=Date.now()}
    }else{browserMap=body;browserMapAt=Date.now()}
    return send(res,200,{ok:true,epoch:epoch})
  });
  if(req.method!=='POST')return send(res,404,{error:'not found'});
  readBody(req,body=>{
    if(body===null)return send(res,400,{error:'bad json'});
    // Reject commands explicitly tied to an earlier simulator session.
    if(body.epoch!=null&&body.epoch!==epoch){
      return send(res,409,{accepted:false,status:'Stale command from an earlier session. Refresh and try again.',severity:'warn',stale_epoch:true,current_epoch:epoch});
    }
    let command=null;
    if(url==='/v1/summon'){
      const x=+body.x,y=+body.y;
      if(!(Number.isFinite(x)&&Number.isFinite(y)&&x>=0.5&&x<=W*PX-0.5&&y>=0.5&&y<=H*PX-0.5))return send(res,422,{accepted:false,status:'Target outside the room'});
      command={type:'summon',x,y};
    }else if(url==='/v1/drive'){
      const x=Number(body.x),y=Number(body.y),turn=Number(body.turn),speed=Number(body.speed);
      if(![x,y,turn,speed].every(Number.isFinite))return send(res,422,{accepted:false,status:'Drive values must be numbers'});
      command={type:'drive',x:Math.max(-1,Math.min(1,x)),y:Math.max(-1,Math.min(1,y)),turn:Math.max(-1,Math.min(1,turn)),speed:Math.max(0.08,Math.min(1,speed))};
    }else if(url==='/v1/return')command={type:'return'};
    else if(url==='/v1/learn')command={type:'learn'};
    else if(url==='/v1/stop')command={type:'stop'};
    else if(url==='/v1/sim/cord')command={type:'cord',plugged:!!body.plugged};
    else if(url==='/v1/zones'){
      // Keep-out zones in metres: [{name,x,y,w,h}], replaces the whole list (empty list clears them).
      const zs=body.zones,roomW=W*PX,roomH=H*PX;
      if(!Array.isArray(zs)||zs.length>20)return send(res,422,{accepted:false,status:'zones must be a list of at most 20'});
      for(const z of zs){const v=[z&&z.x,z&&z.y,z&&z.w,z&&z.h].map(Number);
        if(!v.every(Number.isFinite)||v[2]<0.1||v[3]<0.1||v[0]<0||v[1]<0||v[0]+v[2]>roomW+0.01||v[1]+v[3]>roomH+0.01)return send(res,422,{accepted:false,status:'Each zone needs x, y, w, h in metres inside the room (w, h at least 0.1 m)'})}
      command={type:'zones',zones:zs.map(z=>({id:String(z.id||'').slice(0,40),name:String(z.name||'Keep out').slice(0,24),x:+z.x,y:+z.y,w:+z.w,h:+z.h}))};
    }
    else return send(res,404,{error:'not found'});
    const browserLive=!!browserState&&(Date.now()-browserStateAt<3000);
    const browserEverSeen=!!browserState;
    if(browserEverSeen&&!browserLive)return send(res,503,{accepted:false,status:'Simulator page is not connected. Open /simulator and keep it running.'});
    // Enforce the cord interlock before acknowledging or queueing a movement request.
    // Stop and cord plug/unplug commands remain available so the user can stop or clear the interlock.
    const movementTypes=new Set(['summon','drive','return','learn']);
    const cordPlugged=browserState?!!browserState.cord?.plugged:!!cordIn;
    const cordInterlock=browserState?!!browserState.cord?.interlock_on:!!$('il').checked;
    if(movementTypes.has(command.type)&&cordPlugged&&cordInterlock){
      return send(res,409,{accepted:false,status:'Movement refused: desk cord is plugged in. Unplug the cord before moving.',severity:'bad'});
    }
    const now=Date.now();
    if(['summon','return','learn'].includes(command.type)){
      const signature=command.type==='summon'?JSON.stringify({type:command.type,x:command.x,y:command.y}):command.type;
      // Reject rapid repeats from repeated taps, stale tabs, or duplicate browser handlers.
      if(signature===lastNavSignature&&now-lastNavAt<2000){
        return send(res,409,{accepted:false,status:'Duplicate navigation command ignored. Wait for the current command to finish.',severity:'warn'});
      }
      lastNavSignature=signature;lastNavAt=now;
    }
    if(command.type==='stop'){
      // Stop ALWAYS supersedes queued motion commands (even a coalesced repeat); do not let an old joystick packet run after it.
      pendingCommands=pendingCommands.filter(c=>!['drive','summon','return','learn'].includes(c.type));
      lastNavSignature='';lastNavAt=0;
      // Coalesce repeated Stop clicks for 250 ms, but only while the earlier Stop is still queued.
      if(now-lastStopAt<250&&pendingCommands.some(c=>c.type==='stop'))return send(res,200,{accepted:true,status:'Stop already queued',severity:'info'});
      lastStopAt=now;
    }else if(['summon','return'].includes(command.type)){
      // Navigation commands replace stale manual-drive packets and older navigation requests.
      pendingCommands=pendingCommands.filter(c=>!['drive','summon','return','learn'].includes(c.type));
    }else if(command.type==='drive'){
      // Keep only the newest joystick state instead of accumulating stale input.
      pendingCommands=pendingCommands.filter(c=>c.type!=='drive');
    }
    const queued={id:nextCommandId++,...command,created_at:Date.now(),epoch:epoch};
    pendingCommands.push(queued);
    send(res,200,{accepted:true,status:'Command sent to simulator',severity:'info',command_id:queued.id});
  });
});
server.listen(PORT,'0.0.0.0',()=>console.log('Summon Desk local web app + simulator bridge on port '+PORT+' (token configured, epoch='+epoch+')'));
