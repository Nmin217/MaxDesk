// Runs inside the simulator's scope. Exposes the simulator API and a mobile-first web controller.
$('dm').value='manual'; $('il').checked=true;
const TOKEN=process.env.SUMMON_TOKEN||'dev-token', PORT=+process.env.PORT||8787, PX=0.02;
let lastT=Date.now();
// When the visual browser simulator is open, it is the authoritative simulation state.
let browserState=null,browserStateAt=0,browserMap=null,browserMapAt=0;
let pendingCommands=[],nextCommandId=1;
setInterval(()=>{const n=Date.now();const dt=Math.min(.05,(n-lastT)/1000);lastT=n;update(dt)},16);
function stateJSON(){
  let ns=0,no=0;for(let i=0;i<GW*GH;i++){if(seen[i])ns++;if(smap[i]>0)no++}
  return {time:+time.toFixed(1),mode,status:$('st').textContent,severity:($('st').className||'info')||'info',active,estop,
    pose:{x:+(T.x*PX).toFixed(3),y:+(T.y*PX).toFixed(3),theta:+T.th.toFixed(3)},user:{x:+(user.x*PX).toFixed(3),y:+(user.y*PX).toFixed(3)},
    battery:{robot_pct:+bat.r.toFixed(1),ups_pct:+bat.a.toFixed(1)},charging,cord:{plugged:cordIn,interlock_on:!!$('il').checked,breakaways},
    dock_type:$('dm').value,desk_height_cm:Math.round(deskH*100),alert:alertT>0,explored_pct:Math.round(100*ns/(GW*GH)),remembered_cells:no,
    people:people.map(p=>({x:+(p.x*PX).toFixed(2),y:+(p.y*PX).toFixed(2),settled:!!p.settled}))};
}
function mapJSON(){const rows=[];for(let y=0;y<GH;y++){let r='';for(let x=0;x<GW;x++){const i=y*GW+x;r+=smap[i]>0?'#':traffic[i]>2?'+':seen[i]?'_':'.'}rows.push(r)}return {width_m:W*PX,height_m:H*PX,cell_m:CS*PX,gw:GW,gh:GH,rows}}
function send(res,code,obj){const b=JSON.stringify(obj);res.writeHead(code,{'Content-Type':'application/json; charset=utf-8','Content-Length':Buffer.byteLength(b),'Cache-Control':'no-store','Access-Control-Allow-Origin':'*'});res.end(b)}
function sendFile(res,file,type){try{let b=require('fs').readFileSync(require('path').join(__dirname,'..',file));if(file==='robot/simulator/summon_table_sim.html'){let html=b.toString('utf8');html=html.replace('</head>','<script>window.SUMMON_BRIDGE_TOKEN='+JSON.stringify(TOKEN)+'</script></head>');b=Buffer.from(html)}res.writeHead(200,{'Content-Type':type,'Content-Length':b.length,'Cache-Control':'no-store'});res.end(b)}catch(e){send(res,404,{error:'file not found'})}}
function readBody(req,cb){let s='';req.on('data',c=>{s+=c;if(s.length>262144)req.destroy()});req.on('end',()=>{try{cb(s?JSON.parse(s):{})}catch(e){cb(null)}})}
const server=require('http').createServer((req,res)=>{
  const url=req.url.split('?')[0];
  if(req.method==='OPTIONS'){res.writeHead(204,{'Access-Control-Allow-Origin':'*','Access-Control-Allow-Headers':'Authorization, Content-Type','Access-Control-Allow-Methods':'GET, POST, OPTIONS'});return res.end()}
  // The web app and simulator are served by this same local server, so the phone only needs one URL.
  if(req.method==='GET'&&(url==='/'||url==='/app'||url==='/app/'))return sendFile(res,'mobile/index.html','text/html; charset=utf-8');
  if(req.method==='GET'&&url==='/manifest.webmanifest')return sendFile(res,'mobile/manifest.webmanifest','application/manifest+json; charset=utf-8');
  if(req.method==='GET'&&url==='/simulator')return sendFile(res,'robot/simulator/summon_table_sim.html','text/html; charset=utf-8');
  if(req.method==='GET'&&url==='/v1/health')return send(res,200,{ok:true,name:'summon-desk-bridge (simulator)',version:1});
  if((req.headers.authorization||'')!=='Bearer '+TOKEN)return send(res,401,{error:'unauthorized'});
  if(req.method==='GET'&&url==='/v1/state'){const live=!!browserState&&(Date.now()-browserStateAt<3000);return send(res,200,{...(live?browserState:stateJSON()),simulator_connected:live})}
  if(req.method==='GET'&&url==='/v1/map')return send(res,200,browserMap&&(Date.now()-browserMapAt<5000)?browserMap:mapJSON());
  if(req.method==='GET'&&url==='/v1/sim/poll'){const commands=pendingCommands.splice(0);return send(res,200,{commands,server_time:Date.now()})}
  if(req.method==='POST'&&(url==='/v1/sim/state'||url==='/v1/sim/map'))return readBody(req,body=>{if(body===null)return send(res,400,{error:'bad json'});if(url==='/v1/sim/state'){browserState=body;browserStateAt=Date.now()}else{browserMap=body;browserMapAt=Date.now()}return send(res,200,{ok:true})});
  if(req.method!=='POST')return send(res,404,{error:'not found'});
  readBody(req,body=>{
    if(body===null)return send(res,400,{error:'bad json'});
    let command=null;
    if(url==='/v1/summon'){
      const x=+body.x,y=+body.y;
      if(!(Number.isFinite(x)&&Number.isFinite(y)&&x>=0.5&&x<=W*PX-0.5&&y>=0.5&&y<=H*PX-0.5))return send(res,422,{accepted:false,status:'Target outside the room'});
      command={type:'summon',x,y};
    }else if(url==='/v1/return')command={type:'return'};
    else if(url==='/v1/learn')command={type:'learn'};
    else if(url==='/v1/stop')command={type:'stop'};
    else if(url==='/v1/sim/cord')command={type:'cord',plugged:!!body.plugged};
    else return send(res,404,{error:'not found'});
    if(!browserState||Date.now()-browserStateAt>=3000)return send(res,503,{accepted:false,status:'Simulator page is not connected. Open /simulator and keep it running.'});
    const queued={id:nextCommandId++,...command,created_at:Date.now()};
    pendingCommands.push(queued);
    send(res,200,{accepted:true,status:'Command sent to simulator',severity:'info',command_id:queued.id});
  });
});
server.listen(PORT,'0.0.0.0',()=>console.log('Summon Desk local web app + simulator bridge on port '+PORT+' (token configured)'));
