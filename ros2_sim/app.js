/* External execution adapter. The legacy compiler/validator remains available. */
(() => {
  const el=id=>document.getElementById(id);
  let externalRun=null, polling=false, lastSequence=0, latest=null, shownRevision="";
  const browserRun=el('runBtn').onclick;
  const browserSave=el('saveSessionBtn').onclick;
  const browserObserve=el('observeBtn').onclick;
  const browserScenario=el('scenario').onchange;
  const browserReset=el('resetWorldBtn').onclick;
  const external=()=>el('executionBackend').value==='external';
  async function api(path,body){
    const response=await fetch('/api/sim/'+path,body===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const data=await response.json();if(!response.ok)throw new Error(data.error||'Simulator request failed');return data;
  }
  function viewMode(){
    el('externalControls').hidden=!external();el('simConnection').hidden=!external();
    const scene=external()&&el('externalView').value!=='map';
    canvas.hidden=scene;el('externalScene').hidden=!scene;el('simLegend').hidden=scene;
    el('simHint').textContent=external()?'Gazebo is authoritative. Restart the launcher to change scenario. Choose Telemetry map when rendering is disabled.':'EDIT: drag any object in the browser world.';
  }
  el('externalView').onchange=viewMode;
  function summary(s){
    viewMode();
    const p=s.sensors.position;
    el('externalSummary').textContent=`${s.transport} · ${s.status} · pose ${p?p.map(x=>x.toFixed(2)).join(', '):'unavailable'} m · perception: ${s.perception_mode}`;
    el('simConnection').textContent=s.connected?'CONNECTED':'NO FRESH ODOMETRY';
    el('simConnection').className='badge '+(s.connected?'good':'warn');
    el('externalStatus').textContent=JSON.stringify({status:s.status,transport:s.transport,pose_age_seconds:s.pose_age_seconds,facts:s.facts,sensors:s.sensors},null,2);
    el('externalHuman').hidden=s.status!=='WAITING_FOR_HUMAN';
    if(external())for(const id of ['scenario','resetWorldBtn','modeBtn','observeBtn','battery'])el(id).disabled=true;
    if(external()){
      setExecutionState(s.status);
      const contract=s.runtime_contract;
      if(contract){
        const a=contract.action;
        el('contractPanel').innerHTML=(a?`<b>${esc(a.skill)}(${esc((a.args||[]).join(', '))})</b><div>Status: <b>${esc(contract.status)}</b></div>`:'No skill executing.')+
          (contract.reason?`<div class="no">${esc(contract.reason)}</div>`:'')+
          (contract.checks||[]).map(c=>`<div class="${c.status==='pass'?'ok':c.status==='fail'?'no':''}">${c.status==='pass'?'✓':c.status==='fail'?'✕':'○'} ${esc(c.name)}: ${esc(c.detail)}</div>`).join('')+
          `<div class="small">${esc(contract.completion_behavior||'')}</div>`;
      }else el('contractPanel').textContent='Gateway update required for live skill contracts.';
      const b=s.battery;
      el('battery').hidden=!b?.available;
      el('batteryLabel').textContent='ROS2 battery';
      el('batteryValue').textContent=b?.available?Number(b.percent).toFixed(0):'Unavailable';
      el('batteryNote').textContent=b?.available?'% — '+b.source:' — no battery sensor/model; reserve not verified';
      if(b?.available)el('battery').value=b.percent;

      const revision=s.session?.versions?.at(-1), key=s.run_id+':'+revision?.version;
      if(revision&&shownRevision!==key){
        shownRevision=key;
        el('specOut').textContent=JSON.stringify(revision.mission_spec,null,2);
        el('planOut').textContent=JSON.stringify({mission_revision:revision.version,backend:s.transport,plan:revision.robot_plan},null,2);
        el('pddlOut').textContent=revision.pddl;
      }
    }
    if(external()){
      if(s.scenario===3)world.redPlatform={id:'red_platform',type:'platform',x:760,y:255,w:105,h:52};else delete world.redPlatform;
      el('redPlatformLegend').hidden=!world.redPlatform;
    }
    if(external()&&p){
      world.drone.x=p[0]*100;world.drone.y=520-p[1]*100;world.drone.heading=-s.sensors.yaw;
      const target=s.sensors.objects.red_package;
      if(target){world.target.x=100*target.position[0];world.target.y=520-100*target.position[1];}
      const blocker=s.sensors.objects.gate_blocker;if(blocker)world.gateObstacle={id:'gate_blocker',type:'obstacle',x:100*blocker.position[0],y:520-100*blocker.position[1],w:90,h:25};
      el('beliefPanel').textContent=JSON.stringify(s.facts,null,2);
      el('truthPanel').textContent=`External telemetry projection. Altitude: ${p[2].toFixed(2)} m. Browser geometry is a reference map; Gazebo is authoritative.`;
      for(const camera of ['front','down','external'])if(s.sensors.cameras[camera])el(camera+'Image').src='/api/sim/camera/'+camera+'?t='+Date.now();
    }
  }
  async function refresh(){
    if(polling)return;polling=true;
    try{
      latest=await api('state');summary(latest);
      if(external()&&latest.active&&!externalRun){externalRun=latest.run_id;lastSequence=0;running=true;setRunControls(true);el('executionBackend').disabled=true;setExecutionState('EXTERNAL_EXECUTION');}
      if(externalRun&&latest.run_id===externalRun){
        for(const e of latest.session?.events||[])if(e.sequence>lastSequence){
          lastSequence=e.sequence;log(`[${latest.transport}] ${e.message_type}: ${e.reason||e.outcome||(e.message_type==='BT_NODE'?e.node_path+' '+e.status:e.message_type==='BT_ALTERNATE_SELECTED'?e.trigger+' → '+e.action+'; waived: '+e.waived_goals.join(', '):e.action?.skill||e.result||'')}`,e.result==='FAILED'?'warn':'');
        }
        if(!latest.active){
          log('External outcome: '+latest.status,latest.status==='MISSION_COMPLETED'?'good':'warn');
          session.outcome=latest.status;setExecutionState(latest.status);externalRun=null;running=false;setRunControls(false);el('executionBackend').disabled=false;el('runBtn').disabled=true;for(const id of ['scenario','resetWorldBtn','modeBtn','observeBtn','battery'])el(id).disabled=true;
        }
      }
    }catch(e){el('simConnection').textContent='GATEWAY UNAVAILABLE';if(external()){el('contractPanel').textContent='Gateway unavailable — live runtime checks unavailable.';el('batteryValue').textContent='Unavailable';el('batteryNote').textContent=' — gateway disconnected';}
      el('externalSummary').textContent=e.message+'. Open this app through ros2_sim/server.py on port 8100.';}
    finally{polling=false;}
  }
  el('connectSim').onclick=refresh;
  el('executionBackend').onchange=()=>{
    const enabled=external();viewMode();el('runBtn').textContent=enabled?'Run plan in external simulator':'Run Plan in Simulator';
    el('scenario').disabled=enabled;el('resetWorldBtn').disabled=enabled;el('modeBtn').disabled=enabled;el('observeBtn').disabled=enabled;el('battery').disabled=enabled;
    if(enabled)refresh();else {el('battery').hidden=false;el('batteryLabel').textContent='Battery';el('batteryValue').textContent=el('battery').value;el('batteryNote').textContent='% — safe return reserve: 20%';applyScenario();}
  };
  el('runBtn').onclick=async()=>{
    if(!external())return browserRun();
    try{
      if(validationResult?.status!=='ACCEPTED'||!robotPlan.length)throw new Error('Generate and validate a mission, then build its robot plan first.');
      const result=await api('run',{mission_spec:missionSpec,robot_plan:executionTree(),planner_source:robotPlannerSource,pddl:pddlProblem,validation:validationResult});
      externalRun=result.run_id;lastSequence=0;running=true;setRunControls(true);el('executionBackend').disabled=true;setExecutionState('EXTERNAL_EXECUTION');log('External execution started: '+externalRun,'good');await refresh();
    }catch(e){log(e.message,'bad');}
  };
  el('externalStop').onclick=async()=>{try{await api('stop',{});await refresh();}catch(e){log(e.message,'bad');}};
  async function saveExternal(){try{const r=await api('save',{directory:el('saveDirectory').value});el('saveStatus').textContent='Saved '+r.path;log('External session saved: '+r.path,'good');}catch(e){log(e.message,'bad');}}
  el('externalSave').onclick=saveExternal;
  el('saveSessionBtn').onclick=()=>external()?saveExternal():browserSave();
  for(const [id,action] of [['externalBypass','BYPASS_GATE'],['externalAbort','ABORT_AND_RETURN_HOME']])el(id).onclick=async()=>{try{await api('decision',{action});await refresh();}catch(e){log(e.message,'bad');}};
  // Do not let the browser fabricate observations or edit the live external world.
  for(const name of ['onpointerdown','onpointermove','onpointerup']){
    const handler=canvas[name];canvas[name]=e=>{if(!external())handler(e);};
  }
  el('observeBtn').onclick=()=>external()?refresh():browserObserve();
  el('scenario').onchange=()=>{if(!external())browserScenario();};
  el('resetWorldBtn').onclick=()=>{if(!external())browserReset();};
  const referenceDraw=draw;draw=()=>{referenceDraw();if(external()){ctx.fillStyle='#f8fbff';ctx.fillRect(0,0,canvas.width,26);ctx.fillStyle='#315b85';ctx.font='12px sans-serif';ctx.fillText('External telemetry projection · altitude '+(latest?.sensors?.position?.[2]?.toFixed(2)||'?')+' m · Gazebo is authoritative',12,18);}};
  viewMode();
  setInterval(()=>{if(external()||externalRun)refresh();},1000);
})();
