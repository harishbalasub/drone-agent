import copy,json,subprocess,sys,tempfile,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.mission import Executor
from test_gateway import IdleTransport
from evaluate import request
ROOT=Path(__file__).resolve().parents[2]
class ResetTransport(IdleTransport):
    def reset_world(self):
        self.state.update(position=[.85,.65,.43],pose_received=time.monotonic(),gripper='detached',gripper_received=time.monotonic())
        return {'source':'test_only','observed':{'drone':[.85,.65,.43],'red_package':[7.1,1.3,.12]}}
class ResetContracts(unittest.TestCase):
    def test_verified_reset_archives_episode_without_claiming_package_found(self):
        e=Executor(ResetTransport());e.session={'events':[],'outcome':'MISSION_COMPLETED'};e.facts={'found':True,'placed':True};old=copy.deepcopy(e.session)
        try:
            result=e.reset_world();self.assertEqual(result['status'],'IDLE');self.assertEqual(e.archived_sessions,[old]);self.assertIsNone(e.session);self.assertFalse(e.facts['found']);self.assertFalse(e.facts['placed']);self.assertIsNone(e.run_id)
        finally:e.close()
    def test_active_or_failed_reset_cannot_restart_execution(self):
        e=Executor(ResetTransport())
        try:
            e.active=True
            with self.assertRaisesRegex(ValueError,'Stop'):e.reset_world()
            e.active=False
            e.transport.reset_world=lambda:(_ for _ in ()).throw(RuntimeError('reset not verified'))
            with self.assertRaisesRegex(RuntimeError,'not verified'):e.reset_world()
            self.assertEqual(e.status,'RESET_FAILED')
            with self.assertRaisesRegex(ValueError,'verified world reset'):e.start(request())
        finally:e.close()
    def test_external_reset_uses_gateway_and_keeps_original_tree(self):
        source=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case='''
function setInterval(){};let resetCalls=0;
async function fetch(url,options){if(url==='/api/sim/reset'){resetCalls++;return {ok:true,json:async()=>({status:'IDLE',reset_evidence:{source:'test'}})}};return {ok:true,json:async()=>({reset_supported:true,status:'IDLE',active:false,connected:true,scenario:1,transport:'test',facts:{found:false},pose_age_seconds:0,perception_mode:'test',sensors:{position:[.85,.65,.43],yaw:0,objects:{},cameras:{}}})};}
missionSpec=localCompile(document.getElementById('mission').value);validationResult=validate(missionSpec);robotPlan=buildPlan(missionSpec).plan;robotBehaviorTree=buildBehaviorTree(missionSpec,robotPlan);rememberReusablePlan();const original=JSON.stringify(robotBehaviorTree);
document.getElementById('executionBackend').value='external';
'''
        assertions='''
(async()=>{await document.getElementById('resetWorldBtn').onclick();if(resetCalls!==1||JSON.stringify(robotBehaviorTree)!==original||document.getElementById('runBtn').disabled||document.getElementById('resetWorldBtn').disabled)throw new Error('External reset lost reusable plan');if(!session.versions.some(v=>v.stage==='EXTERNAL_WORLD_RESET'))throw new Error('Missing audit');print('PASS external reset');})().catch(e=>{printerr(e.stack);imports.system.exit(1)});
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'reset.js';p.write_text((ROOT/'tests/browser_stub.js').read_text()+source+case+(ROOT/'ros2_sim/app.js').read_text()+assertions)
            r=subprocess.run(['gjs',str(p)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        self.assertEqual(r.returncode,0,r.stdout);self.assertIn('PASS external reset',r.stdout)

    def test_external_scenario_change_loads_real_world_and_invalidates_plan(self):
        source=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case="""
function setInterval(){};let loaded=1,scenarioCalls=0;
async function fetch(url,options){if(url==='/api/sim/scenario'){loaded=JSON.parse(options.body).scenario;scenarioCalls++;return {ok:true,json:async()=>({status:'IDLE',reset_evidence:{scenario:loaded}})}};return {ok:true,json:async()=>({reset_supported:true,status:'IDLE',active:false,connected:true,scenario:loaded,transport:'test',facts:{found:false},pose_age_seconds:0,perception_mode:'test',sensors:{position:[.85,.65,.43],yaw:0,objects:{},cameras:{}}})};}
missionSpec=localCompile(document.getElementById('mission').value);validationResult=validate(missionSpec);robotPlan=buildPlan(missionSpec).plan;robotBehaviorTree=buildBehaviorTree(missionSpec,robotPlan);rememberReusablePlan();document.getElementById('executionBackend').value='external';
"""
        assertions="""
(async()=>{for(const scene of [3,4,2]){document.getElementById('scenario').value=String(scene);await document.getElementById('scenario').onchange();if(loaded!==scene||missionSpec||robotPlan.length||reusablePlan||Boolean(world.redPlatform)!==[3,4].includes(scene)||Boolean(world.gateObstacle)!==[2,4].includes(scene)||document.getElementById('scenario').disabled)throw new Error('Scenario did not load/invalidate plan');}if(scenarioCalls!==3)throw new Error('Missing switch');print('PASS external scenario');})().catch(e=>{printerr(e.stack);imports.system.exit(1)});
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'scenario.js';p.write_text((ROOT/'tests/browser_stub.js').read_text()+source+case+(ROOT/'ros2_sim/app.js').read_text()+assertions)
            r=subprocess.run(['gjs',str(p)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        self.assertEqual(r.returncode,0,r.stdout);self.assertIn('PASS external scenario',r.stdout)
