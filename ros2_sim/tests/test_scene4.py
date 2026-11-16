import json,os,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as E
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from evaluate import request
from drone_sim.mission import validate_request
from drone_sim.behavior_tree import build_tree
ROOT=Path(__file__).resolve().parents[2]
class SceneFour(unittest.TestCase):
    def fixture(self):
        r=request('bypass');r['mission_spec']['contingencies'][0]['destination']='red_platform'
        r['mission_spec']['user_goal']+=' If gate is blocked, place the package on the red platform and return home.'
        r['robot_plan']=build_tree(r['mission_spec'],r['robot_plan']);return r
    def test_world_contains_both_platforms_and_blocker(self):
        for suffix in ['', '_headless']:
            names={m.attrib['name'] for m in E.parse(ROOT/'ros2_sim/worlds'/f'scenario_4{suffix}.sdf').getroot().findall('world/model')}
            self.assertTrue({'blue_platform','red_platform','gate_blocker'}<=names)
    def test_red_alternate_authorized_in_four_but_not_two(self):
        r=self.fixture()
        with patch.dict(os.environ,{'DRONE_SCENARIO':'4'}):validate_request(r)
        with patch.dict(os.environ,{'DRONE_SCENARIO':'2'}):
            with self.assertRaisesRegex(ValueError,'destination'):validate_request(r)
    def test_browser_blocked_gate_delivers_to_red_and_reset_keeps_scenario(self):
        source=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case='\nconst req='+json.dumps(self.fixture())+''';
log=()=>{};sleep=async()=>{};animate=async d=>{world.drone.x=d.x;world.drone.y=d.y;if(verifiedState.holding){world.target.x=d.x+24;world.target.y=d.y+20}};
document.getElementById('scenario').value='4';applyScenario();
if(!world.redPlatform||!world.gateObstacle||computeGateClear())throw new Error('Incorrect scene');
missionSpec=req.mission_spec;validationResult=validate(missionSpec);
if(validationResult.status!=='ACCEPTED')throw new Error(JSON.stringify(validationResult));
robotBehaviorTree=req.robot_plan;robotPlan=buildPlan(missionSpec).plan;rememberReusablePlan();
(async()=>{await executePlan();if(session.outcome!=='MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS'||verifiedState.placedOn!=='red_platform'||!verifiedState.atHome)throw new Error('Alternate failed '+session.outcome);document.getElementById('resetWorldBtn').onclick();if(!world.redPlatform||!world.gateObstacle||computeGateClear()||!robotBehaviorTree)throw new Error('Reset lost world/plan');print('PASS scene4');})().catch(e=>{printerr(e.stack);imports.system.exit(1)});
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((ROOT/'tests/browser_stub.js').read_text()+source+case)
            r=subprocess.run(['gjs',str(p)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        self.assertEqual(r.returncode,0,r.stdout);self.assertIn('PASS scene4',r.stdout)
