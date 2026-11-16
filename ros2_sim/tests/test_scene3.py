import os,sys,json,subprocess,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as E
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from evaluate import request
from drone_sim.mission import validate_request
ROOT=Path(__file__).resolve().parents[2]
class SceneThree(unittest.TestCase):
    def test_world_has_distinct_platform_colors_and_clear_gate(self):
        for suffix in ['', '_headless']:
            world=E.parse(ROOT/'ros2_sim/worlds'/('scenario_3'+suffix+'.sdf')).getroot().find('world')
            blue=world.find('model[@name="blue_platform"]');red=world.find('model[@name="red_platform"]')
            self.assertIsNotNone(red);self.assertIsNone(world.find('model[@name="gate_blocker"]'))
            b=list(map(float,blue.find('.//material/diffuse').text.split()));r=list(map(float,red.find('.//material/diffuse').text.split()))
            self.assertGreater(b[2],b[1]);self.assertGreater(r[0],r[2]);self.assertNotEqual(blue.find('pose').text,red.find('pose').text)
    def test_external_red_destination_requires_scene_three(self):
        r=request()
        for g in r['mission_spec']['goals']:
            if g['predicate']=='placed':g['args'][1]='red_platform'
        for a in r['robot_plan']:
            a['args']=['red_platform' if x=='blue_platform' else x for x in a['args']]
        with patch.dict(os.environ,{'DRONE_SCENARIO':'3'}):validate_request(r)
        with patch.dict(os.environ,{'DRONE_SCENARIO':'1'}):
            with self.assertRaisesRegex(ValueError,'Scenario 3'):validate_request(r)
    def test_browser_red_platform_mission_does_not_deliver_to_blue(self):
        h=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case='''
log=()=>{};sleep=async()=>{};animate=async d=>{world.drone.x=d.x;world.drone.y=d.y;if(verifiedState.holding){world.target.x=d.x+24;world.target.y=d.y+20}};
document.getElementById('scenario').value='3';applyScenario();
const text='Find the red package, pick it up, place it on the red platform, and return home.';
missionSpec=normalizeMissionSpec(localCompile(text),text);missionSpec.contingencies=compileContingencies(text);robotPlan=buildPlan(missionSpec).plan;
if(!robotPlan.some(a=>a.skill==='PLACE'&&a.args[1]==='red_platform'))throw new Error('Wrong delivery goal');
executePlan().then(()=>{if(session.outcome!=='MISSION_COMPLETED'||!verifiedState.placed||world.target.x!==world.redPlatform.x||world.target.y!==world.redPlatform.y)throw new Error('Red platform delivery failed');print('PASS red delivery');});
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((ROOT/'tests/browser_stub.js').read_text()+h+case)
            output=subprocess.check_output(['gjs',str(p)],text=True)
        self.assertIn('PASS red delivery',output)
if __name__=='__main__':unittest.main()
