import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
class AppContract(unittest.TestCase):
    def test_external_run_uses_gateway_not_animation(self):
        html=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text();source=html.split('<script>',1)[1].split('</script>',1)[0]
        stub=(ROOT/'tests/browser_stub.js').read_text()
        pre='''\nfunction setInterval(){};let calls=[];
async function fetch(url,opts){calls.push([url,opts]);return {ok:true,json:async()=>url.endsWith('/run')?{run_id:'run1'}:{transport:'test',connected:true,status:'MISSION_COMPLETED',active:false,run_id:'run1',pose_age_seconds:0,perception_mode:'test',runtime_contract:{action:{skill:'RETURN_HOME',args:[]},status:'SUCCEEDED',checks:[{name:'Battery reserve',status:'unknown',detail:'Not modeled'}],completion_behavior:'Hover at 1.6 m'},battery:{available:false},facts:{},sensors:{position:[.85,.65,1.6],yaw:0,objects:{},cameras:{}},session:{events:[]}}};}
'''
        case='''\nmissionSpec=localCompile(document.getElementById('mission').value);validationResult=validate(missionSpec);robotPlan=buildPlan(missionSpec).plan;pddlProblem=compilePddl(missionSpec);
document.getElementById('executionBackend').value='external';
let animated=false;skill=async()=>{animated=true;};
document.getElementById('runBtn').onclick().then(()=>{if(animated||!calls.some(c=>c[0]==='/api/sim/run'))throw new Error('External execution did not use gateway');if(!document.getElementById('contractPanel').innerHTML.includes('RETURN_HOME'))throw new Error('Missing live skill contract');if(document.getElementById('batteryValue').textContent!=='Unavailable')throw new Error('Fabricated external battery');print('PASS external adapter and telemetry panels');});
'''
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'app.js';p.write_text(stub+pre+source+(ROOT/'ros2_sim/app.js').read_text()+case)
            subprocess.run(['gjs',str(p)],check=True)
