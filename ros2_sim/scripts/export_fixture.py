#!/usr/bin/env python3
"""Export the default spec/plan from the unmodified production HTML compiler."""
from pathlib import Path
import json
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]
html=(ROOT/'accountable_drone_agent_prototype_v2.html').read_text()
source=html.split('<script>',1)[1].split('</script>',1)[0]
stub=(ROOT/'tests/browser_stub.js').read_text()
goal='Find the red package, inspect it from two viewpoints, pick it up, fly through the gate, place it on the blue platform, and return home.'
script=stub+source+'\nconst example=localCompile('+json.dumps(goal)+'); print(JSON.stringify({mission_spec:example,robot_plan:buildPlan(example).plan,pddl:compilePddl(example)}));'
with tempfile.TemporaryDirectory() as folder:
    p=Path(folder)/'fixture.js';p.write_text(script)
    output=subprocess.check_output(['gjs',str(p)],text=True)
fixture=json.loads(output.splitlines()[-1]);fixture['mission_spec']['mission_id']='mission_ros2_example'
(ROOT/'ros2_sim/config/default_mission.json').write_text(json.dumps(fixture,indent=2)+'\n')
