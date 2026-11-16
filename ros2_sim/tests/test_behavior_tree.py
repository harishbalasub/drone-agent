import copy,json,sys,tempfile,subprocess,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.behavior_tree import build_tree,execute_tree
from drone_sim.mission import validate_request
from evaluate import request

class BehaviorTreeContracts(unittest.TestCase):
    def test_clear_traverses_but_blocked_delivers_without_traversing(self):
        req=request('bypass',3);tree=build_tree(req['mission_spec'],req['robot_plan'])
        for clear in [True,False]:
            actions=[];self.assertTrue(execute_tree(tree,lambda a:actions.append(a['skill']),lambda p:clear if p=='gate_clear' else not clear))
            self.assertEqual('TRAVERSE_GATE' in actions,clear);self.assertEqual('BYPASS_GATE' in actions,not clear)
            self.assertEqual(actions.count('INSPECT'),3);self.assertIn('PLACE',actions);self.assertEqual(actions[-1],'RETURN_HOME')
    def test_unknown_and_skill_failures_never_select_alternate(self):
        req=request('bypass');tree=build_tree(req['mission_spec'],req['robot_plan']);actions=[]
        def unknown(p):raise RuntimeError('GATE_CLEAR_UNKNOWN')
        with self.assertRaisesRegex(RuntimeError,'UNKNOWN'):execute_tree(tree,lambda a:actions.append(a['skill']),unknown)
        self.assertNotIn('BYPASS_GATE',actions)
        def failed(a):
            actions.append(a['skill'])
            if a['skill']=='TRAVERSE_GATE':raise RuntimeError('CONTACT_COLLISION')
        with self.assertRaisesRegex(RuntimeError,'COLLISION'):execute_tree(tree,failed,lambda p:True)
        self.assertNotIn('BYPASS_GATE',actions)
    def test_gateway_rejects_unauthorized_or_edited_branch(self):
        req=request('bypass');tree=build_tree(req['mission_spec'],req['robot_plan']);req['robot_plan']=tree;validate_request(req)
        req['mission_spec']['contingencies'][0]['authorization']='SYSTEM_POLICY'
        with self.assertRaisesRegex(ValueError,'alternates'):validate_request(req)
        req=request('bypass');tree=build_tree(req['mission_spec'],req['robot_plan']);req['robot_plan']=tree
        next(n for n in tree['root']['children'] if n['type']=='Fallback')['children'][1]['children'][2]['args']=['home']
        with self.assertRaisesRegex(ValueError,'alternates'):validate_request(req)
    def test_exact_user_phrase_builds_same_tree_in_browser_and_gateway(self):
        root=Path(__file__).resolve().parents[2];html=(root/'accountable_drone_agent_ros2_v3.html').read_text();source=html.split('<script>',1)[1].split('</script>',1)[0]
        req=request('bypass',3)
        case='\nconst input='+json.dumps(req)+';input.mission_spec.contingencies=compileContingencies("if gate is not clear, goto blue platform directly");print(JSON.stringify(buildBehaviorTree(input.mission_spec,input.robot_plan)));'
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((root/'tests/browser_stub.js').read_text()+source+case)
            output=subprocess.check_output(['gjs',str(p)],text=True)
        tree=json.loads(output.strip().splitlines()[-1]);self.assertEqual(tree,build_tree(req['mission_spec'],req['robot_plan']))
        req['robot_plan']=tree;validate_request(req)
    def test_browser_blocked_gate_delivers_before_return_without_replan(self):
        root=Path(__file__).resolve().parents[2];html=(root/'accountable_drone_agent_ros2_v3.html').read_text();source=html.split('<script>',1)[1].split('</script>',1)[0]
        case="""
let messages=[],actions=[];log=m=>messages.push(m);sleep=async()=>{};
animate=async dest=>{world.drone.x=dest.x;world.drone.y=dest.y;if(verifiedState.holding){world.target.x=dest.x+24;world.target.y=dest.y+20}};
const originalSkill=skill;skill=async a=>{actions.push(a.skill);await originalSkill(a)};
document.getElementById('scenario').value='2';applyScenario();
const text='Find the red package, inspect it from three viewpoints, pick it up, fly through the gate, place it on the blue platform, and return home. If gate is not clear, go to blue platform.';
missionSpec=localCompile(text);missionSpec.contingencies=compileContingencies(text);robotPlan=buildPlan(missionSpec).plan;planWorldVersion=worldVersion;
executePlan().then(()=>{
 if(session.outcome!=='MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS')throw new Error(session.outcome);
 if(!verifiedState.placed||verifiedState.holding||!verifiedState.atHome)throw new Error('Delivered package not verified');
 if(actions.includes('TRAVERSE_GATE')||actions.indexOf('PLACE')>actions.indexOf('RETURN_HOME'))throw new Error('Wrong branch/order');
 if(session.interrupts.length||session.events.filter(e=>e.message_type==='BT_ALTERNATE_SELECTED').length!==1)throw new Error('Fallback should not require recovery');
 if(Object.keys(session).some(k=>!['session_id','started_at','saved_at','events','versions','interrupts','human_decisions','outcome','scenario','world','state'].includes(k)))throw new Error('Session export schema mismatch');
 print('PASS browser gate alternate delivers, releases, then returns home');
});
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((root/'tests/browser_stub.js').read_text()+source+case)
            output=subprocess.check_output(['gjs',str(p)],text=True)
        self.assertIn('PASS browser gate alternate',output)
    def test_browser_rejected_recovery_preserves_original_mission(self):
        root=Path(__file__).resolve().parents[2];html=(root/'accountable_drone_agent_ros2_v3.html').read_text();source=html.split('<script>',1)[1].split('</script>',1)[0]
        case="""
missionSpec=localCompile(document.getElementById('mission').value);robotPlan=buildPlan(missionSpec).plan;
const original=missionSpec,plan=robotPlan;checkRecoveryMotion=()=>{throw new Error('NO_SAFE_ROUTE')};
installRecovery(recoverySpec(original,'ABORT_AND_RETURN_HOME',{id:'test',world_version:worldVersion,position:world.drone,reason:'test'})).then(()=>{throw new Error('Expected rejection')},()=>{
 if(missionSpec!==original||robotPlan!==plan)throw new Error('Rejected candidate replaced original mission');print('PASS recovery rollback');
});
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((root/'tests/browser_stub.js').read_text()+source+case)
            output=subprocess.check_output(['gjs',str(p)],text=True)
        self.assertIn('PASS recovery rollback',output)
    def test_alternate_wording_is_recognized_or_requires_clarification(self):
        root=Path(__file__).resolve().parents[2];source=(root/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case="""
const phrases=[
 'If gate is not clear, goto blue platform directly',
 'If the gate is not clear, go to the blue platform',
 'If the gate is blocked, go directly to the blue platform',
 'If gate is blocked, skip it and goto blue platform',
 'If gate is not clear, skip the gate traversal',
 "If gate isn't clear, go straight to the blue platform"
];
for(const text of phrases){const policy=compileContingencies(text)[0];if(policy.action!=='BYPASS_GATE'||policy.recognition_status!=='RECOGNIZED')throw new Error(text);}
const spec=localCompile(document.getElementById('mission').value);
spec.contingencies=compileContingencies('If gate is not clear, improvise another solution');
if(validate(spec).status!=='NEEDS_CLARIFICATION')throw new Error('Unparsed conditional silently authorized an abort');
if(compileContingencies('If gate is blocked, ask me before bypassing')[0].action!=='ASK_HUMAN')throw new Error('Human approval skipped');
if(compileContingencies('If gate is blocked, do not skip it')[0].action==='BYPASS_GATE')throw new Error('Negated skip authorized');
print('PASS alternate wording and clarification');
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((root/'tests/browser_stub.js').read_text()+source+case)
            output=subprocess.check_output(['gjs',str(p)],text=True)
        self.assertIn('PASS alternate wording',output)
if __name__=='__main__':unittest.main()
