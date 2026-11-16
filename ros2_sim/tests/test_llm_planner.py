"""Mock structured provider responses; no paid or live API requests."""
import copy,json,os,subprocess,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from test_behavior_tree import request
from drone_sim.behavior_tree import build_tree,execute_tree,validate_generated_tree
from drone_sim.mission import validate_request

class LLMPlanner(unittest.TestCase):
    def fixture(self):
        req=request('bypass',3)
        req['mission_spec']['contingencies'][0]['destination']='red_platform'
        tree=build_tree(req['mission_spec'],req['robot_plan'])
        # A valid topology different from the local template: blocked branch first,
        # plus a nested sequence. The executor must preserve these choices.
        branch=next(n for n in tree['root']['children'] if n['type']=='Fallback')
        branch['children'].reverse()
        tree['root']={'type':'Sequence','children':[tree['root']]}
        return req,tree
    def test_gateway_accepts_non_template_tree_and_destination(self):
        req,tree=self.fixture();req['robot_plan']=tree
        with patch.dict(os.environ,{'DRONE_SCENARIO':'3'}):validate_request(req)
        for clear,destination in [(True,'blue_platform'),(False,'red_platform')]:
            actions=[];execute_tree(tree,actions.append,lambda p:clear if p=='gate_clear' else not clear)
            self.assertEqual([a['args'][1] for a in actions if a['skill']=='PLACE'],[destination])
            self.assertEqual(actions[-1]['skill'],'RETURN_HOME')
        broken=copy.deepcopy(tree)
        def remove(n):
            if 'children' in n:
                n['children']=[x for x in n['children'] if x.get('skill')!='INSPECT']
                for x in n['children']:remove(x)
        remove(broken)
        remove(broken['root'])
        with self.assertRaises(ValueError):validate_generated_tree(broken,req['mission_spec'])
    def test_browser_uses_provider_topology_and_executes_red_alternate(self):
        req,tree=self.fixture();nodes=[]
        def flatten(n):
            ident='n'+str(len(nodes));row={'id':ident,'kind':n['type'],'children':[],'skill':n.get('skill',''),'predicate':n.get('predicate',''),'args':n.get('args',[])};nodes.append(row)
            row['children']=[flatten(c) for c in n.get('children',[])];return ident
        root_id=flatten(tree['root']);proposal={'format':'behavior_tree_nodes_v1','root_id':root_id,'nodes':nodes,'rationale':'Mock provider fixture'}
        root=Path(__file__).resolve().parents[2]
        source=(root/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case='\nconst req='+json.dumps(req)+';const proposal='+json.dumps(proposal)+';const expected='+json.dumps(tree)+''';
document.getElementById('scenario').value='3';applyScenario();
world.gateObstacle={x:world.gate.x,y:world.gate.y,w:50,h:50};
missionSpec=req.mission_spec;validationResult={status:'ACCEPTED',checks:[]};
document.getElementById('apiKey').value='mock-key';document.getElementById('provider').value='openai';
let pddlCalls=0;const originalCompilePddl=compilePddl;compilePddl=(...args)=>{pddlCalls++;return originalCompilePddl(...args)};validate(missionSpec);if(pddlCalls)throw new Error('Validation required PDDL');
let calls=0;callOpenAI=async(key,model,goal,options)=>{calls++;if(options.schema!==BT_PROPOSAL_SCHEMA||!options.prompt.includes('Worksheet:'))throw new Error('Missing planner context');return proposal};
log=()=>{};sleep=async()=>{};animate=async dest=>{world.drone.x=dest.x;world.drone.y=dest.y;if(verifiedState.holding){world.target.x=dest.x+24;world.target.y=dest.y+20}};
(async()=>{await document.getElementById('planBtn').onclick();
if(calls!==1||robotPlannerSource!=='llm'||JSON.stringify(robotBehaviorTree)!==JSON.stringify(expected))throw new Error('Provider tree changed or rejected');
await executePlan();if(session.outcome!=='MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS'||verifiedState.placedOn!=='red_platform'||!verifiedState.atHome)throw new Error('Alternate failed: '+session.outcome);
document.getElementById('resetWorldBtn').onclick();
if(JSON.stringify(robotBehaviorTree)!==JSON.stringify(expected)||missionSpec.revision_context||completedGoals.length||verifiedState.holding||verifiedState.placed||verifiedState.inspectedViews.length||document.getElementById('runBtn').disabled||session.outcome!=='NOT_STARTED')throw new Error('Reset lost plan or retained execution facts');
if(calls!==1||!computeGateClear())throw new Error('Reset called provider or failed to restore world');
await executePlan();
if(session.outcome!=='MISSION_COMPLETED'||verifiedState.placedOn!=='blue_platform'||!verifiedState.atHome||calls!==1)throw new Error('Reused plan failed clear-gate rerun');
if(pddlCalls||pddlProblem||session.versions.some(v=>v.stage==='PDDL'||v.stage==='REPLAN_PDDL'))throw new Error('PDDL was generated automatically');
const keptTree=JSON.stringify(robotBehaviorTree),runWasDisabled=document.getElementById('runBtn').disabled;document.getElementById('pddlBtn').onclick();
if(pddlCalls!==1||!pddlProblem.includes('(define (problem')||JSON.stringify(robotBehaviorTree)!==keptTree)throw new Error('Optional PDDL export failed');
compilePddl=()=>{throw new Error('Mock export failure')};document.getElementById('pddlBtn').onclick();
if(JSON.stringify(robotBehaviorTree)!==keptTree||document.getElementById('runBtn').disabled!==runWasDisabled)throw new Error('Optional export failure blocked execution');
document.getElementById('scenario').value='1';document.getElementById('scenario').onchange();
if(robotBehaviorTree||reusablePlan||!document.getElementById('runBtn').disabled)throw new Error('Scenario change retained stale plan');
print('PASS mocked LLM tree preserved, red alternate executed, reset and blue rerun without LLM');})().catch(e=>{printerr(e.message);imports.system.exit(1)});
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((root/'tests/browser_stub.js').read_text()+source+case)
            completed=subprocess.run(['gjs',str(p)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            self.assertEqual(completed.returncode,0,completed.stdout)
            result=completed.stdout
        self.assertIn('PASS mocked LLM',result)
