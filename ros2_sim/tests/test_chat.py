import subprocess,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
class ChatSupervisor(unittest.TestCase):
    def run_case(self,case):
        source=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        code=(ROOT/'tests/browser_stub.js').read_text()+source+(ROOT/'ros2_sim/tests/chat_dom_stub.js').read_text()+(ROOT/'ros2_sim/chat.js').read_text()+"\n(async()=>{\n"+case+"\nprint('PASS chat');})().catch(e=>{printerr(e.stack);imports.system.exit(1)});"
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'chat.js';p.write_text(code)
            r=subprocess.run(['gjs',str(p)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        self.assertEqual(r.returncode,0,r.stdout);self.assertIn('PASS chat',r.stdout)
    def test_assumption_angle_edit_validation_and_stale_actions(self):
        self.run_case('''
await document.getElementById('generateBtn').onclick();
if(!missionSpec.assumptions.some(a=>a.constraint_name==='minimum_view_separation_deg'))throw new Error('Hidden system angle');
document.getElementById('validateBtn').onclick();if(validationResult.status!=='NEEDS_CLARIFICATION')throw new Error('Assumption did not block');
const stale=action('Accept assumption');await action('Change angle').onclick();
const editor=descendants(document.getElementById('chatThread')).filter(n=>n.className==='chat-editor').at(-1);editor.children[0].value='45';editor.children[1].onclick();
if(missionSpec.constraints[0].params.degrees!==45||missionSpec.assumptions.length||robotPlan.length||validationResult.status!=='ACCEPTED')throw new Error('Angle edit did not revalidate');
await stale.onclick();if(missionSpec.constraints[0].params.degrees!==45)throw new Error('Old card changed mission');
document.getElementById('battery').value=20;document.getElementById('validateBtn').onclick();
if(validationResult.status!=='REJECTED'||!session.events.some(e=>e.message_type==='CHAT_MESSAGE'&&e.title==='Mission cannot start'&&e.text.includes('reserve')))throw new Error('Battery issue not surfaced');
''')
    def test_confirmation_milestones_and_reset(self):
        self.run_case('''
log=()=>{};sleep=async()=>{};animate=async d=>{world.drone.x=d.x;world.drone.y=d.y;if(verifiedState.holding){world.target.x=d.x+24;world.target.y=d.y+20}};
await document.getElementById('generateBtn').onclick();await action('Accept assumption').onclick();
if(validationResult.status!=='ACCEPTED'||!missionSpec.user_confirmed_assumptions.length)throw new Error('Assumption confirm failed');
await document.getElementById('planBtn').onclick();await document.getElementById('runBtn').onclick();
for(const title of ['Package located','Package picked up','Gate traversed','Package placed','Returned home','Mission completed'])if(!session.events.some(e=>e.message_type==='CHAT_MESSAGE'&&e.title===title))throw new Error('Missing '+title);
const picked=session.events.find(e=>e.title==='Package picked up');if(picked.frames.length!==2||!picked.frames[0].label.includes('not a camera')||!picked.evidence.facts.holding)throw new Error('Wrong milestone evidence');
document.getElementById('resetWorldBtn').onclick();if(!robotBehaviorTree||document.getElementById('runBtn').disabled)throw new Error('Chat broke reuse');
''')
    def test_llm_chat_proposes_before_applying_and_blocks_inflight_edits(self):
        self.run_case('''
await document.getElementById('generateBtn').onclick();const original=JSON.stringify(missionSpec);
document.getElementById('apiKey').value='mock-secret';let calls=0;
callOpenAI=async(k,m,g,options)=>{calls++;if(!options.prompt.includes('Current mission:'))throw new Error('No state context');return {intent:'AMEND_MISSION',revised_mission:'Find the red package and return home.',clarification:''}};
document.getElementById('chatInput').value='Skip inspection';await DroneChat.send();
if(calls!==1||JSON.stringify(missionSpec)!==original||!action('Apply and validate'))throw new Error('Unreviewed mutation');
running=true;document.getElementById('chatInput').value='Deliver to red instead';await DroneChat.send();if(calls!==1)throw new Error('Modified inflight mission');running=false;
DroneChat.externalState({run_id:'test',status:'EXECUTING',facts:{holding:true},battery:{available:false}});document.getElementById('executionBackend').value='external';DroneChat.status();
if(!session.events.some(e=>e.title==='Live robot status'&&e.text.includes('unavailable; reserve not verified')))throw new Error('Invented battery');
DroneChat.externalEvent({run_id:'test',sequence:1,message_type:'ACTION_RESULT',result:'FAILED',action:{skill:'PICK',args:['red_package']}},{transport:'test'});
if(session.events.some(e=>e.title==='Package picked up'))throw new Error('Failed action reported success');
''')
