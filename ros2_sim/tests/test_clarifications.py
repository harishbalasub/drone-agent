import subprocess,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
class ClarificationFlow(unittest.TestCase):
    def run_case(self,case):
        html=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text();source=html.split('<script>',1)[1].split('</script>',1)[0]
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case.js';p.write_text((ROOT/'tests/browser_stub.js').read_text()+source+case)
            return subprocess.check_output(['gjs',str(p)],text=True)
    def test_partial_confirmation_keeps_other_items_pending_and_invalidates_plan(self):
        self.assertIn('PASS reviews',self.run_case('''
missionSpec=localCompile(document.getElementById('mission').value);missionSpec.assumptions=[{text:'Package remains upright',source:'EXPLICIT_ASSUMPTION'},{text:'Use calibrated camera',source:'EXPLICIT_ASSUMPTION'}];
robotPlan=buildPlan(missionSpec).plan;const before=JSON.stringify(verifiedState);document.getElementById('validateBtn').onclick();
if(validationResult.status!=='NEEDS_CLARIFICATION'||!document.getElementById('clarificationPanel').innerHTML.includes('Use calibrated camera'))throw new Error('No actionable review');
document.getElementById('assumptionConfirm_0').checked=true;document.getElementById('assumptionConfirm_1').checked=false;confirmSelectedAssumptions();
if(missionSpec.assumptions.length!==1||validationResult.status!=='NEEDS_CLARIFICATION'||robotPlan.length||!document.getElementById('runBtn').disabled)throw new Error('Partial review authorized execution');
document.getElementById('assumptionConfirm_0').checked=true;confirmSelectedAssumptions();
if(validationResult.status!=='ACCEPTED'||missionSpec.user_confirmed_assumptions.length!==2)throw new Error('Confirmed reviews still blocked');
if(JSON.stringify(verifiedState)!==before)throw new Error('User review fabricated sensor evidence');
if(!session.versions.some(v=>v.stage==='USER_ASSUMPTION_CONFIRMATION'))throw new Error('No audit trail');print('PASS reviews');
'''))
    def test_assumption_confirmation_does_not_clear_unresolved_ambiguity(self):
        self.assertIn('PASS unresolved',self.run_case('''
missionSpec=localCompile(document.getElementById('mission').value);missionSpec.assumptions=[{text:'Package upright',source:'EXPLICIT_ASSUMPTION'}];missionSpec.unresolved=[{text:'Which platform?',reason:'Two pads exist'}];
document.getElementById('validateBtn').onclick();document.getElementById('assumptionConfirm_0').checked=true;confirmSelectedAssumptions();
if(validationResult.status!=='NEEDS_CLARIFICATION'||missionSpec.unresolved.length!==1)throw new Error('Ambiguity cleared without answer');
document.getElementById('unresolvedAnswer_0').value='Use the blue platform';document.getElementById('clarifiedMission').value=missionSpec.user_goal;
let received='';document.getElementById('generateBtn').onclick=async()=>{received=document.getElementById('mission').value;};
regenerateWithClarifications().then(()=>{if(!received.includes('Which platform?: Use the blue platform'))throw new Error('Answer not sent to compiler');print('PASS unresolved');});
'''))
if __name__=='__main__':unittest.main()
