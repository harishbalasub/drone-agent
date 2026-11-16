"""Exercise actual v3 request schemas, not a reduced hand-written fixture."""
import io,json,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import prototype_server as server
class ProviderSchemas(unittest.TestCase):
    def test_real_schemas_have_no_empty_enums_and_preserve_constraint_links(self):
        source=(ROOT/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        case="""
const spec=normalizeMissionSpec({assumptions:[{text:'Unrelated planning premise',source:'EXPLICIT_ASSUMPTION',constraint_name:'none'},{text:'30 degree viewpoint separation',source:'EXPLICIT_ASSUMPTION',constraint_name:'minimum_view_separation_deg'}]},'Inspect the package');
if(spec.assumptions[0].constraint_name||spec.assumptions[1].constraint_name!=='minimum_view_separation_deg')throw new Error('Constraint normalization failed');
if(!compilerPrompt('test').includes('constraint_name=none'))throw new Error('Prompt does not explain sentinel');
print(JSON.stringify({schemas:[MISSION_SCHEMA,BT_PROPOSAL_SCHEMA],direct:[geminiResponseSchema(MISSION_SCHEMA),geminiResponseSchema(BT_PROPOSAL_SCHEMA)]}));
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'schemas.js';p.write_text((ROOT/'tests/browser_stub.js').read_text()+source+case)
            data=json.loads(subprocess.check_output(['gjs',str(p)],text=True).strip().splitlines()[-1])
        self.assertEqual(data['schemas'][0]['properties']['assumptions']['items']['properties']['constraint_name']['enum'],['none','minimum_view_separation_deg'])
        def check(node):
            if isinstance(node,dict):
                if 'enum' in node:self.assertNotIn('',node['enum'])
                for value in node.values():check(value)
            elif isinstance(node,list):
                for value in node:check(value)
        for schema,direct in zip(data['schemas'],data['direct']):
            adapted=server.gemini_response_schema(schema);self.assertEqual(adapted,direct);check(adapted)
            # Verify the actual proxy payload preserves the valid enum.
            handler=object.__new__(server.Handler);handler.path='/api/llm'
            payload=json.dumps({'provider':'gemini','api_key':'mock','model':'mock','prompt':'test','schema':schema}).encode()
            handler.headers={'Content-Length':str(len(payload))};handler.rfile=io.BytesIO(payload)
            replies=[];handler._json=lambda code,body:replies.append(code)
            with patch.object(server,'post_json',return_value={'candidates':[{'content':{'parts':[{'text':'{}'}]}}]}) as upstream:handler.do_POST()
            self.assertEqual(replies,[200]);self.assertEqual(upstream.call_args.args[1]['generationConfig']['responseSchema'],direct)
