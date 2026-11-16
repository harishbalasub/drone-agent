"""Tab-only credential lifecycle without live provider calls."""
import subprocess,tempfile,unittest
from pathlib import Path

class KeySession(unittest.TestCase):
    def test_opt_in_reload_provider_switch_forget_and_unavailable_storage(self):
        root=Path(__file__).resolve().parents[2]
        source=(root/'accountable_drone_agent_ros2_v3.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
        storage="const storage=new Map();let sessionStorage={getItem:k=>storage.get(k)||null,setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)};\n"
        case='''
document.getElementById('apiKey').value='mock-openai';rememberCurrentKey();
if(storage.size)throw new Error('Stored without opt-in');
document.getElementById('rememberApiKey').onchange({target:{checked:true}});
document.getElementById('provider').value='gemini';document.getElementById('provider').onchange({target:{value:'gemini'}});
if(document.getElementById('apiKey').value)throw new Error('Leaked OpenAI key into Gemini');
document.getElementById('apiKey').value='mock-gemini';document.getElementById('apiKey').oninput();
tabCredentials={enabled:false,provider:'openai',keys:{},models:{}};document.getElementById('apiKey').value='';loadTabCredentials();
if(document.getElementById('apiKey').value!=='mock-gemini'||document.getElementById('provider').value!=='gemini')throw new Error('Reload failed');
document.getElementById('resetWorldBtn').onclick();if(document.getElementById('apiKey').value!=='mock-gemini')throw new Error('Reset lost key');
if(JSON.stringify(session).includes('mock-gemini'))throw new Error('Credential entered mission logs');
document.getElementById('provider').value='openai';document.getElementById('provider').onchange({target:{value:'openai'}});
if(document.getElementById('apiKey').value!=='mock-openai')throw new Error('Per-provider restore failed');
document.getElementById('forgetApiKeys').onclick();if(storage.size||document.getElementById('apiKey').value)throw new Error('Forget failed');
sessionStorage.setItem=()=>{throw new Error('Storage blocked')};document.getElementById('apiKey').value='mock-current';document.getElementById('rememberApiKey').onchange({target:{checked:true}});
if(tabCredentials.enabled||document.getElementById('apiKey').value!=='mock-current')throw new Error('Storage failure handling');
print('PASS tab credentials lifecycle');
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'test.js';p.write_text((root/'tests/browser_stub.js').read_text()+storage+source+case)
            result=subprocess.run(['gjs',str(p)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        self.assertEqual(result.returncode,0,result.stdout);self.assertIn('PASS tab credentials',result.stdout)
