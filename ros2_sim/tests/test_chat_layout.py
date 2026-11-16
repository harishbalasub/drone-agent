from html.parser import HTMLParser
from pathlib import Path
import unittest
class Layout(HTMLParser):
    def __init__(self):super().__init__();self.stack=[];self.ids={}
    def handle_starttag(self,tag,attrs):
        a=dict(attrs);hidden='hidden' in a or any(s[1] for s in self.stack)
        if 'id' in a:
            assert a['id'] not in self.ids,'Duplicate DOM ID '+a['id']
            self.ids[a['id']]={'hidden':hidden,'parents':[s[2] for s in self.stack]}
        if tag not in {'input','img','br','hr','link','meta','source','wbr'}:self.stack.append((tag,hidden,a.get('id')))
    def handle_startendtag(self,tag,attrs):
        self.handle_starttag(tag,attrs)
        if tag not in {'input','img','br','hr','link','meta','source','wbr'}:self.handle_endtag(tag)
    def handle_endtag(self,tag):
        for i in range(len(self.stack)-1,-1,-1):
            if self.stack[i][0]==tag:self.stack=self.stack[:i];break
class ChatLayout(unittest.TestCase):
    def test_one_visible_mission_workflow_and_optional_export(self):
        view=Layout();view.feed((Path(__file__).resolve().parents[2]/'accountable_drone_agent_ros2_v3.html').read_text())
        for ident in ['mission','generateBtn','validateBtn','planBtn','runBtn','loadExampleBtn']:
            self.assertTrue(view.ids[ident]['hidden'],ident+' still duplicates the chat workflow')
        for ident in ['chatInput','chatSendBtn','chatDefaultBtn','provider','apiKey','resetWorldBtn','saveSessionBtn']:
            self.assertFalse(view.ids[ident]['hidden'],ident+' should remain accessible')
        self.assertIn('pddlTab',view.ids['pddlBtn']['parents'])
