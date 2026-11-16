import json
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import make_handler
from drone_sim.transport import Sensors
from drone_sim.mission import Executor

class IdleTransport(Sensors):
    kind='test_only'
    def velocity(self,v,yaw_rate=0):pass
    def grip(self,close):pass
    def emit(self,*a):pass
    def close(self):pass

class HTTPContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ex=Executor(IdleTransport());cls.http=ThreadingHTTPServer(('127.0.0.1',0),make_handler(cls.ex));cls.base='http://127.0.0.1:'+str(cls.http.server_port)
        cls.thread=threading.Thread(target=cls.http.serve_forever,daemon=True);cls.thread.start()
    @classmethod
    def tearDownClass(cls):cls.http.shutdown();cls.http.server_close();cls.ex.close();cls.thread.join()
    def test_app_and_legacy_are_served(self):
        for page in ['accountable_drone_agent_ros2_v3.html','accountable_drone_agent_prototype_v2.html']:
            with urllib.request.urlopen(self.base+'/'+page) as r:self.assertEqual(r.status,200);r.read()
    def test_state_reports_disconnected(self):
        with urllib.request.urlopen(self.base+'/api/sim/state') as r:self.assertFalse(json.load(r)['connected'])
    def test_external_save_includes_chat_audit(self):
        from unittest.mock import patch
        from drone_sim.mission import ROOT
        previous=self.ex.session
        self.ex.session={'session_id':'chat-test','started_at':'test','events':[{'message_type':'MISSION_RESULT'}],'versions':[],'interrupts':[],'human_decisions':[],'outcome':'TEST','metrics':{}}
        chat={'message_type':'CHAT_MESSAGE','text':'Mission needs review','frames':[]}
        try:
            with patch('server.save_session',return_value=ROOT/'session_logs/test') as save:
                req=urllib.request.Request(self.base+'/api/sim/save',data=json.dumps({'directory':'session_logs/test','ui_events':[chat]}).encode(),headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(req) as response:self.assertEqual(response.status,200)
                self.assertEqual(save.call_args.args[1]['events'][-1],chat)
        finally:self.ex.session=previous

    def test_no_fake_camera(self):
        with self.assertRaises(urllib.error.HTTPError) as e:urllib.request.urlopen(self.base+'/api/sim/camera/front')
        self.assertEqual(e.exception.code,404)
    def test_unknown_endpoint_and_cross_origin_rejected(self):
        for path,headers,status in [('unknown',{},404),('stop',{'Origin':'https://another-site.example'},403)]:
            req=urllib.request.Request(self.base+'/api/sim/'+path,data=b'{}',headers={'Content-Type':'application/json',**headers})
            with self.assertRaises(urllib.error.HTTPError) as e:urllib.request.urlopen(req)
            self.assertEqual(e.exception.code,status)

if __name__=='__main__':unittest.main()
