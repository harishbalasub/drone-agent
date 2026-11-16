import base64,io,sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.mission import Executor
from drone_sim.transport import png_rgb
from test_gateway import IdleTransport
class EventCameraEvidence(unittest.TestCase):
    def test_capture_is_frozen_and_stale_frames_are_not_current(self):
        t=IdleTransport();e=Executor(t)
        try:
            now=time.monotonic();t.images['front']=png_rgb(2,2,b'\xff\x00\x00'*4,6);t.images['down']=t.images['front'];t.state['cameras']={'front':{'received':now},'down':{'received':now-8}}
            frames=e.camera_evidence();self.assertTrue(frames[0]['data_url'].startswith('data:image/jpeg;base64,'));self.assertIn('stale',frames[1]['reason']);self.assertNotIn('data_url',frames[1])
            old=frames[0]['data_url'];t.images['front']=b'not an image';self.assertEqual(frames[0]['data_url'],old)
            self.assertIn('decoded',e.camera_evidence()[0]['reason'])
        finally:e.close()
    def test_only_successful_milestones_capture_images(self):
        t=IdleTransport();e=Executor(t);e.run_id='test';e.session={'events':[]}
        try:
            failed=e.event('ACTION_RESULT',action={'skill':'PICK'},result='FAILED');self.assertNotIn('camera_evidence',failed)
            success=e.event('ACTION_RESULT',action={'skill':'PICK'},result='SUCCEEDED');self.assertEqual(len(success['camera_evidence']),2);self.assertIn('No camera frame',success['camera_evidence'][0]['reason'])
        finally:e.close()
