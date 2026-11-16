import json,os,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.transport import Sensors

class LearnedPerceptionContracts(unittest.TestCase):
    def test_ideal_package_is_blocked_but_gate_stays_available(self):
        with patch.dict(os.environ,{'DRONE_PACKAGE_PERCEPTION':'rgbd'}):s=Sensors()
        s.semantic_json(NS(data=json.dumps({'frame':'map','source':'logical_camera_oracle','objects':{'red_package':{'position':[7,1,.12]},'gate_left':{'position':[4,3,1]}}})),'down')
        self.assertNotIn('red_package',s.snapshot()['objects'])
        self.assertIn('gate_left',s.snapshot()['objects'])
    def test_missing_prediction_does_not_fall_back_to_truth(self):
        with patch.dict(os.environ,{'DRONE_PACKAGE_PERCEPTION':'rgbd'}):s=Sensors()
        s.package_json(NS(data=json.dumps({'frame':'map','source':'learned_rgbd','valid':False})))
        self.assertNotIn('red_package',s.snapshot()['objects'])
    def test_learned_pose_has_explicit_provenance(self):
        with patch.dict(os.environ,{'DRONE_PACKAGE_PERCEPTION':'rgbd'}):s=Sensors()
        s.package_json(NS(data=json.dumps({'frame':'map','source':'learned_rgbd','valid':True,'stamp_ns':123,'estimate':{'position':[7,1,.12]}})))
        self.assertEqual(s.snapshot()['objects']['red_package']['source'],'learned_rgbd')
        self.assertEqual(s.snapshot()['objects']['red_package']['stamp_ns'],123)
if __name__=='__main__':unittest.main()
