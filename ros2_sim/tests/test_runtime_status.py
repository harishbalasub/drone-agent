import sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.runtime_status import contract_status

class RuntimeEvidence(unittest.TestCase):
    def test_failed_skill_is_not_rendered_successful_even_with_fresh_pose(self):
        session={'events':[{'message_type':'ACTION_STARTED','sequence':1,'action':{'skill':'PICK','args':['red_package']}},{'message_type':'ACTION_RESULT','sequence':2,'result':'FAILED','reason':'GRASP_ALIGNMENT_FAILED'}]}
        s={'position':[7,1,.8],'pose_received':time.monotonic(),'objects':{},'gripper':'detached'}
        c=contract_status(s,session,False,{'sensor_timeout':1.5,'home':[.85,.65,1.6]})
        self.assertEqual(c['status'],'FAILED');self.assertEqual(c['reason'],'GRASP_ALIGNMENT_FAILED')
        self.assertEqual(next(x for x in c['checks'] if x['name']=='Battery reserve')['status'],'unknown')
        self.assertEqual(next(x for x in c['checks'] if x['name']=='Package observation')['status'],'unknown')
    def test_stale_pose_reported_during_return_and_home_does_not_imply_landed(self):
        session={'events':[{'message_type':'ACTION_STARTED','sequence':1,'action':{'skill':'RETURN_HOME','args':[]}}]}
        c=contract_status({'position':[.85,.65,1.6],'pose_received':time.monotonic()-10},session,True,{'sensor_timeout':1.5,'home':[.85,.65,1.6]})
        self.assertEqual(c['status'],'EXECUTING')
        self.assertEqual(c['checks'][0]['status'],'fail')
        self.assertIn('landing is not commissioned',c['completion_behavior'])
if __name__=='__main__':unittest.main()
