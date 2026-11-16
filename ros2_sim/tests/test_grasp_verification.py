import copy,time,unittest
from unittest.mock import patch
from test_gateway import IdleTransport
from drone_sim.mission import Executor

class GraspVerification(unittest.TestCase):
    def test_attached_bit_requires_fresh_package_motion(self):
        for package_z,source,received,ok in [(.42,'learned_rgbd',.1,True),(.12,'learned_rgbd',.1,False),(.42,'logical_camera_oracle',.1,False),(.42,'learned_rgbd',-1,False)]:
            t=IdleTransport();e=Executor(t);e.facts={'picked':False,'holding':False};clock=[0.]
            state={'position':[7.,1.,.84],'gripper':'attached','objects':{'red_package':{'position':[7.,1.,package_z],'source':source,'received':received}}}
            e.sense=lambda:copy.deepcopy(state);e.hold=lambda seconds:clock.__setitem__(0,clock[0]+seconds)
            try:
                with patch('drone_sim.mission.time.monotonic',side_effect=lambda:clock[0]):
                    if ok:self.assertEqual(e.verify_payload_lift([7,1,.12],[7,1,.54],timeout=.5),[7,1,.42])
                    else:
                        with self.assertRaisesRegex(RuntimeError,'GRASP_PAYLOAD_NOT_VERIFIED'):e.verify_payload_lift([7,1,.12],[7,1,.54],timeout=.5)
                self.assertFalse(e.facts.get('picked',False))
            finally:e.close()
    def test_failed_test_lift_releases_joint_without_claiming_pickup(self):
        t=IdleTransport();e=Executor(t);e.facts={'picked':False,'holding':False};grips=[];calls=[]
        obj={'position':[7,1,.12],'source':'learned_rgbd','stamp_ns':123,'quality':{}}
        e.observed=lambda *args:copy.deepcopy(obj);e.move=lambda p:None;e.hold=lambda seconds:None;e.sense=lambda:{'position':[7,1,.54],'gripper':'attached'}
        def goto(position,**kwargs):
            calls.append(kwargs)
            if kwargs.get('timeout')==8:raise RuntimeError('WAYPOINT_TIMEOUT')
        e.goto=goto;e.wait_gripper=lambda state:None;t.grip=lambda close:grips.append(close)
        try:
            with self.assertRaisesRegex(RuntimeError,'WAYPOINT_TIMEOUT'):e.pick_rgbd()
            self.assertEqual(grips,[True,False]);self.assertIn({'timeout':8},calls);self.assertFalse(e.facts.get('holding',False));self.assertFalse(e.facts.get('picked',False))
        finally:e.close()
    def test_reset_can_clear_episode_evidence_without_losing_joint_feedback(self):
        t=IdleTransport();t.state.update(position=[1,2,1.6],pose_received=time.monotonic(),gripper='detached',gripper_received=123,objects={'red_package':{}},package_perception={'valid':True})
        t.clear_observations(preserve_feedback=True)
        self.assertEqual(t.state['gripper'],'detached');self.assertEqual(t.state['gripper_received'],123);self.assertEqual(t.state['position'],[1,2,1.6]);self.assertFalse(t.state['objects']);self.assertNotIn('package_perception',t.state)

    def test_airborne_closeup_is_allowed_only_as_grasp_candidate(self):
        import numpy as np
        from rgbd.observation_filter import filter_depth_mask,validate_height
        depth=np.array([[.22,.04,float('nan'),.7]])
        mask=np.ones_like(depth,dtype=bool)
        self.assertEqual(filter_depth_mask(depth,mask).tolist(),[[False,False,False,True]])
        self.assertEqual(filter_depth_mask(depth,mask,True).tolist(),[[True,False,False,True]])
        validate_height([7,1,.42],True,.42)
        with self.assertRaisesRegex(ValueError,'UNSUPPORTED_PACKAGE_SUPPORT_HEIGHT'):validate_height([7,1,.42])
        with self.assertRaisesRegex(ValueError,'PACKAGE_NOT_AT_GRIPPER_HEIGHT'):validate_height([7,1,.12],True,.42)
        with self.assertRaisesRegex(ValueError,'PACKAGE_NOT_AT_GRIPPER_HEIGHT'):validate_height([7,1,.42],True)
