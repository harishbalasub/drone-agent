import copy
import json
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.mission import validate_request,route,intersects,ROOT,Executor
from drone_sim.transport import Sensors,rotate,png_rgb
from evaluate import request

class Contracts(unittest.TestCase):
    def test_default_plan_preserves_all_goals(self):
        spec,plan=validate_request(request());self.assertEqual(len(plan),10)
        self.assertEqual(sum(g['predicate']=='inspected' for g in spec['goals']),2)
    def test_three_views_not_silently_dropped(self):
        r=request(views=3);validate_request(r)
        r['robot_plan']=[a for a in r['robot_plan'] if a['args']!=['red_package','view_2']]
        with self.assertRaisesRegex(ValueError,'omits goal'):validate_request(r)
    def test_wrong_object_and_destination_rejected(self):
        for field,value in [('skill','SHELL'),('args',['other_package'])]:
            r=request();r['robot_plan'][0][field]=value
            with self.assertRaises(ValueError):validate_request(r)
    def test_route_avoids_inflated_obstacle(self):
        boxes=[{'min':[3,1,0],'max':[4,3,2]}];start=[1,2,1.6];end=[7,2,1.6]
        points=route(start,end,boxes);self.assertGreater(len(points),1)
        for a,b in zip([start]+points,points):self.assertFalse(intersects(a[:2],b[:2],[2.65,.65],[4.35,3.35]))
    def test_no_route_out_of_blocked_start(self):
        with self.assertRaises(RuntimeError):route([3.5,2,1.6],[7,2,1.6],[{'min':[3,1,0],'max':[4,3,2]}])
    def test_frame_transform(self):
        class Q:x=0;y=0;z=2**-.5;w=2**-.5
        result=rotate(Q(),[1,0,0]);self.assertAlmostEqual(result[0],0);self.assertAlmostEqual(result[1],1)
    def test_png(self):self.assertTrue(png_rgb(1,1,b'\xff\x00\x00',3).startswith(b'\x89PNG'))
    def test_spec_assumptions_do_not_get_proven_by_schema(self):
        r=request();r['mission_spec']['unresolved']=[{'text':'unknown','reason':'unknown'}]
        with self.assertRaises(ValueError):validate_request(r)
    def test_ordering_and_unsupported_constraints_rejected(self):
        r=request();r['robot_plan'][1],r['robot_plan'][4]=r['robot_plan'][4],r['robot_plan'][1]
        with self.assertRaisesRegex(ValueError,'ordering'):validate_request(r)
        r=request();r['mission_spec']['constraints'].append({'name':'unsupported_rule','params':{}})
        with self.assertRaisesRegex(ValueError,'constraint'):validate_request(r)
    def test_stale_pose_blocks_motion(self):
        from test_gateway import IdleTransport
        import time
        t=IdleTransport();t.state.update(position=[.85,.65,1.6],pose_received=time.monotonic()-5)
        e=Executor(t)
        try:
            with self.assertRaisesRegex(RuntimeError,'STALE_ODOMETRY'):e.sense()
        finally:e.close()
    def test_gate_unknown_blocked_and_clear_are_distinct(self):
        from test_gateway import IdleTransport
        import time
        t=IdleTransport();now=time.monotonic();t.state.update(position=[5.55,2.8,1.6],pose_received=now,semantic_received={'front':now,'down':now})
        e=Executor(t)
        try:
            with self.assertRaisesRegex(RuntimeError,'GATE_CLEAR_UNKNOWN'):e.gate_clear()
            t.state['objects']={k:{'received':now,'position':[0,0,0]} for k in ['gate_left','gate_right']}
            self.assertTrue(e.gate_clear())
            t.state['objects']['gate_blocker']={'received':now,'position':[5.55,3.65,1.1]}
            self.assertFalse(e.gate_clear())
        finally:e.close()
    def test_payload_loss_invalidates_execution(self):
        from test_gateway import IdleTransport
        import time
        t=IdleTransport();t.state.update(position=[.85,.65,1.6],pose_received=time.monotonic(),gripper='detached')
        e=Executor(t);e.facts={'holding':True}
        try:
            with self.assertRaisesRegex(RuntimeError,'PAYLOAD_LOST'):e.sense()
        finally:e.close()
    def test_world_geometry_and_old_files(self):
        import xml.etree.ElementTree as E
        for scenario in [1,2]:
            root=E.parse(ROOT/'worlds'/f'scenario_{scenario}.sdf').getroot();names={m.attrib['name'] for m in root.findall('world/model')}
            self.assertTrue({'drone','red_package','blue_platform','home','gate_left','gate_right','obstacle_1'}<=names)
            self.assertEqual('gate_blocker' in names,scenario==2)

if __name__=='__main__':unittest.main()
