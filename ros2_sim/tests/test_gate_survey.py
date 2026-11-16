"""Gate evidence must be acquired at the gate, even without a NAVIGATE node."""
import math,time,unittest
from unittest.mock import patch
from test_gateway import IdleTransport
from drone_sim.mission import Executor

class GateSurveyContracts(unittest.TestCase):
    def test_survey_moves_and_faces_gate_before_using_observations(self):
        for blocked in [False,True]:
            t=IdleTransport();e=Executor(t);calls=[]
            def move(position):
                calls.append(('move',position,e.heading))
                now=time.monotonic();t.state.update(position=position,pose_received=now,semantic_received={'front':now,'down':now})
                t.state['objects']={n:{'position':[0,0,0],'received':now} for n in ['gate_left','gate_right']+(['gate_blocker'] if blocked else [])}
            e.move=move;e.hold=lambda seconds:calls.append(('hold',seconds))
            try:
                self.assertEqual(e.survey_gate(),not blocked)
                self.assertEqual(calls[0],('move',[5.55,2.8,1.6],math.pi/2))
                self.assertEqual(calls[1],('hold',.6))
            finally:e.close()
    def test_unknown_is_never_treated_as_blocked_or_clear(self):
        e=Executor(IdleTransport());e.move=lambda p:None;e.hold=lambda seconds:None
        try:
            with patch.object(e,'gate_clear',side_effect=RuntimeError('GATE_CLEAR_UNKNOWN')),patch('drone_sim.mission.time.monotonic',side_effect=[0,4]):
                with self.assertRaisesRegex(RuntimeError,'GATE_CLEAR_UNKNOWN'):e.survey_gate()
        finally:e.close()
