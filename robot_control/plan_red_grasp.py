"""Read-only IK planning: inspected set_pose_target_srv returns pulses, never publishes."""
import argparse,json,time,sys,numpy as np,rclpy
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
from observe_red_cube import Observer
from kinematics_msgs.srv import GetRobotPose,SetRobotPose
from kinematics.kinematics_control import set_pose_target
from sdk import common
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd
parser=argparse.ArgumentParser()
parser.add_argument('--execute',action='store_true')
parser.add_argument('--grasp-height-offset-m',type=float,default=0)
parser.add_argument('--forward-offset-m',type=float,default=0)
parser.add_argument('--pitch-deg',type=float,default=80)
args=parser.parse_args()
if not 30<=args.pitch_deg<=80:raise ValueError('Pitch must be between 30 and 80 degrees')
if not -.015<=args.grasp_height_offset_m<=0:
 raise ValueError('Experimental downward correction must be between -0.015 and 0 metres')
if not 0<=args.forward_offset_m<=.02:
 raise ValueError('Experimental forward correction must be between 0 and 0.02 metres')
rclpy.init();n=Observer()
def call(typ,path,req):
 c=n.create_client(typ,path)
 if not c.wait_for_service(timeout_sec=4):raise RuntimeError(path+' unavailable')
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 if not f.done() or f.result() is None:raise RuntimeError(path+' timed out')
 return f.result()
def verify_joints(expected):
 req=GetBusServoState.Request()
 for sid,value in expected:
  q=GetBusServoCmd();q.id=sid;q.get_position=1;req.cmd.append(q)
 res=call(GetBusServoState,'/ros_robot_controller/bus_servo/get_state',req)
 if not res.success or len(res.state)!=len(expected):raise RuntimeError('Missing actual joint feedback')
 measured={}
 for (sid,value),state in zip(expected,res.state):
  if len(state.position)!=1:raise RuntimeError('Missing actual position for joint '+str(sid))
  measured[sid]=state.position[0]
  if abs(state.position[0]-value)>20:
   raise RuntimeError(f'Joint {sid} failed to follow: target={value}, actual={state.position[0]}. Sequence aborted.')
 print(json.dumps({'verified_actual_joints':measured}),flush=True)
try:
 end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 r=n.report();t=r['red_candidates']
 if len(t)!=1:raise RuntimeError('Need one red target')
 t=t[0];u,v=t['center'];z=t['same_pixel_depth_m_unverified_registration'];k=n.msgs['rgb_info'][0].k
 cam=np.array([(u-k[2])*z/k[0],(v-k[5])*z/k[4],z,1])
 res=call(GetRobotPose,'/kinematics/get_current_pose',GetRobotPose.Request())
 if not res.solution:raise RuntimeError('No current arm FK')
 p=res.pose;endpoint=common.xyz_quat_to_mat([p.position.x,p.position.y,p.position.z],[p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z])
 hand=np.array([[0,0,1,-.101],[-1,0,0,0],[0,-1,0,.037],[0,0,0,1]])
 target=(endpoint@hand@cam)[:3];target[0]+=args.forward_offset_m;poses={}
 for name,dz in [('above',.08),('grasp',args.grasp_height_offset_m),('lift',.10)]:
  pt=target.copy();pt[2]+=dz
  sol=call(SetRobotPose,'/kinematics/set_pose_target',set_pose_target(pt,args.pitch_deg,[-180,180],1))
  poses[name]={'position':pt.tolist(),'pulse':list(sol.pulse),'rpy':list(sol.rpy),'current_pulse':list(sol.current_pulse)}
 print(json.dumps({'observation':r,'arm_target_m':target.tolist(),'forward_offset_m':args.forward_offset_m,'grasp_height_offset_m':args.grasp_height_offset_m,'poses':poses},indent=2),flush=True)
 if args.execute:
  if any(r.get('ages_s',{}).get(k,999)>.4 for k in ('rgb','depth')):raise RuntimeError('Stale target')
  if not (.18<target[0]<.32 and abs(target[1])<.05 and -.015<target[2]<.08):raise RuntimeError('Target outside checked grasp volume')
  for pose in poses.values():
   if len(pose['pulse'])!=5 or not all(20<=x<=980 for x in pose['pulse']):raise RuntimeError('Invalid/unreachable joint solution')
  verify_joints(list(enumerate(poses['above']['current_pulse'],1)))
  pub=n.create_publisher(ServosPosition,'/servo_controller',1)
  deadline=time.monotonic()+4
  while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
  if pub.get_subscription_count()==0:raise RuntimeError('No servo subscriber')
  def move(label,servos,duration):
   print(label,flush=True);set_servo_position(pub,duration,tuple(servos));end=time.monotonic()+duration+.5
   while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
   # Closing may stop on an object; all arm movements and opening must track.
   if label!='close_gripper':verify_joints(servos)
  move('open_gripper',[(10,150)],.8)
  move('above_cube',list(enumerate(poses['above']['pulse'],1)),5.0)
  move('lower_to_cube',list(enumerate(poses['grasp']['pulse'],1)),5.0)
  move('close_gripper',[(10,560)],1)
  move('lift_cube',list(enumerate(poses['lift']['pulse'],1)),5.0)
  print('Pickup motion sequence complete; object retention not yet verified.',flush=True)
finally:n.destroy_node();rclpy.shutdown()
