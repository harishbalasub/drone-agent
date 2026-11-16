"""ROSOrin-specific short, straight approach. Default is read-only."""
import sys,time,math,json,signal
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from observe_red_cube import Observer
rclpy.init();n=Observer();pub=n.create_publisher(Twist,'/controller/cmd_vel',1)
odom=[]
n.create_subscription(Odometry,'/odom',lambda m:odom.__setitem__(slice(None),[m.pose.pose.position.x,m.pose.pose.position.y,time.monotonic()]),10)
def stop():
 for _ in range(5):pub.publish(Twist());time.sleep(.05)
def interrupt(a,b):raise RuntimeError('Interrupted')
signal.signal(signal.SIGTERM,interrupt)
execute='--execute' in sys.argv
small_step='--step-2cm' in sys.argv
travel_limit=.03 if small_step else .20
speed=.02 if small_step else .035
try:
 end=time.monotonic()+8
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 origin=odom[:];start=time.monotonic();stable=0
 if len(origin)!=3:raise RuntimeError('No odometry')
 while True:
  for _ in range(30):rclpy.spin_once(n,timeout_sec=0)
  r=n.report();ts=r.get('red_candidates',[])
  if time.monotonic()-start>12:raise RuntimeError('Approach time bound reached')
  if any(r.get('ages_s',{}).get(k,999)>.4 for k in ('rgb','depth','scan')):
   if execute:pub.publish(Twist())
   stable=0;print(json.dumps({'paused_stale':r.get('ages_s')}),flush=True);time.sleep(.05);continue
  if r.get('rgb_depth_stamp_delta_s',999)>.12:raise RuntimeError('Unsynchronized RGB/depth')
  if len(ts)!=1:raise RuntimeError('Need exactly one red target')
  xyz=ts[0].get('base_xyz_m_assuming_aligned_depth')
  if not xyz or not all(math.isfinite(v) for v in xyz):raise RuntimeError('Invalid target depth/transform')
  x,y,z=xyz
  if abs(y)>.045 or not -.015<z<.09:raise RuntimeError('Target outside straight floor approach corridor')
  clearance=r.get('front_scan_min_m')
  if clearance is None or clearance<.35:raise RuntimeError('Forward obstacle or missing scan')
  if len(odom)!=3 or time.monotonic()-odom[2]>.4:raise RuntimeError('Stale odometry')
  travel=math.hypot(odom[0]-origin[0],odom[1]-origin[1])
  if travel>travel_limit or time.monotonic()-start>12:raise RuntimeError('Approach bound reached')
  if not execute:
   print(json.dumps({'dry_run':True,'target_base_m':xyz,'clearance_m':clearance,'max_travel_m':travel_limit,'speed_m_s':speed}));break
  if (small_step and (travel>=.02 or x<=.25)) or (not small_step and x<=.31):print(json.dumps({'approach_complete':True,'target_base_m':xyz,'travel_m':travel}));break
  if x>.65:raise RuntimeError('Target too far for this bounded approach')
  stable+=1
  if stable>=5:
   v=Twist();v.linear.x=speed;pub.publish(v)
  time.sleep(.04)
finally:
 if execute:stop()
 n.destroy_node();rclpy.shutdown()
