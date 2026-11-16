"""Read-only RGB/depth red-target measurement; never publishes motion."""
import json,time
import message_filters
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from cv_bridge import CvBridge
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation
cv2.setNumThreads(1)
class Observer(Node):
 def __init__(self):
  super().__init__('red_cube_observer');self.msgs={};self.bridge=CvBridge();self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  for key,topic,typ in [('rgb','/depth_cam/rgb0/image_raw',Image),('depth','/depth_cam/depth0/image_raw',Image),('rgb_info','/depth_cam/rgb0/camera_info',CameraInfo),('depth_info','/depth_cam/depth0/camera_info',CameraInfo),('scan','/scan',LaserScan)]:
   if key not in ('rgb','depth'): self.create_subscription(typ,topic,lambda m,k=key:self.msgs.__setitem__(k,(m,time.monotonic())),qos_profile_sensor_data)
  self.rgb_sub=message_filters.Subscriber(self,Image,'/depth_cam/rgb0/image_raw',qos_profile=qos_profile_sensor_data)
  self.depth_sub=message_filters.Subscriber(self,Image,'/depth_cam/depth0/image_raw',qos_profile=qos_profile_sensor_data)
  self.sync=message_filters.ApproximateTimeSynchronizer([self.rgb_sub,self.depth_sub],10,.03)
  self.sync.registerCallback(self.paired)
 def paired(self,rgb,depth):
  now=time.monotonic();self.msgs['rgb']=(rgb,now);self.msgs['depth']=(depth,now)
 def report(self):
  now=time.monotonic();out={'ages_s':{k:round(now-t,3) for k,(m,t) in self.msgs.items()}}
  if 'rgb' not in self.msgs: return dict(out,error='No RGB image')
  rgb=self.bridge.imgmsg_to_cv2(self.msgs['rgb'][0],'bgr8');hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
  mask=cv2.inRange(hsv,(0,100,55),(12,255,255))|cv2.inRange(hsv,(168,100,55),(179,255,255))
  mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8));targets=[]
  for c in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]:
   area=cv2.contourArea(c)
   if area<100:continue
   x,y,w,h=cv2.boundingRect(c)
   if not .4<w/h<2.5:continue
   targets.append({'bbox':[x,y,w,h],'area_px':area,'center':[x+w/2,y+h/2]})
  out['red_candidates']=targets;out['rgb_shape']=list(rgb.shape)
  for k in ('rgb_info','depth_info'):
   if k in self.msgs:
    m=self.msgs[k][0];out[k]={'frame':m.header.frame_id,'k':list(m.k),'size':[m.width,m.height]}
  if 'depth' in self.msgs:
   m=self.msgs['depth'][0];depth=self.bridge.imgmsg_to_cv2(m,'passthrough');out['depth_encoding']=m.encoding;out['depth_shape']=list(depth.shape);out['depth_frame']=m.header.frame_id
   # Same-pixel measurement is diagnostic only until registration is verified.
   for t in targets:
    x,y=map(int,t['center'])
    if depth.shape!=rgb.shape[:2]:continue
    roi=depth[max(0,y-3):y+4,max(0,x-3):x+4];vals=roi[np.isfinite(roi)&(roi>0)]
    scale=.001 if m.encoding in ('16UC1','mono16') else 1.0
    t['same_pixel_depth_m_unverified_registration']=round(float(np.median(vals))*scale,4) if vals.size else None
   try:
    tf=self.tf.lookup_transform('base_link',self.msgs['rgb'][0].header.frame_id,rclpy.time.Time()).transform
    q=tf.rotation;rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();tr=np.array([tf.translation.x,tf.translation.y,tf.translation.z]);k=self.msgs['rgb_info'][0].k
    for t in targets:
     z=t.get('same_pixel_depth_m_unverified_registration')
     if z:
      u,v=t['center'];pt=rot@np.array([(u-k[2])*z/k[0],(v-k[5])*z/k[4],z])+tr
      t['base_xyz_m_assuming_aligned_depth']=pt.tolist()
   except Exception as e:out['transform_error']=str(e)
   out['rgb_depth_stamp_delta_s']=abs((self.msgs['rgb'][0].header.stamp.sec-m.header.stamp.sec)+(self.msgs['rgb'][0].header.stamp.nanosec-m.header.stamp.nanosec)/1e9)
  if 'scan' in self.msgs:
   s=self.msgs['scan'][0];ranges=np.array(s.ranges);angles=s.angle_min+np.arange(len(ranges))*s.angle_increment;angles=np.arctan2(np.sin(angles+np.pi),np.cos(angles+np.pi));valid=np.isfinite(ranges)&(ranges>=s.range_min)&(ranges<=s.range_max)&(np.abs(angles)<.45)
   out['scan_frame']=s.header.frame_id;out['front_scan_min_m']=float(ranges[valid].min()) if valid.any() else None
  return out
if __name__=='__main__':
 rclpy.init();n=Observer();end=time.monotonic()+8
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 print(json.dumps(n.report(),indent=2));n.destroy_node();rclpy.shutdown()
