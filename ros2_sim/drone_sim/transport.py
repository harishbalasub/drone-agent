"""Two explicit transports: ROS 2 deployment and Gazebo-native integration tests.

Logical cameras provide ideal scene semantics. Optional RGB-D mode blocks their
package observations and consumes the separately inferred learned package topic.
Odometry uses Gazebo truth as a simulated motion-capture source (AirVLA-like).
"""
import base64
import copy
import math
import os
import struct
import threading
import time
import zlib


def xyz(v): return [float(v.x), float(v.y), float(v.z)]


def rotate(q, v):
    # Unit-quaternion vector rotation, quaternion order x,y,z,w.
    u=[q.x,q.y,q.z]; w=q.w
    cross=lambda a,b:[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    uv=cross(u,v); uuv=cross(u,uv)
    return [v[i]+2*(w*uv[i]+uuv[i]) for i in range(3)]


def png_rgb(width, height, data, step):
    """Small dependency-free RGB PNG for browser sensor previews."""
    def chunk(t,b): return struct.pack('!I',len(b))+t+b+struct.pack('!I',zlib.crc32(t+b)&0xffffffff)
    raw=b''.join(b'\x00'+data[y*step:y*step+width*3] for y in range(height))
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw,1))+chunk(b'IEND',b'')


class Sensors:
    def __init__(self):
        self.lock=threading.RLock()
        self.state={'position':None,'velocity':[0,0,0],'yaw':0.,'pose_received':0.,
                    'objects':{},'gripper':'unknown','gripper_received':0.,'contacts':[],
                    'contact_received':0.,'imu_received':0.,'cameras':{},'semantic_received':{},'depth':None,'sequence':0}
        self.images={}
        self.learned_package=os.environ.get("DRONE_PACKAGE_PERCEPTION")=="rgbd"

    def clear_observations(self,preserve_feedback=False):
        with self.lock:
            feedback={k:self.state[k] for k in ['position','velocity','yaw','pose_received','gripper','gripper_received','imu_received']} if preserve_feedback else {}
            self.state.clear()
            self.state.update(position=None,velocity=[0,0,0],yaw=0.,pose_received=0.,objects={},gripper='unknown',gripper_received=0.,contacts=[],contact_received=0.,imu_received=0.,cameras={},semantic_received={},depth=None,sequence=0)
            self.state.update(feedback)
            self.images.clear()

    def snapshot(self):
        with self.lock: return copy.deepcopy(self.state)

    def odometry(self,m,ros=False):
        p=m.pose.pose if ros else m.pose
        t=m.twist.twist if ros else m.twist
        q=p.orientation
        with self.lock:
            self.state.update(position=xyz(p.position),velocity=xyz(t.linear),
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),pose_received=time.monotonic())
            self.state['sequence']+=1

    def semantic(self,m,camera,ros=False):
        # Logical-camera pose is in world; model poses are in camera coordinates.
        models=m.models if ros else m.model
        now=time.monotonic(); cp=m.pose
        with self.lock:
            self.state['semantic_received'][camera]=now
            for model in models:
                local=xyz(model.pose.position); rotated=rotate(cp.orientation,local)
                name=model.name.split('::')[0]
                if self.learned_package and name=='red_package':continue
                self.state['objects'][name]={'position':[a+b for a,b in zip(xyz(cp.position),rotated)],
                    'received':now,'camera':camera,'source':'logical_camera_oracle'}

    def semantic_json(self,m,camera):
        import json
        try:
            value=json.loads(m.data)
            if value.get('frame')!='map' or value.get('source')!='logical_camera_oracle':return
            now=time.monotonic()
            with self.lock:
                self.state['semantic_received'][camera]=now
                for name,obj in value['objects'].items():
                    if self.learned_package and name=='red_package':continue
                    pos=obj['position']
                    if len(pos)!=3 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in pos):continue
                    self.state['objects'][name]={'position':pos,'received':now,'camera':camera,'source':value['source']}
        except (ValueError,TypeError,KeyError):return

    def package_json(self,m):
        import json
        if not self.learned_package:return
        try:
            v=json.loads(m.data)
            if v.get('source')!='learned_rgbd' or v.get('frame')!='map':return
            with self.lock:
                self.state['package_perception']={**v,'received':time.monotonic()}
                if not v.get('valid'):return
                pos=v['estimate']['position']
                if len(pos)!=3 or not all(math.isfinite(x) for x in pos):return
                self.state['objects']['red_package']={'position':pos,'received':time.monotonic(),'camera':'down','source':'learned_rgbd','stamp_ns':v['stamp_ns'],'quality':v['estimate']}
        except (ValueError,KeyError,TypeError):return

    def gripper(self,m):
        with self.lock: self.state.update(gripper=m.data,gripper_received=time.monotonic())

    def contacts(self,m,ros=False):
        contacts=m.contacts if ros else m.contact
        names=[]
        for c in contacts:
            # ROS uses strings, Gazebo uses Entity messages.
            a=c.collision1; b=c.collision2
            names.append([a if isinstance(a,str) else a.name,b if isinstance(b,str) else b.name])
        with self.lock: self.state.update(contacts=names,contact_received=time.monotonic())

    def imu(self,m):
        with self.lock: self.state['imu_received']=time.monotonic()

    def depth(self,m,ros=False):
        if ros and m.encoding!='32FC1':return
        data=bytes(m.data)
        if len(data)<m.step*m.height or m.step<m.width*4:return
        values=[]
        for y in range(m.height//2-3,m.height//2+4):
            for x in range(m.width//2-3,m.width//2+4):
                d=struct.unpack_from('>f' if ros and m.is_bigendian else '<f',data,y*m.step+x*4)[0]
                if math.isfinite(d) and d>0:values.append(d)
        with self.lock:self.state['depth']={'received':time.monotonic(),'center_median_m':sorted(values)[len(values)//2] if values else None,'width':m.width,'height':m.height}

    def image(self,m,camera,ros=False):
        if ros and m.encoding not in ('rgb8','bgr8'): return
        data=bytes(m.data); step=m.step
        if ros and m.encoding=='bgr8':
            data=bytearray(data)
            for i in range(0,len(data)-2,3): data[i],data[i+2]=data[i+2],data[i]
            data=bytes(data)
        if len(data)<step*m.height or step<m.width*3: return
        png=png_rgb(m.width,m.height,data,step)
        with self.lock:
            self.images[camera]=png
            self.state['cameras'][camera]={'width':m.width,'height':m.height,'received':time.monotonic()}


class GazeboTransport(Sensors):
    kind='gazebo_native'
    def reset_world(self,scenario=None):
        from drone_sim.world_reset import reset_gazebo
        return reset_gazebo(self,scenario=scenario)

    def __init__(self):
        super().__init__()
        from gz.transport13 import Node
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.logical_camera_image_pb2 import LogicalCameraImage
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.twist_pb2 import Twist
        from gz.msgs10.empty_pb2 import Empty
        self.Twist=Twist; self.Empty=Empty; self.node=Node()
        self.node.subscribe(Odometry,'/drone/odometry',self.odometry)
        for camera in ['down','front']:
            self.node.subscribe(LogicalCameraImage,'/drone/semantic_'+camera,lambda m,c=camera:self.semantic(m,c))
        self.node.subscribe(StringMsg,'/drone/gripper/state',self.gripper)
        self.node.subscribe(StringMsg,'/drone/perception/package',self.package_json)
        self.node.subscribe(Contacts,'/drone/contacts',self.contacts)
        self.node.subscribe(IMU,'/drone/imu',self.imu)
        for camera in ['front','down','external']:
            self.node.subscribe(Image,f'/drone/camera/{camera}/image',lambda m,c=camera:self.image(m,c))
        self.node.subscribe(Image,'/drone/camera/down/depth_image',self.depth)
        self.cmd=self.node.advertise('/drone/cmd_vel',Twist)
        self.attach=self.node.advertise('/drone/gripper/attach',Empty)
        self.detach=self.node.advertise('/drone/gripper/detach',Empty)

    def velocity(self,v,yaw_rate=0):
        m=self.Twist(); m.linear.x,m.linear.y,m.linear.z=v; m.angular.z=yaw_rate
        self.cmd.publish(m)

    def grip(self,close): (self.attach if close else self.detach).publish(self.Empty())
    def emit(self,topic,data): pass  # ROS status topics only exist in ROS mode.
    def close(self): self.velocity([0,0,0])


class RosTransport(Sensors):
    kind='ros2'
    def reset_world(self,scenario=None):
        from drone_sim.world_reset import reset_gazebo
        return reset_gazebo(self,scenario=scenario)

    def __init__(self):
        super().__init__()
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Image, Imu
        from std_msgs.msg import String, Empty
        from ros_gz_interfaces.msg import Contacts
        self.rclpy=rclpy; rclpy.init(); self.node=Node('drone_mission_gateway')
        self.Twist=Twist; self.Empty=Empty; self.String=String; self.publishers={}
        self.node.create_subscription(Odometry,'/drone/odometry',lambda m:self.odometry(m,True),qos_profile_sensor_data)
        for camera in ['down','front']:
            self.node.create_subscription(String,'/drone/perception/'+camera,lambda m,c=camera:self.semantic_json(m,c),10)
        self.node.create_subscription(String,'/drone/gripper/state',self.gripper,10)
        self.node.create_subscription(String,'/drone/perception/package',self.package_json,10)
        self.node.create_subscription(Contacts,'/drone/contacts',lambda m:self.contacts(m,True),qos_profile_sensor_data)
        self.node.create_subscription(Imu,'/drone/imu',self.imu,qos_profile_sensor_data)
        for camera in ['front','down','external']:
            self.node.create_subscription(Image,f'/drone/camera/{camera}/image',lambda m,c=camera:self.image(m,c,True),qos_profile_sensor_data)
        self.node.create_subscription(Image,'/drone/camera/down/depth_image',lambda m:self.depth(m,True),qos_profile_sensor_data)
        self.cmd=self.node.create_publisher(Twist,'/drone/cmd_vel',10)
        self.attach=self.node.create_publisher(Empty,'/drone/gripper/attach',10)
        self.detach=self.node.create_publisher(Empty,'/drone/gripper/detach',10)
        self.thread=threading.Thread(target=rclpy.spin,args=(self.node,),daemon=True); self.thread.start()

    def velocity(self,v,yaw_rate=0):
        m=self.Twist(); m.linear.x,m.linear.y,m.linear.z=map(float,v); m.angular.z=float(yaw_rate); self.cmd.publish(m)
    def grip(self,close): (self.attach if close else self.detach).publish(self.Empty())
    def emit(self,topic,data):
        import json
        if topic not in self.publishers: self.publishers[topic]=self.node.create_publisher(self.String,topic,10)
        m=self.String();m.data=json.dumps(data); self.publishers[topic].publish(m)
    def close(self):
        self.velocity([0,0,0]); self.rclpy.shutdown();self.thread.join(timeout=2);self.node.destroy_node()
