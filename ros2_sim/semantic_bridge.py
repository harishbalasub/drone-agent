#!/usr/bin/env python3
"""Bridge ideal Gazebo logical-camera observations to stable ROS 2 JSON topics.

Humble ros_gz binary packages do not expose LogicalCameraImage. This small
adapter performs coordinate conversion and publishes std_msgs/String instead.
No RGB detection or learned perception is claimed.
"""
import json
import threading
import time
import rclpy
from rclpy.node import Node as RosNode
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.logical_camera_image_pb2 import LogicalCameraImage
from drone_sim.transport import Sensors


def main():
    rclpy.init();ros=RosNode('drone_semantic_sensor_bridge');gz=GzNode();sensors=Sensors()
    pubs={c:ros.create_publisher(String,'/drone/perception/'+c,10) for c in ['front','down']}
    def receive(m,camera):
        sensors.semantic(m,camera)
        now=time.monotonic();state=sensors.snapshot()
        objects={k:v for k,v in state['objects'].items() if v['camera']==camera and now-v['received']<.15}
        msg=String();msg.data=json.dumps({'schema_version':'1.0','frame':'map','source':'logical_camera_oracle','camera':camera,'objects':objects})
        pubs[camera].publish(msg)
    for camera in pubs:gz.subscribe(LogicalCameraImage,'/drone/semantic_'+camera,lambda m,c=camera:receive(m,c))
    try:rclpy.spin(ros)
    except KeyboardInterrupt:pass
    finally:ros.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
