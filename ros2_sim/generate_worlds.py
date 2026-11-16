#!/usr/bin/env python3
"""Self-contained AirVLA-inspired model; dimensions are configurable approximations."""
from pathlib import Path
import xml.etree.ElementTree as E
ROOT = Path(__file__).resolve().parent

def box(name, pos, size, color, static=True, mass=1):
    inertia = ' '.join([])
    x,y,z=size
    return f'''<model name="{name}"><static>{str(static).lower()}</static><pose>{' '.join(map(str,pos))} 0 0 0</pose><link name="link">
    <inertial><mass>{mass}</mass><inertia><ixx>{mass*(y*y+z*z)/12}</ixx><iyy>{mass*(x*x+z*z)/12}</iyy><izz>{mass*(x*x+y*y)/12}</izz></inertia></inertial>
    <collision name="collision"><geometry><box><size>{x} {y} {z}</size></box></geometry></collision>
    <visual name="visual"><geometry><box><size>{x} {y} {z}</size></box></geometry><material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
    </link></model>'''

def drone():
    s='''<model name="drone"><pose>0.85 0.65 0.43 0 0 0</pose><self_collide>false</self_collide>
    <link name="base_link"><inertial><mass>1.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.04</izz></inertia></inertial>
    <collision name="body"><geometry><box><size>0.28 0.22 0.12</size></box></geometry></collision>
    <visual name="body"><geometry><box><size>0.28 0.22 0.12</size></box></geometry><material><ambient>0.15 0.3 0.8 1</ambient><diffuse>0.15 0.3 0.8 1</diffuse></material></visual>
    <collision name="landing_leg_left"><pose>0 0.15 -0.2 0 0 0</pose><geometry><box><size>0.04 0.04 0.4</size></box></geometry></collision>
    <collision name="landing_leg_right"><pose>0 -0.15 -0.2 0 0 0</pose><geometry><box><size>0.04 0.04 0.4</size></box></geometry></collision>
    <visual name="gripper_left"><pose>0 -0.11 -0.23 0 0 0</pose><geometry><box><size>0.04 0.03 0.25</size></box></geometry></visual>
    <visual name="gripper_right"><pose>0 0.11 -0.23 0 0 0</pose><geometry><box><size>0.04 0.03 0.25</size></box></geometry></visual>
    <sensor name="imu" type="imu"><always_on>true</always_on><update_rate>100</update_rate><topic>/drone/imu</topic></sensor>
    <sensor name="contact" type="contact"><always_on>true</always_on><update_rate>30</update_rate><contact><collision>body</collision></contact><topic>/drone/contacts</topic></sensor>
    <sensor name="semantic_down" type="logical_camera"><pose>0 0 -0.08 0 1.57079632679 0</pose><always_on>true</always_on><update_rate>15</update_rate><topic>/drone/semantic_down</topic><logical_camera><near>0.05</near><far>5</far><horizontal_fov>1.8</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor>
    <sensor name="semantic_front" type="logical_camera"><always_on>true</always_on><update_rate>15</update_rate><topic>/drone/semantic_front</topic><logical_camera><near>0.05</near><far>6</far><horizontal_fov>1.8</horizontal_fov><aspect_ratio>1.5</aspect_ratio></logical_camera></sensor>
    <sensor name="forward_rgb" type="camera"><pose>0.16 0 0 0 0 0</pose><always_on>true</always_on><update_rate>5</update_rate><topic>/drone/camera/front/image</topic><camera><horizontal_fov>1.3</horizontal_fov><image><width>256</width><height>256</height><format>R8G8B8</format></image><clip><near>0.05</near><far>15</far></clip></camera></sensor>
    <sensor name="down_rgb" type="camera"><pose>0 0 -0.08 0 1.57079632679 0</pose><always_on>true</always_on><update_rate>5</update_rate><topic>/drone/camera/down/image</topic><camera><horizontal_fov>1.3</horizontal_fov><image><width>256</width><height>256</height><format>R8G8B8</format></image><clip><near>0.05</near><far>15</far></clip></camera></sensor>
    <sensor name="down_depth" type="depth_camera"><pose>0 0 -0.08 0 1.57079632679 0</pose><always_on>true</always_on><update_rate>5</update_rate><topic>/drone/camera/down/depth_image</topic><camera><horizontal_fov>1.3</horizontal_fov><image><width>256</width><height>256</height><format>R_FLOAT32</format></image><clip><near>0.05</near><far>10</far></clip></camera></sensor>
    </link>'''
    for i,(x,y,d) in enumerate([(0.18,-0.18,1),(-0.18,0.18,1),(0.18,0.18,-1),(-0.18,-0.18,-1)]):
        s+=f'''<link name="rotor_{i}"><pose>{x} {y} 0.08 0 0 0</pose><inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00002</izz></inertia></inertial><visual name="blade"><geometry><cylinder><radius>0.1</radius><length>0.01</length></cylinder></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual></link>
        <joint name="rotor_{i}_joint" type="revolute"><parent>base_link</parent><child>rotor_{i}</child><axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
        <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel"><robotNamespace>drone</robotNamespace><jointName>rotor_{i}_joint</jointName><linkName>rotor_{i}</linkName><turningDirection>{'ccw' if d==1 else 'cw'}</turningDirection><timeConstantUp>0.0125</timeConstantUp><timeConstantDown>0.025</timeConstantDown><maxRotVelocity>1100</maxRotVelocity><motorConstant>8.54858e-06</motorConstant><momentConstant>0.016</momentConstant><commandSubTopic>command/motor_speed</commandSubTopic><actuator_number>{i}</actuator_number><rotorDragCoefficient>8.06428e-05</rotorDragCoefficient><rollingMomentCoefficient>1e-06</rollingMomentCoefficient><rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim><motorType>velocity</motorType></plugin>'''
    s+='''<plugin filename="gz-sim-multicopter-control-system" name="gz::sim::systems::MulticopterVelocityControl"><robotNamespace>drone</robotNamespace><commandSubTopic>cmd_vel</commandSubTopic><enableSubTopic>enable</enableSubTopic><comLinkName>base_link</comLinkName><velocityGain>2.7 2.7 3.5</velocityGain><attitudeGain>2 3 0.15</attitudeGain><angularRateGain>0.4 0.52 0.18</angularRateGain><maximumLinearAcceleration>1 1 1</maximumLinearAcceleration><rotorConfiguration>'''
    for i,d in enumerate([1,1,-1,-1]):
        s+=f'<rotor><jointName>rotor_{i}_joint</jointName><forceConstant>8.54858e-06</forceConstant><momentConstant>0.016</momentConstant><direction>{d}</direction></rotor>'
    s+='''</rotorConfiguration></plugin>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>30</update_frequency></plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><dimensions>3</dimensions><odom_publish_frequency>30</odom_publish_frequency><odom_topic>/drone/odometry</odom_topic><odom_frame>map</odom_frame><robot_base_frame>base_link</robot_base_frame></plugin>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint"><parent_link>base_link</parent_link><child_model>red_package</child_model><child_link>link</child_link><initial_attach>false</initial_attach><attach_topic>/drone/gripper/attach</attach_topic><detach_topic>/drone/gripper/detach</detach_topic><output_topic>/drone/gripper/state</output_topic></plugin>
    </model>'''
    return s

def generate(blocked=False, cameras=True, scenario=None):
    scenario=scenario or (2 if blocked else 1)
    systems=['Physics','UserCommands','SceneBroadcaster','Imu','Contact','LogicalCamera']
    files=['physics','user-commands','scene-broadcaster','imu','contact','logical-camera']
    s='<sdf version="1.9"><world name="mission"><physics name="physics" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>'
    for name,filename in zip(systems,files):
        s+=f'<plugin filename="gz-sim-{filename}-system" name="gz::sim::systems::{name}">' + ('<engine><filename>libgz-physics7-dartsim-plugin.so.7</filename></engine>' if name=='Physics' else '') + '</plugin>'
    if cameras: s+='<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
    s+='''<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -1</direction></light>'''
    s+=box('ground',(4.5,2.6,-.05),(12,9,.1),'.75 .78 .8')
    s+=box('home',(.85,.65,.005),(.6,.6,.01),'.1 .1 .1')
    s+=box('obstacle_1',(3.6,2.35,1),(1.05,.72,2),'.4 .4 .4')
    s+=box('blue_platform',(7.6,4.15,.25),(1.05,.52,.5),'.05 .2 .9')
    if scenario==3:s+=box('red_platform',(7.6,2.65,.25),(1.05,.52,.5),'.9 .03 .03')
    s+=box('gate_left',(4.9,3.65,1.25),(.12,.16,2.5),'.5 .2 .7')
    s+=box('gate_right',(6.2,3.65,1.25),(.12,.16,2.5),'.5 .2 .7')
    s+=box('gate_top',(5.55,3.65,2.5),(1.42,.16,.12),'.5 .2 .7')
    if blocked: s+=box('gate_blocker',(5.55,3.65,1.1),(.9,.25,2.2),'.6 .3 .1')
    s+=box('red_package',(7.1,1.3,.12),(.18,.18,.24),'.9 .03 .03',False,.06)
    s+=drone()
    s+='''<model name="external_camera"><static>true</static><pose>4.5 -3 5 0 0.5 1.5707963</pose><link name="camera"><sensor name="external_rgb" type="camera"><always_on>true</always_on><update_rate>5</update_rate><topic>/drone/camera/external/image</topic><camera><horizontal_fov>1.5</horizontal_fov><image><width>256</width><height>256</height><format>R8G8B8</format></image><clip><near>0.1</near><far>30</far></clip></camera></sensor></link></model>'''
    s+='</world></sdf>'
    tree=E.ElementTree(E.fromstring(s)); E.indent(tree,space='  ')
    path=ROOT/'worlds'/f"scenario_{scenario}{'' if cameras else '_headless'}.sdf"
    tree.write(path,encoding='unicode',xml_declaration=True)
    return path

if __name__=='__main__':
    for scene in [1,2,3]:
        for cameras in [False,True]: print(generate(scene==2,cameras,scene))
