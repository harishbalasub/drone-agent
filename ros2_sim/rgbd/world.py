"""Derived dataset scenes. Existing mission worlds are not modified."""
from pathlib import Path
import math
import random
import xml.etree.ElementTree as E
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def sample_scene(seed):
    rng=random.Random(seed)
    return {'seed':seed,'target_present':rng.random()>.2,'package_position':[rng.uniform(5.8,7.6),rng.uniform(.8,2.1),.12],
            'package_yaw':rng.uniform(-math.pi,math.pi),'floor_color':[rng.uniform(.25,.85) for _ in range(3)],
            'light':rng.uniform(.45,1.),'split': 'train' if seed%20<14 else 'validation' if seed%20<17 else 'test'}


def write_world(scene,path):
    tree=E.parse(ROOT/'worlds/scenario_1.sdf');world=tree.getroot().find('world')
    world.attrib['name']='rgbd_dataset'
    for model in list(world.findall('model')):
        name=model.attrib['name']
        if name in {'drone','external_camera'}:world.remove(model)
        elif name=='red_package':
            if not scene['target_present']:world.remove(model);continue
            model.find('static').text='true';p=scene['package_position']
            model.find('pose').text=f'{p[0]} {p[1]} {p[2]} 0 0 {scene["package_yaw"]}'
            E.SubElement(E.SubElement(model,'plugin',filename='gz-sim-label-system',name='gz::sim::systems::Label'),'label').text='1'
        elif name=='ground':
            color=' '.join(map(str,scene['floor_color']))+' 1'
            for item in model.findall('.//material/ambient')+model.findall('.//material/diffuse'):item.text=color
    light=world.find('light/diffuse');light.text=' '.join([str(scene['light'])]*3)+' 1'
    rng=random.Random(scene['seed']+12345)
    for i in range(3):
        model=E.SubElement(world,'model',name='distractor_'+str(i));E.SubElement(model,'static').text='true'
        p=scene['package_position'];E.SubElement(model,'pose').text=f'{p[0]+rng.uniform(-1,1)} {p[1]+rng.uniform(-.7,.7)} .15 0 0 0'
        link=E.SubElement(model,'link',name='link');visual=E.SubElement(link,'visual',name='visual');geometry=E.SubElement(visual,'geometry')
        if i==0:E.SubElement(E.SubElement(geometry,'sphere'),'radius').text='.13';color='.8 .04 .04 1'
        else:E.SubElement(E.SubElement(geometry,'box'),'size').text='.15 .3 .3';color='.4 .25 .08 1' if i==1 else '.1 .5 .3 1'
        material=E.SubElement(visual,'material');E.SubElement(material,'ambient').text=color;E.SubElement(material,'diffuse').text=color
    # Static rig: camera-only sampling, not drone-flight training. Physics mission
    # evaluation remains a separate step after the perception model is trained.
    rig=E.SubElement(world,'model',name='rgbd_rig');E.SubElement(rig,'static').text='true';E.SubElement(rig,'pose').text='7.1 1.3 1.6 0 0 0'
    link=E.SubElement(rig,'link',name='camera_link')
    for name,kind,topic,fmt in [('rgb','camera','/dataset/rgb','R8G8B8'),('depth','depth_camera','/dataset/depth','R_FLOAT32'),('mask','segmentation','/dataset/mask','R8G8B8')]:
        sensor=E.SubElement(link,'sensor',name=name,type=kind);E.SubElement(sensor,'pose').text='0 0 0 0 1.5707963267948966 0'
        E.SubElement(sensor,'topic').text=topic;E.SubElement(sensor,'update_rate').text='5';E.SubElement(sensor,'always_on').text='true'
        camera=E.SubElement(sensor,'camera');E.SubElement(camera,'horizontal_fov').text='1.3'
        image=E.SubElement(camera,'image');E.SubElement(image,'width').text='256';E.SubElement(image,'height').text='256';E.SubElement(image,'format').text=fmt
        clip=E.SubElement(camera,'clip');E.SubElement(clip,'near').text='.05';E.SubElement(clip,'far').text='10'
        if kind=='segmentation':E.SubElement(camera,'segmentation_type').text='semantic'
    E.indent(tree,space='  ');tree.write(path,encoding='unicode',xml_declaration=True)
    return path


def camera_transform(position,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    # Body Rz(yaw) * downward camera Ry(pi/2).
    result=np.eye(4);result[:3,:3]=np.array([[0,-s,c],[0,c,s],[-1,0,0]]);result[:3,3]=position
    return result
