#!/usr/bin/env python3
"""Build one new clear-road physical integration fixture, never run Gazebo.

This is a four-method smoke scene, not one of the six-family performance
layouts or an independent geometry sample. The same world/map/guide is used
for every method. Frozen R2 algorithm does not consume the new common guide.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from static_map_export import make_artifacts
from common_route import build_bundle


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(output):
    output=Path(output)
    if output.exists():
        raise FileExistsError('preserve existing scene output')
    root=ET.Element('sdf',version='1.6')
    world=ET.SubElement(root,'world',name='astra_navigation_n01_clear')
    def text(node,key,value):
        ET.SubElement(node,key).text=str(value)
    text(world,'gravity','0 0 -9.81')
    physics=ET.SubElement(world,'physics',name='physics',type='ode')
    text(physics,'max_step_size','.002');text(physics,'real_time_update_rate',500)
    light=ET.SubElement(world,'light',name='sun',type='directional')
    text(light,'pose','0 0 10 0 0 0');text(light,'diffuse','.9 .9 .9 1');text(light,'direction','-.5 .1 -.9')
    def solid(name,pose,kind,dimensions,color):
        model=ET.SubElement(world,'model',name=name);text(model,'static','true');text(model,'pose',pose)
        link=ET.SubElement(model,'link',name='body')
        for category in ('collision','visual'):
            item=ET.SubElement(link,category,name='shape')
            primitive=ET.SubElement(ET.SubElement(item,'geometry'),kind)
            for key,value in dimensions.items():text(primitive,key,value)
            if category=='visual':
                material=ET.SubElement(item,'material');text(material,'ambient',color);text(material,'diffuse',color)
    solid('ground_plane','0 0 0 0 0 0','plane',{'normal':'0 0 1','size':'40 30'},'.75 .78 .8 1')
    solid('roadside_box','1.4 2.8 .6 0 0 .6','box',{'size':'1.5 .5 1.2'},'.25 .45 .7 1')
    solid('roadside_cylinder','1.2 -2.6 .6 0 0 0','cylinder',{'radius':'.45','length':'1.2'},'.7 .4 .2 1')
    model=ET.SubElement(world,'model',name='astra_n01_camera');text(model,'static','true')
    text(model,'pose','0 0 19 0 1.5707963267948966 1.5707963267948966')
    link=ET.SubElement(model,'link',name='body');sensor=ET.SubElement(link,'sensor',name='overview',type='camera')
    text(sensor,'always_on','true');text(sensor,'update_rate',2)
    camera=ET.SubElement(sensor,'camera');text(camera,'horizontal_fov',1.05)
    image=ET.SubElement(camera,'image')
    for key,value in (('width',2560),('height',1440),('format','R8G8B8')):text(image,key,value)
    clip=ET.SubElement(camera,'clip');text(clip,'near','.1');text(clip,'far',100)
    plugin=ET.SubElement(sensor,'plugin',name='n01_camera',filename='libgazebo_ros_camera.so')
    for key,value in {'robotNamespace':'/astra_n01','cameraName':'camera','imageTopicName':'image_raw','cameraInfoTopicName':'camera_info','frameName':'astra_n01_camera_optical','updateRate':2,'alwaysOn':'true'}.items():text(plugin,key,value)
    dense=ET.SubElement(world,'plugin',name='whole_contact_evidence',filename='libastra_dense_evidence_plugin.so');text(dense,'state_hz',50)
    ET.SubElement(world,'plugin',name='passive_sweep_evidence',filename='libastra_physics_metadata_p03_plugin.so')
    xml=ET.tostring(root,encoding='unicode')+'\n'
    scene={'schema':'astra_navigation_n01_clear_scene_v1','scene_id':'navigation_n01_clear','split':'bootstrap_smoke',
        'parent_geometry_id':'new_clear_road_integration_fixture','actors':[],
        'route':{'frame':'world','spawn':[-5.4,0.,0.],'start':[-4.,0.,0.],'goal':[5.,0.,0.]},
        'robot_collision_reference':{'spawn_xyz_rpy':[-5.4,0.,0.,0.,0.,0.],'reference':'base_link'},
        'static_map_bounds_xyxy':[-20.,-15.,20.,15.],
        'camera':{'topic':'/astra_n01/camera/image_raw','width':2560,'height':1440,'rate_hz':2},
        'performance_evidence':False,'independent_layout_count_for_core_population':0,
        'frozen_R2_common_guide_consumption':False,'all_methods_same_shared_static_prior':True}
    output.mkdir(parents=True);(output/'world.sdf').write_text(xml)
    (output/'scene_manifest.yaml').write_text(yaml.safe_dump(scene,sort_keys=False))
    exported=output/'map_export';exported.mkdir()
    pgm,metadata,request,audit=make_artifacts(xml,scene)
    (exported/'map.pgm').write_bytes(pgm);(exported/'map.yaml').write_bytes(metadata)
    (exported/'route_request.json').write_text(json.dumps(request,indent=2)+'\n')
    (exported/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    route=build_bundle(exported,output/'common_bundle',radius=1.)
    assert abs(route['route_length_m']-9.)<1e-9
    paths=[output/'world.sdf',output/'scene_manifest.yaml']+sorted(exported.iterdir())+sorted((output/'common_bundle').iterdir())
    manifest={'status':'offline_smoke_geometry_only','execution_authorized':False,'seed_allocated':False,
        'generator_sha256':digest(__file__),'route_length_m':route['route_length_m'],
        'files':[{'path':str(p.resolve()),'sha256':digest(p)} for p in paths],
        'live_camera_and_physical_route_validation':'pending'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=build(args.output)
    print(json.dumps({'status':result['status'],'route_length_m':result['route_length_m']}))
