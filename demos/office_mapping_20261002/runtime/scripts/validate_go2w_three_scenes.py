#!/usr/bin/python3
import os,xml.etree.ElementTree as ET
from pathlib import Path
p=ET.parse(os.environ['GO2W_WORLD_SCENE']).getroot();u=ET.parse(os.environ['MATRIX_UE_SCENE_OVERRIDE']).getroot()
for name in ['box_OBS15','box_OBS16']:
 g=u.find(f"./worldbody/geom[@name='{name}']");a=list(map(float,g.get('euler').split()));g.set('euler',f'{-a[0]} 0 0')
for a,b in zip(p.findall('./worldbody/geom'),u.findall('./worldbody/geom')):
 assert a.attrib==b.attrib,(a.get('name'),a.attrib,b.attrib)
assert len(p.findall('./worldbody/geom'))==len(u.findall('./worldbody/geom'))
assert not p.findall(".//joint[@type='slide']")
print('Original three-scene course: render/physics geometry matches after measured ramp roll compensation.')
