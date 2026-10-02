#!/usr/bin/python3
import os,xml.etree.ElementTree as ET
from pathlib import Path
p=Path(os.environ['GO2W_WORLD_SCENE']);u=Path(os.environ['MATRIX_UE_SCENE_OVERRIDE'])
a=ET.parse(p).getroot();b=ET.parse(u).getroot()
assert a.find('include').get('file')=='go2w.xml'
assert ET.tostring(a.find('worldbody'))==ET.tostring(b.find('worldbody')),'Office physics/UE scene mismatch'
assert not a.findall(".//joint[@type='slide']"),'Unexpected proxy joints'
assert len(a.findall('.//worldbody/geom'))>100,'Office geometry missing'
print('OfficeWorld physical/render scene validated; no moving cylinder proxy joints.')
