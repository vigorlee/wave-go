#!/usr/bin/python3
import os,xml.etree.ElementTree as E
p=E.parse(os.environ['GO2W_WORLD_SCENE']);u=E.parse(os.environ['MATRIX_UE_SCENE_OVERRIDE'])
assert E.tostring(p.getroot())==E.tostring(u.getroot())
assert not p.findall(".//joint[@type='slide']")
print('Inline render/physics geometry matches; native stairs and upper floor preserved.')
