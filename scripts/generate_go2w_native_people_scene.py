#!/usr/bin/python3
"""Keep UE state indices while suppressing obsolete non-colliding proxy meshes."""
import xml.etree.ElementTree as ET
from sync_go2w_extended_ue_scene import PHYSICS,UE,ue_scene_root
root=ue_scene_root(ET.parse(PHYSICS).getroot())
mesh=root.find(".//asset/mesh[@name='PEDESTRIAN_OBS_LINK']")
if mesh is None:raise RuntimeError('proxy mesh missing')
# Removing geoms shifts the packaged UE protocol indices. Tiny render meshes
# preserve those indices; no proxy points are injected into navigation.
mesh.set('scale','0.001 0.001 0.001')
ET.indent(root)
out=UE.with_name('scene_terrain_yard_native_people.xml')
out.write_text(ET.tostring(root,encoding='unicode')+'\n');print(out)
