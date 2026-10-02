"""Check geometry exported by the ACTUAL UE scene loader, including Office offset."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
def validate():
 manifest=json.loads((ROOT/'config/go2w_office_inline_manifest.json').read_text())
 physics=ET.parse(manifest['physics'])
 actual=ET.parse(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_custom.xml')
 expected=[physics.find(f"./worldbody/geom[@name='{name}']") for name in manifest['added_geoms']]
 expected=[g for g in expected if g.get('contype')!='0']
 geoms=[g for g in actual.findall('./worldbody/geom') if g.get('name','').startswith('BP_Scene_')]
 assert len(geoms)==len(expected)==60,(len(geoms),len(expected))
 maximum=0.
 for source,spawned in zip(expected,geoms):
  p=np.fromstring(spawned.get('pos'),sep=' ')+[7.6,24.2,0];q=np.fromstring(source.get('pos'),sep=' ')
  maximum=max(maximum,float(np.max(np.abs(p-q))))
  assert np.allclose(p,q,atol=2e-5),source.get('name')
  assert np.allclose(np.fromstring(source.get('size'),sep=' '),np.fromstring(spawned.get('size'),sep=' '),atol=2e-5)
  e=np.fromstring(source.get('euler','0 0 0'),sep=' ');quat=Rotation.from_euler('xyz',e).as_quat()[[3,0,1,2]]
  assert np.allclose(quat,np.fromstring(spawned.get('quat'),sep=' '),atol=2e-5),source.get('name')
 report=dict(verified_ue_actor_count=len(geoms),maximum_position_error_m=maximum,ue_world_offset_cm=[-760,2420,0],sizes_preserved=True,rotations_match_physics=True)
 print(json.dumps(report));return report
if __name__=='__main__':validate()
