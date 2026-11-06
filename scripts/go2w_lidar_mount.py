"""Convert the UE sensor mount (cm, degrees) to the ROS body frame."""
import json,math
from pathlib import Path
def mount(sensor):
 p=sensor['position'];r=sensor['rotation']
 return [p['x']*.01,-p['y']*.01,p['z']*.01,math.radians(r['roll']),-math.radians(r['pitch']),-math.radians(r['yaw'])]

def body_points(points, sensor):
 """Apply the configured sensor mounting rotation and translation once.

 Raw LiDAR returns are in the sensor frame, including its downward pitch.
 Body/world pose alone cannot replace this fixed sensor extrinsic.
 """
 import numpy as np
 from scipy.spatial.transform import Rotation
 values=mount(sensor)
 rotation=Rotation.from_euler('xyz',values[3:]).as_matrix()
 return np.asarray(points)@rotation.T+values[:3]
if __name__=='__main__':
 root=Path(__file__).resolve().parents[1]
 print(' '.join(map(str,mount(json.loads((root/'matrix/config/config.json').read_text())['robot']['sensors']['lidar']))))
