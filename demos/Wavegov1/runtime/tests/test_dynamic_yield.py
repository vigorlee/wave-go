import math
import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from go2w_dynamic_yield import CrossingYield
SCENE=ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml'

class CrossingYieldTests(unittest.TestCase):
    def test_crossings_wait_and_complete_over_motion_phases(self):
        # Include braking lag, both corridor offsets, and the transverse gallery.
        for target,offset,amplitude,plane in [('pedestrian_ramp_exit',.85,2.5,20.8),('pedestrian_north_corridor',.4,2.2,25.8),('pedestrian_gallery_crossing',32.5,1.75,10.25)]:
            for phase in range(12):
                guard=CrossingYield(SCENE);lateral,normal,center,omega=guard.crossings[target]
                robot=[0.,0.];robot[lateral]=offset;robot[normal]=plane-1.9;v=0.;nearest=100.
                for step in range(4000):
                    t=step*.02
                    positions=[]
                    for name,(lat,norm,c,w) in guard.crossings.items():
                        p=[100.,100.,.87]
                        if name==target:p[norm]=plane;p[lat]=c+amplitude*math.cos(w*t+phase*math.pi/6)
                        positions.append((name,tuple(p)))
                    wait,_=guard.evaluate(t,robot,positions)
                    desired=0. if wait else .20;v+=max(-.8*.02,min(.8*.02,desired-v));robot[normal]+=v*.02
                    p=dict(positions)[target];nearest=min(nearest,math.hypot(robot[0]-p[0],robot[1]-p[1]))
                    if robot[normal]>plane+1.2:break
                self.assertLess(step,3999,(target,phase,'did not clear'))
                self.assertGreaterEqual(nearest,.95,(target,phase,nearest))

    def test_missing_velocity_waits(self):
        guard=CrossingYield(SCENE)
        self.assertTrue(guard.evaluate(0.,[0.,0.],[])[0])

if __name__=='__main__':unittest.main()
