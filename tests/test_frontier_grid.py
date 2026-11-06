import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_frontier_grid import FrontierGrid


class SensorMappingTests(unittest.TestCase):
    def test_starts_unknown_without_map_or_route(self):
        g=FrontierGrid(100,.1)
        self.assertTrue((g.occupancy()==-1).all())
        self.assertEqual(g.frontiers(np.zeros(2)),[])

    def test_ray_does_not_clear_behind_wall(self):
        g=FrontierGrid(100,.1);g.integrate(np.array([[2.,0.,.5]]),[0.,0.,.5],0.)
        a=g.occupancy();before=g.cells([1.,0.]);wall=g.cells([2.,0.]);behind=g.cells([3.,0.])
        self.assertEqual(a[before[1],before[0]],0)
        self.assertEqual(a[wall[1],wall[0]],100)
        self.assertEqual(a[behind[1],behind[0]],-1)

    def test_high_ceiling_return_does_not_invent_free_ground(self):
        g=FrontierGrid(100,.1);g.integrate(np.array([[2.,0.,5.]]),[0.,0.,.5],0.)
        self.assertFalse(g.seen.any())

    def test_frontier_goals_stay_in_measured_free_component(self):
        g=FrontierGrid(100,.1)
        g.seen[30:70,30:70]=True;g.odds[30:70,30:70]=-1
        # Separate observed island is unreachable from the robot.
        g.seen[5:20,5:20]=True;g.odds[5:20,5:20]=-1
        goals=g.frontiers(np.zeros(2));self.assertTrue(goals)
        for goal in goals:
            cell=g.cells([goal['x'],goal['y']])
            self.assertTrue(30<=cell[0]<70 and 30<=cell[1]<70)
            self.assertEqual(g.occupancy()[cell[1],cell[0]],0)

    def test_farthest_mode_targets_remote_end_of_connected_corridor(self):
        g=FrontierGrid(240,.1)
        g.seen[100:140,100:215]=True;g.odds[100:140,100:215]=-1
        goals=g.frontiers(np.array([3.,0.]),far_origin=np.zeros(2))
        self.assertTrue(goals)
        self.assertGreater(goals[0]['x'],8.)
        cell=g.cells([goals[0]['x'],goals[0]['y']])
        self.assertEqual(g.occupancy()[cell[1],cell[0]],0)

    def test_low_curb_is_an_obstacle_not_free_floor(self):
        g=FrontierGrid(100,.1);g.integrate(np.array([[2.,0.,.15]]),[0.,0.,.55],0.)
        wall=g.cells([2.,0.]);self.assertEqual(g.occupancy()[wall[1],wall[0]],100)

    def test_overhead_ray_cannot_erase_measured_low_curb(self):
        g=FrontierGrid(100,.1);g.integrate(np.array([[2.,0.,.15]]),[0.,0.,.55],0.)
        for _ in range(20):g.integrate(np.array([[4.,0.,.8]]),[0.,0.,.55],0.)
        cell=g.cells([2.,0.]);self.assertEqual(g.occupancy()[cell[1],cell[0]],100)
        for _ in range(20):g.integrate(np.array([[2.,0.,0.]]),[0.,0.,.55],0.)
        self.assertEqual(g.occupancy()[cell[1],cell[0]],0)

    def test_cosmos_can_select_frontier_beyond_old_eight_metre_limit(self):
        g=FrontierGrid(400,.1)
        g.seen[90:310,90:310]=True;g.odds[90:310,90:310]=-1
        self.assertEqual(g.frontiers(np.zeros(2)),[])
        candidates=g.frontiers(np.zeros(2),far_origin=np.zeros(2))
        self.assertTrue(candidates)
        self.assertGreater(np.hypot(candidates[0]['x'],candidates[0]['y']),8.)

    def test_farthest_goal_does_not_require_unknown_boundary(self):
        g=FrontierGrid(400,.1);g.seen[:]=True;g.odds[:]=3
        g.odds[190:210,170:360]=-2
        candidates=g.frontiers(np.zeros(2),far_origin=np.zeros(2))
        self.assertTrue(candidates)
        self.assertGreater(candidates[0]['x'],15.)
        self.assertEqual(candidates[0]['source'],'farthest_observed_free_space')
    def test_farthest_mode_does_not_send_robot_back_toward_start(self):
        g=FrontierGrid(400,.1);g.seen[:]=True;g.odds[:]=3
        g.odds[195:205,200:350]=-2
        self.assertEqual(g.frontiers(np.array([14.5,0.]),far_origin=np.zeros(2)),[])

    def test_initial_forward_direction_excludes_farthest_rear_wall(self):
        g=FrontierGrid(400,.1);g.seen[:]=True;g.odds[:]=3
        g.odds[190:210,40:290]=-2
        candidates=g.frontiers(np.zeros(2),far_origin=np.zeros(2),far_direction=np.array([1.,0.]))
        self.assertTrue(candidates)
        self.assertTrue(all(c['x']>0 for c in candidates))
        self.assertGreater(candidates[0]['x'],8.)
