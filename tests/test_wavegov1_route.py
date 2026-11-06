"""Regression for the early-turn stall at the slalom/ramp transition."""
import json
import math
import unittest
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from cosmos_vln_protocol import load_route_catalog


class Wavegov1Route(unittest.TestCase):
    def test_catalog_passes_the_runtime_profile_mode_validation(self):
        routes = load_route_catalog(ROOT/'config/go2w_wavegov1_route.json')
        self.assertIn('wavegov1_original_course',routes)

    def test_fast_stair_pass_is_checked_between_global_replans(self):
        tree = ET.parse(ROOT/'config/go2w_continuous_nav.xml')
        pipeline = tree.find('.//PipelineSequence')
        self.assertEqual(pipeline[0].tag,'RemovePassedGoals')
        self.assertIsNone(tree.find('.//RateController//RemovePassedGoals'))

    def test_entry_is_not_skipped_at_previous_stall_position(self):
        stages = json.loads((ROOT/'config/go2w_wavegov1_route.json').read_text())['routes'][0]['stages']
        entry = next(s['goal'] for s in stages if s['stage_id'] == 'ramp_up_entry')
        radius = float(ET.parse(ROOT/'config/go2w_continuous_nav.xml').find('.//RemovePassedGoals').get('radius'))
        # Previous stop: (0.001, 12.572), turning too early towards the right.
        self.assertGreater(math.dist((entry['x'],entry['y']),(.001,12.572)),radius)
        # Pass the entry on the ramp before redirecting around its cylinder.
        self.assertLess(math.dist((entry['x'],entry['y']),(0.,13.16)),radius)
        self.assertTrue(all(a['goal']['y'] < b['goal']['y'] for a,b in zip(stages,stages[1:])))

    def test_route_terminates_after_original_ramp_without_added_ascent(self):
        stages = json.loads((ROOT/'config/go2w_wavegov1_route.json').read_text())['routes'][0]['stages']
        self.assertEqual(sum(s['profile']=='stair_up' for s in stages),1)
        self.assertEqual(sum(s['profile']=='stair_down' for s in stages),1)
        self.assertLess(stages[-1]['goal']['y'],20.)

    def test_descent_reference_clears_the_native_exit_barrel(self):
        stages = json.loads((ROOT/'config/go2w_wavegov1_route.json').read_text())['routes'][0]['stages']
        goals = {s['stage_id']:s['goal'] for s in stages}
        a, b = goals['ramp_down_entry'], goals['ramp_down_exit']
        scene = ET.parse(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_mid360_course.xml')
        barrel = scene.find('.//geom[@name="extended_native_barrel_1"]')
        x, y, _ = map(float,barrel.get('pos').split())
        dx, dy = b['x']-a['x'], b['y']-a['y']
        t = max(0.,min(1.,((x-a['x'])*dx+(y-a['y'])*dy)/(dx*dx+dy*dy)))
        center_distance = math.hypot(a['x']+t*dx-x,a['y']+t*dy-y)
        # 0.20 m barrel + 0.15 m half footprint + 0.25 m planning clearance.
        self.assertGreaterEqual(center_distance,.60)
