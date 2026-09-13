import json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from cosmos_vln_protocol import load_route_catalog

class FullCornerRouteTests(unittest.TestCase):
    def test_original_terrain_sequence_is_preserved(self):
        old=json.loads((ROOT/'config/go2w_wavegov1_route.json').read_text())['routes'][0]['stages']
        new=json.loads((ROOT/'config/go2w_all_obstacles_corner.json').read_text())['routes'][0]['stages']
        self.assertEqual(new[:len(old)],old)
        self.assertGreater(len(new),len(old))
    def test_endpoint_requires_north_gallery_and_east_corridor(self):
        route=load_route_catalog(ROOT/'config/go2w_all_obstacles_corner.json')['all_obstacles_to_east_corner']
        stages=route.stages
        self.assertEqual((stages[-1].x,stages[-1].y),(18.,22.8))
        self.assertTrue(any(s.y>32 for s in stages))
        self.assertTrue(any(s.x>17 and s.y>32 for s in stages))
