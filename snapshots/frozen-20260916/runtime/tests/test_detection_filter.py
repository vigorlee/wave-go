import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_detection_filter import DetectionFilter,classify

class DetectionFilterTests(unittest.TestCase):
 def test_ambiguous_labels_not_arbitrarily_assigned(self):
  self.assertEqual(classify('wall cylinder'),0)
  self.assertEqual(classify('stairs ramp'),7)
 def test_duplicate_and_low_confidence_suppressed(self):
  f=DetectionFilter();ids,a=f.select([[0,0,50,100],[2,2,49,99],[60,0,90,100]],[.8,.7,.3],['person','person','cylinder'],(200,200))
  self.assertEqual(ids,[0]);self.assertEqual([v['reason'] for v in a],[None,'duplicate_box','class_confidence'])
 def test_temporal_confirmation_expires_without_ghost_boxes(self):
  f=DetectionFilter(temporal=True);args=([[0,0,40,100]],[.8],['person'],(200,200))
  self.assertEqual(f.select(*args,now=0)[0],[])
  self.assertEqual(f.select(*args,now=.2)[0],[0])
  self.assertEqual(f.select([],[],[],(200,200),now=.4)[0],[])
  self.assertEqual(f.select(*args,now=.6)[0],[])
  self.assertEqual(f.select(*args,now=2)[0],[])
 def test_wall_sized_person_rejected(self):
  self.assertEqual(DetectionFilter().select([[0,0,190,180]],[.9],['person'],(200,200))[0],[])
