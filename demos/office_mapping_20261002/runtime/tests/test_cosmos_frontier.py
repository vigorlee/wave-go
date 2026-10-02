import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2w_cosmos_frontier import validate_choice

class CosmosChoiceTests(unittest.TestCase):
    def test_only_measured_candidate_ids_can_move(self):
        self.assertEqual(validate_choice('{"action":"navigate","candidate_id":1,"confidence":0.9}',2)['candidate_id'],1)
        for item in ('-1','2','true','"1"'):
            with self.assertRaises(ValueError):
                validate_choice('{"action":"navigate","candidate_id":'+item+',"confidence":0.9}',2)
    def test_low_confidence_and_nan_fail_closed(self):
        for confidence in ('0.59','NaN','1.01','null'):
            with self.assertRaises(ValueError):
                validate_choice('{"action":"navigate","candidate_id":0,"confidence":'+confidence+'}',1)
    def test_hold_never_becomes_navigation(self):
        self.assertEqual(validate_choice('{"action":"hold","reason":"person blocks exit"}',2)['action'],'hold')
    def test_missing_json_is_rejected(self):
        with self.assertRaises(ValueError):validate_choice('go forward 10 metres',2)

    def test_rejects_multiple_or_truncated_decisions(self):
        for text in ('[{"action":"navigate","candidate_id":0,"confidence":0.8},{"action":"hold"}]', '{"action":"navigate","candidate_id":0,"confidence":0.8} trailing', '<think>{"action":"navigate","candidate_id":0,"confidence":0.8}</think>unfinished'):
            with self.assertRaises(ValueError):validate_choice(text,2)
    def test_final_answer_overrides_reasoning(self):
        self.assertEqual(validate_choice('thinking {"action":"navigate","candidate_id":0,"confidence":0.8}</think>```json\n{"action":"hold"}\n```',2)['action'],'hold')
