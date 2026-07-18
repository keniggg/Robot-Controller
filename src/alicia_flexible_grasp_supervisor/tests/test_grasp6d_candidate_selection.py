#!/usr/bin/env python3
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.grasp6d_candidate_selector import Grasp6DCandidate, select_best_grasp6d_candidate
from alicia_flexible_grasp.grasp.gripper_geometry import (
    CandidateGateResult,
    candidate_rank_key,
)


class Grasp6DCandidateSelectionTest(unittest.TestCase):
    def test_selector_rejects_colliding_and_unreachable_candidates(self):
        candidates = [
            Grasp6DCandidate(score=0.95, collision_free=False, reachable=True, tactile_score=1.0),
            Grasp6DCandidate(score=0.90, collision_free=True, reachable=False, tactile_score=1.0),
            Grasp6DCandidate(score=0.70, collision_free=True, reachable=True, tactile_score=1.0),
        ]

        selected = select_best_grasp6d_candidate(candidates)

        self.assertIs(selected, candidates[2])

    def test_selector_combines_model_score_and_tactile_safety(self):
        candidates = [
            Grasp6DCandidate(score=0.80, collision_free=True, reachable=True, tactile_score=0.2),
            Grasp6DCandidate(score=0.70, collision_free=True, reachable=True, tactile_score=1.0),
        ]

        selected = select_best_grasp6d_candidate(candidates, tactile_weight=0.35)

        self.assertIs(selected, candidates[1])

    def test_analytical_rank_places_motion_before_model_score(self):
        slow_high_score = CandidateGateResult(
            True, '', '', 0.044, 0.002, 0.010, 1.0, 2.0, 0.002, '', 6
        )
        fast_low_score = CandidateGateResult(
            True, '', '', 0.044, 0.002, 0.010, 1.0, 1.0, 0.002, '', 6
        )

        self.assertLess(
            candidate_rank_key(fast_low_score, 0.10),
            candidate_rank_key(slow_high_score, 0.99),
        )

    def test_failed_geometry_cannot_be_ranked_back_by_high_model_score(self):
        failed = CandidateGateResult(
            False,
            'GRIPPER_TOO_NARROW',
            'required opening exceeds 50 mm',
            0.060,
            0.0,
            0.010,
            1.0,
            0.0,
            0.0,
            'jaw_width',
            2,
        )

        with self.assertRaises(ValueError):
            candidate_rank_key(failed, 1.0)


if __name__ == '__main__':
    unittest.main()
