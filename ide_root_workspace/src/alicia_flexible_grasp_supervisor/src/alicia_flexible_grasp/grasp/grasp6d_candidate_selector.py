from dataclasses import dataclass
from typing import Any, Optional, Sequence


@dataclass
class Grasp6DCandidate:
    score: float
    collision_free: bool = True
    reachable: bool = True
    tactile_score: float = 1.0
    pose_camera: Any = None
    pose_base: Any = None
    width_m: Optional[float] = None
    source: str = 'alicia_d_grasp_6d'


def select_best_grasp6d_candidate(candidates: Sequence[Grasp6DCandidate], tactile_weight=0.2):
    valid = [candidate for candidate in candidates if candidate.collision_free and candidate.reachable]
    if not valid:
        return None
    return max(valid, key=lambda candidate: _combined_score(candidate, tactile_weight))


def _combined_score(candidate, tactile_weight):
    tactile_weight = min(1.0, max(0.0, float(tactile_weight)))
    model_weight = 1.0 - tactile_weight
    tactile_score = min(1.0, max(0.0, float(candidate.tactile_score)))
    return model_weight * float(candidate.score) + tactile_weight * tactile_score
