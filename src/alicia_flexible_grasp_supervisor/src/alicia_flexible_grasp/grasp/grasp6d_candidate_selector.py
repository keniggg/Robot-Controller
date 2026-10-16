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
    ranked = rank_grasp6d_candidates_for_strict_check(
        candidates, tactile_weight=tactile_weight
    )
    return next(
        (candidate for candidate in ranked if candidate.reachable is True),
        None,
    )


def rank_grasp6d_candidates_for_strict_check(candidates, tactile_weight=0.2):
    """Soft-rank collision-free candidates before strict reachability.

    ``reachable`` is deliberately ignored here: the expensive strict checker
    belongs after stable-candidate ranking and the bounded Top-N slice.
    """

    eligible = [
        (index, candidate)
        for index, candidate in enumerate(candidates)
        if candidate.collision_free is True
    ]
    eligible.sort(
        key=lambda item: (
            -_combined_score(item[1], tactile_weight),
            item[0],
        )
    )
    return tuple(candidate for _index, candidate in eligible)


def _combined_score(candidate, tactile_weight):
    tactile_weight = min(1.0, max(0.0, float(tactile_weight)))
    model_weight = 1.0 - tactile_weight
    tactile_score = min(1.0, max(0.0, float(candidate.tactile_score)))
    return model_weight * float(candidate.score) + tactile_weight * tactile_score
