"""Persist the first loss observation without changing target/geometry state."""
from copy import deepcopy
import numpy as np


def first_loss_observation(previous, *, reason, generation, selection_stamp_ns,
                           observed_stamp_ns, source_stamp_ns, anchor_source_stamp_ns,
                           anchor_position_base, anchor_extent_m,
                           reference_support_plane_base, last_good_source_stamp_ns,
                           last_consumed_source_stamp_ns, candidate_evidence=None):
    """Return a frozen diagnostic record; repeated generic loss cannot overwrite it.

    source_stamp_ns stays None when no source pair caused the failure (for
    example an exact-stream timeout). Wall/ROS observation time is a separate
    field and must never be presented as the missing image source timestamp.
    Explicit selection handling owns resetting ``previous`` to None.
    """
    if previous is not None:
        return deepcopy(previous)
    return dict(reason=str(reason), generation=int(generation),
                selection_stamp_ns=int(selection_stamp_ns),
                observed_stamp_ns=int(observed_stamp_ns),
                source_stamp_ns=None if source_stamp_ns is None else int(source_stamp_ns),
                source_pair_available=source_stamp_ns is not None,
                anchor_source_stamp_ns=int(anchor_source_stamp_ns),
                anchor_position_base=list(anchor_position_base),
                anchor_extent_m=list(anchor_extent_m),
                reference_support_plane_base=list(reference_support_plane_base),
                last_good_source_stamp_ns=int(last_good_source_stamp_ns),
                last_consumed_source_stamp_ns=int(last_consumed_source_stamp_ns),
                candidate_evidence=deepcopy(candidate_evidence),
                diagnostic_only=True, target_recovered=False)


def candidate_loss_observation(result, anchor, last, config, was_lost):
    """Describe original association gates without choosing or mutating state."""
    rows=[]
    for candidate in result.candidates:
        distance=float(np.linalg.norm(candidate.position_base-anchor.position_base))
        step=float(np.linalg.norm(candidate.position_base-last.position_base))
        ratios=candidate.extent_m[:2]/np.maximum(anchor.extent_m[:2],.003)
        eligible=bool(distance<=config.association_distance_m and
                      step<=config.association_distance_m and
                      np.all((ratios>=.5)&(ratios<=2.)))
        rows.append(dict(anchor_distance_m=distance,last_distance_m=step,
                         extent_ratios_xy=ratios.tolist(),eligible=eligible,
                         position_base=candidate.position_base.tolist(),
                         extent_m=candidate.extent_m.tolist(),quality=float(candidate.quality)))
    return dict(tracker_lost_before_choose=bool(was_lost),candidate_count=len(rows),
                eligible_count=sum(row['eligible'] for row in rows),
                association_distance_m=float(config.association_distance_m),
                extent_ratio_limits=[.5,2.],candidates=rows[:32],
                candidate_rows_truncated=max(0,len(rows)-32),
                segmentation_metrics=deepcopy(result.metrics),
                last_position_base=last.position_base.tolist())
