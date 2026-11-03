"""Fixed ORB proposal generator for the offline appearance candidate.

Descriptor supports may extend outside the eroded keypoint mask. Consequently
neither matches nor inlier counts establish identity: only the separately
bounded, masked dense evaluator may corroborate this one fixed proposal.
"""
from dataclasses import dataclass
import time
from typing import Optional, Tuple

import cv2
import numpy as np
from alicia_grasp_modes.arrival_dense_identity import (
    FrozenReference, HomographyProposal, _inputs, _source, propose_homography)


@dataclass(frozen=True)
class ProposalEvidence:
    code: str
    reference_digest: str
    reference_source_stamp_ns: int
    current_source_stamp_ns: int
    reference_keypoints: int = 0
    current_keypoints: int = 0
    mutual_ratio_matches: int = 0
    ransac_inliers: int = 0
    appearance_match: bool = False
    motion_authorized: bool = False
    metric_geometry_authority: bool = False


class _Expired(Exception):
    pass


def propose_orb_homography(reference: FrozenReference, image, mask, rgb_valid,
                            source_stamp_ns: int, *, deadline_monotonic: float,
                            clock=time.monotonic) -> Tuple[Optional[HomographyProposal], ProposalEvidence]:
    """One fixed ORB/H estimate, source-bound, with no fitting to dense scores.

    The absolute deadline is supplied by the already authorized caller. This
    function never renews it. Arrays must correspond to actual input sources;
    their authenticity remains the caller's responsibility.
    """
    source = 0
    counts = dict(reference_keypoints=0, current_keypoints=0, mutual_ratio_matches=0, ransac_inliers=0)

    def check():
        now = clock()
        if (isinstance(deadline_monotonic, (bool, np.bool_))
                or not isinstance(deadline_monotonic, (int, float, np.number))
                or not np.isfinite(deadline_monotonic) or not np.isfinite(now)
                or now >= deadline_monotonic):
            raise _Expired()

    def result(proposal, code):
        try:
            check()
        except _Expired:
            proposal, code = None, 'DEADLINE_EXCEEDED'
        return proposal, ProposalEvidence(code, reference.digest, reference.source_stamp_ns, source, **counts)

    try:
        check()
        source = _source(source_stamp_ns)
        if source <= reference.source_stamp_ns:
            return result(None, 'CURRENT_SOURCE_NOT_AFTER_REFERENCE')
        image, mask, valid = _inputs(image, mask, rgb_valid)
        if mask.shape != reference.mask.shape:
            return result(None, 'CAMERA_SHAPE_CHANGED')
        orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8,
                            edgeThreshold=15, firstLevel=0, WTA_K=2,
                            scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=20)
        kernel = np.ones((7, 7), np.uint8)

        def detect(bgr, actual_mask):
            inner = cv2.erode((actual_mask > 0).astype(np.uint8)*255, kernel)
            return orb.detectAndCompute(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), inner)

        kr, dr = detect(reference.bgr, reference.mask)
        counts['reference_keypoints'] = len(kr)
        check()
        kc, dc = detect(image, mask)
        counts['current_keypoints'] = len(kc)
        check()
        if dr is None or dc is None or len(dr) < 2 or len(dc) < 2:
            return result(None, 'NO_DESCRIPTOR_PROPOSAL')
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        forward = {pair[0].queryIdx: pair[0] for pair in matcher.knnMatch(dr, dc, k=2)
                   if len(pair) == 2 and pair[0].distance < .75*pair[1].distance}
        check()
        reverse = {pair[0].queryIdx: pair[0] for pair in matcher.knnMatch(dc, dr, k=2)
                   if len(pair) == 2 and pair[0].distance < .75*pair[1].distance}
        matches = [match for i, match in forward.items()
                   if match.trainIdx in reverse and reverse[match.trainIdx].trainIdx == i]
        counts['mutual_ratio_matches'] = len(matches)
        check()
        # Four is only the algebraic minimum to propose a homography, never an
        # appearance acceptance count. No fallback to another estimator follows.
        if len(matches) < 4:
            return result(None, 'INSUFFICIENT_MATCHES_TO_PROPOSE_H')
        src = np.asarray([kr[match.queryIdx].pt for match in matches], np.float32)
        dst = np.asarray([kc[match.trainIdx].pt for match in matches], np.float32)
        cv2.setRNGSeed(1729)
        H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 3., maxIters=2000, confidence=.995)
        check()
        if H is None or inliers is None or not np.all(np.isfinite(H)):
            return result(None, 'NO_FINITE_H_PROPOSAL')
        counts['ransac_inliers'] = int(np.count_nonzero(inliers))
        return result(propose_homography(reference, source, H), 'H_PROPOSED_ONLY')
    except _Expired:
        return None, ProposalEvidence('DEADLINE_EXCEEDED', reference.digest, reference.source_stamp_ns, source, **counts)
    except (ValueError, cv2.error, OverflowError):
        return result(None, 'INVALID_PROPOSAL_INPUT')
