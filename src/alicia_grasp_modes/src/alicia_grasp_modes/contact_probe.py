"""Request-local contact tilt probes using the unchanged Alicia CAD geometry."""
from types import SimpleNamespace

import numpy as np

from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopCandidateContractError, _open_finger_corners,
    materialize_tabletop_candidates, semantic_axes_to_tool_rotation,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M as PAIR,
    ANALYTICAL_PALM_CENTER_TOOL_XYZ_M as PALM,
)


def contact_tilt_probe(proposal, support_point, support_normal, gripper,
                       tool_jaw_axis, tool_finger_length_axis):
    # Alternative tool conventions retain the general materializer.
    if tool_jaw_axis != 'y' or tool_finger_length_axis != 'z':
        return None
    # Establish the same validated inputs and zero-tilt wrist branches once.
    materialize_tabletop_candidates(
        proposal, support_point, support_normal, gripper,
        tool_jaw_axis, tool_finger_length_axis)
    corners_local = _open_finger_corners(np.eye(4), gripper,
                                         tool_jaw_axis, tool_finger_length_axis)
    normal = np.array(support_normal, dtype=float, copy=True)
    point = np.array(support_point, dtype=float, copy=True)
    center = np.array(proposal.contact_center_base, dtype=float, copy=True)
    contact_height = float(np.dot(center-point, normal))
    clearance = float(gripper.support_clearance_m)

    jaw_axis = np.array(proposal.jaw_axis_base, dtype=float, copy=True)
    insertion_axis = np.array(proposal.insertion_axis_base, dtype=float, copy=True)
    tilt_axis = np.cross(jaw_axis, normal)
    tilt_axis /= np.linalg.norm(tilt_axis)

    def probe(angle_deg, branch):
        if (not isinstance(branch, tuple) or len(branch) != 2
                or type(branch[0]) is not int or branch[0] not in (0, 1)
                or type(branch[1]) not in (float, int) or branch[1] not in (-1., 1.)):
            raise TabletopCandidateContractError(
                'TABLETOP_APPROACH_INVALID', 'invalid contact tilt probe branch')
        angle = float(angle_deg)
        jaw, polarity = branch
        if (isinstance(angle_deg, bool)
                or not np.isfinite(angle) or not 0 < angle <= 45):
            raise TabletopCandidateContractError('TABLETOP_APPROACH_INVALID',
                                                 'invalid contact tilt probe branch')
        radians = np.deg2rad(angle)
        c, s = float(np.cos(radians)), float(polarity*np.sin(radians))
        jaw_sign = 1. if jaw == 0 else -1.
        tilted = c * insertion_axis + s * jaw_sign * tilt_axis
        tilted /= np.linalg.norm(tilted)
        rotation = semantic_axes_to_tool_rotation(
            tilted, jaw_sign * jaw_axis, tool_jaw_axis, tool_finger_length_axis)
        delta = center - rotation @ PAIR
        translation = delta - normal*float(np.dot(delta, normal))
        corners = corners_local @ rotation.T + translation
        translation = translation + (clearance-float(np.min((corners-point)@normal)))*normal
        corners = corners_local @ rotation.T + translation
        heights = (corners-point) @ normal
        pair_center = translation + rotation @ PAIR
        lateral = pair_center-center
        lateral -= float(np.dot(lateral,normal))*normal
        palm_center = translation + rotation @ PALM
        insertion = rotation[:, 2]
        if (abs(float(np.min(heights))-clearance) > 1e-8
                or not float(np.min(heights))-1e-9 <= contact_height <= float(np.max(heights))+1e-9
                or float(np.linalg.norm(lateral)) > 1e-8
                or float(np.dot(palm_center-pair_center,insertion)) >= 0):
            raise TabletopCandidateContractError('TOOL0_GEOMETRY_INVALID',
                                                 'contact tilt probe failed CAD contract')
        transform = np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=translation
        return SimpleNamespace(T_base_tool0=transform, contact_center_base=center)

    return probe
