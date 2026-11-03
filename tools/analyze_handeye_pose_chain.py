#!/usr/bin/env python3
"""Read archived stationary hand-eye observations; never fit or contact ROS.

Usage:
  python3 tools/analyze_handeye_pose_chain.py SESSION_DIRECTORY
Output defaults to SESSION_DIRECTORY/pose_chain_independent_diagnostic.json.
Only Python's standard library and NumPy are required.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np


JOINT_NAMES = ["Joint%d" % n for n in range(1, 7)]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Expected finite 4x4 transform")
    return matrix


def angle_deg(matrix):
    cosine = (np.trace(matrix[:3, :3]) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def relative_angles(first, second):
    # T_base_tool * X * T_camera_board is constant for a fixed rigid board.
    # Consequently A X = X B, and angle(A) = angle(B) for every rigid X.
    a = np.linalg.inv(transform(first["T_base_tool"])) @ transform(second["T_base_tool"])
    b = transform(first["T_camera_board"]) @ np.linalg.inv(transform(second["T_camera_board"]))
    robot, camera = angle_deg(a), angle_deg(b)
    return robot, camera, camera - robot


def arm_positions(sample):
    mapping = dict(zip(sample["accepted_joint_names"], sample["accepted_joint_positions"]))
    values = np.asarray([mapping[name] for name in JOINT_NAMES], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("Nonfinite accepted joint position")
    return values


def analyze(session):
    source = session / "task_pose_validation_result.json"
    validation = json.loads(source.read_text())
    windows, offsets_ms, capture_sources, capture_data = [], [], [], {}
    angular_pairs, return_pairs = [], []
    for segment_index, segment in enumerate(validation["segments"]):
        for group in segment["groups"]:
            capture_path = session / group["capture"] / "capture.json"
            capture = json.loads(capture_path.read_text())
            samples = capture["samples"]
            if not samples:
                raise ValueError("Empty observation window: %s" % capture_path)
            capture_data[group["capture"]] = samples
            capture_sources.append({"path": str(capture_path), "sha256": digest(capture_path)})
            q = np.asarray([arm_positions(sample) for sample in samples])
            source_stamps = [sample["source_stamp_ns"] for sample in samples]
            sample_offsets = []
            for sample in samples:
                offset_ns = sample["accepted_joint_stamp_ns"] - sample["source_stamp_ns"]
                if offset_ns != sample["accepted_joint_time_offset_ns"]:
                    raise ValueError("Archived time offset disagrees with source stamps")
                sample_offsets.append(offset_ns / 1e6)
                if sample["control_reference_epoch_ns"] != segment["epoch_ns"]:
                    raise ValueError("Mixed control epochs within archived window")
            offsets_ms.extend(sample_offsets)
            tools = [transform(sample["T_base_tool"]) for sample in samples]
            tools_xyz = np.asarray([matrix[:3, 3] for matrix in tools])
            windows.append({
                "capture": group["capture"], "segment_index": segment_index,
                "epoch_ns": segment["epoch_ns"], "pose_id": group.get("pose_id"),
                "role": group.get("role"), "frames": len(samples),
                "source_duration_sec": (max(source_stamps) - min(source_stamps)) / 1e9,
                "accepted_arm_joint_peak_to_peak_deg": np.degrees(np.ptp(q, axis=0)).tolist(),
                "accepted_arm_joint_exactly_constant": bool(np.all(q == q[0])),
                "tool_translation_max_from_first_mm": float(np.max(np.linalg.norm(tools_xyz - tools_xyz[0], axis=1)) * 1000),
                "tool_rotation_max_from_first_deg": max(angle_deg(np.linalg.inv(tools[0]) @ t) for t in tools),
                "accepted_joint_time_offset_range_ms": [min(sample_offsets), max(sample_offsets)],
            })
        for first, second in itertools.combinations(segment["groups"], 2):
            robot, camera, difference = relative_angles(first, second)
            identity = {
                "segment_index": segment_index, "epoch_ns": segment["epoch_ns"],
                "first_capture": first["capture"], "second_capture": second["capture"],
                "first_pose_id": first.get("pose_id"), "second_pose_id": second.get("pose_id"),
            }
            angular_pairs.append(dict(identity, robot_relative_rotation_deg=robot,
                                      camera_relative_rotation_deg=camera,
                                      camera_minus_robot_rotation_deg=difference))
            dq = np.asarray(second["q_deg"]) - np.asarray(first["q_deg"])
            if np.linalg.norm(dq) <= 0.5:
                delta = (transform(second["T_base_board"])[:3, 3] -
                         transform(first["T_base_board"])[:3, 3]) * 1000
                return_pairs.append(dict(identity,
                    measured_joint_delta_deg=dq.tolist(),
                    measured_joint_delta_l2_deg=float(np.linalg.norm(dq)),
                    board_delta_xyz_mm=delta.tolist(),
                    board_translation_difference_mm=float(np.linalg.norm(delta)),
                    exact_measured_joint_match=bool(np.all(dq == 0))))

    strongest = sorted(angular_pairs, key=lambda pair: abs(pair["camera_minus_robot_rotation_deg"]), reverse=True)[:5]
    robust_pairs = []
    for pair in strongest:
        differences = [relative_angles(a, b)[2] for a, b in itertools.product(
            capture_data[pair["first_capture"]], capture_data[pair["second_capture"]])]
        robust_pairs.append(dict(pair,
            first_window_frames=len(capture_data[pair["first_capture"]]),
            second_window_frames=len(capture_data[pair["second_capture"]]),
            all_frame_pair_count=len(differences),
            all_frame_camera_minus_robot_rotation_range_deg=[min(differences), max(differences)],
            all_frame_camera_minus_robot_rotation_mean_deg=float(np.mean(differences))))

    frame_count = sum(window["frames"] for window in windows)
    if frame_count != validation["raw_frames"] or len(windows) != validation["all_groups"]:
        raise ValueError("Capture count disagrees with validation summary")
    return {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "analysis_kind": "offline_archived_observations_no_fit_no_ros",
        "session": str(session),
        "source": {"path": str(source), "sha256": digest(source)},
        "analysis_script": {"path": str(Path(__file__).resolve()), "sha256": digest(Path(__file__))},
        "formulas": {
            "notation": "T_P_Q maps Q-frame coordinates into P-frame coordinates; X = T_tool_camera",
            "fixed_board": "T_base_tool_i * X * T_camera_board_i = constant T_base_board",
            "A": "inverse(T_base_tool_i) * T_base_tool_j",
            "B": "T_camera_board_i * inverse(T_camera_board_j)",
            "invariant": "A*X = X*B implies angle(A) = angle(B), independent of X",
            "angle_deg": "degrees(acos(clip((trace(R)-1)/2, -1, 1)))",
            "rotation_difference_sign": "camera relative rotation angle minus robot relative rotation angle",
            "near_return_selection": "all within-epoch capture pairs with L2 norm of selected accepted arm joint difference <= 0.5 degrees",
            "near_return_board_difference": "norm(translation(T_base_board_j) - translation(T_base_board_i)); uses existing frozen X and is not X-independent",
        },
        "summary": {
            "observation_windows": len(windows), "raw_frames": frame_count,
            "all_windows_accepted_arm_joints_exactly_constant": all(w["accepted_arm_joint_exactly_constant"] for w in windows),
            "max_window_accepted_arm_joint_peak_to_peak_deg": max(max(w["accepted_arm_joint_peak_to_peak_deg"]) for w in windows),
            "joint_image_time_offset_ms": {
                "sign": "accepted joint source stamp minus image source stamp",
                "minimum": min(offsets_ms), "median": float(np.median(offsets_ms)), "maximum": max(offsets_ms),
                "absolute_p95": float(np.percentile(np.abs(offsets_ms), 95)),
            },
            "within_epoch_rotation_pairs": len(angular_pairs),
            "near_measured_return_pairs": len(return_pairs),
        },
        "evidence_boundaries": [
            "No transforms or reference poses were fitted or replaced; all comparisons use archived values.",
            "Rotation-angle invariance is independent of hand-eye X; translation return differences still use the existing frozen X.",
            "A nonzero relative-angle discrepancy identifies inconsistency somewhere among accepted FK, camera board observations, or rigidity/static-board assumptions; it does not uniquely identify the faulty component.",
            "A fixed X cannot eliminate relative-angle discrepancy, but a revised X may still reduce some translation residuals. This analysis does not certify current X or attribute all 6.14 mm to the angular discrepancy.",
            "The 0.5-degree selection denotes nearby measured poses, not exact repeated configurations. Good wrist returns do not exclude shoulder/elbow backlash at untested exact returns.",
            "Constant accepted feedback is not external metrology of physical links. Stable camera windows and FK weaken ordinary motion-timestamp mismatch as the dominant explanation but do not rule out all systematic timing or feedback faults.",
            "Command-minus-feedback residual is not substituted for hand-eye error: archived T_base_tool already uses accepted measured feedback.",
            "All-frame combinations quantify stability of the observed discrepancy, not independent repeated trials or a statistical confidence interval.",
            "Only within-control-epoch comparisons are used; no cross-epoch calibration validation is inferred.",
        ],
        "windows": windows,
        "near_measured_return_pairs": return_pairs,
        "strongest_rotation_pairs_all_frame_robustness": robust_pairs,
        "all_within_epoch_selected_frame_rotation_pairs": angular_pairs,
        "capture_sources": capture_sources,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    session = args.session.resolve(strict=True)
    output = args.output or session / "pose_chain_independent_diagnostic.json"
    result = analyze(session)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(output), "summary": result["summary"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
