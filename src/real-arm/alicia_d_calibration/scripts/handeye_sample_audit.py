#!/usr/bin/env python3
"""Audit easy_handeye samples without saving or publishing a calibration."""

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
from easy_handeye_msgs.srv import TakeSample

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handeye_invariance_metrics import (
    mean_transform,
    quaternion_angle_rad,
    quaternion_from_matrix,
    summarize_transforms,
    transform_from_translation_quaternion,
)


ALGORITHMS = {
    "OpenCV/Tsai-Lenz": cv2.CALIB_HAND_EYE_TSAI,
    "OpenCV/Park": cv2.CALIB_HAND_EYE_PARK,
    "OpenCV/Horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "OpenCV/Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "OpenCV/Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def transform_message_to_matrix(message):
    return transform_from_translation_quaternion(
        [message.translation.x, message.translation.y, message.translation.z],
        [message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w],
    )


def compute_candidate(samples, method, indices=None):
    if indices is None:
        indices = range(len(samples))
    selected = [samples[index] for index in indices]
    if len(selected) < 3:
        raise ValueError("at least three samples are required")
    hand_rotations = [sample["T_base_tool"][:3, :3] for sample in selected]
    hand_translations = [sample["T_base_tool"][:3, 3] for sample in selected]
    marker_rotations = [sample["T_camera_board"][:3, :3] for sample in selected]
    marker_translations = [sample["T_camera_board"][:3, 3] for sample in selected]
    rotation, translation = cv2.calibrateHandEye(
        hand_rotations,
        hand_translations,
        marker_rotations,
        marker_translations,
        method=method,
    )
    candidate = np.eye(4, dtype=float)
    candidate[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    candidate[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    if not np.all(np.isfinite(candidate)):
        raise ValueError("calibration contains non-finite values")
    determinant = float(np.linalg.det(candidate[:3, :3]))
    orthogonality = float(
        np.linalg.norm(candidate[:3, :3].T.dot(candidate[:3, :3]) - np.eye(3))
    )
    if abs(determinant - 1.0) > 1e-4 or orthogonality > 1e-4:
        raise ValueError(
            "calibration rotation is invalid: det=%.6f orthogonality=%.3g"
            % (determinant, orthogonality)
        )
    return candidate


def board_transforms(samples, candidate, indices=None):
    if indices is None:
        indices = range(len(samples))
    return [
        samples[index]["T_base_tool"].dot(candidate).dot(
            samples[index]["T_camera_board"]
        )
        for index in indices
    ]


def errors_against_reference(transforms, reference):
    reference_translation = reference[:3, 3]
    reference_quaternion = quaternion_from_matrix(reference)
    translation_errors = []
    orientation_errors_deg = []
    for transform in transforms:
        translation_errors.append(
            float(np.linalg.norm(transform[:3, 3] - reference_translation))
        )
        orientation_errors_deg.append(
            math.degrees(
                quaternion_angle_rad(
                    quaternion_from_matrix(transform), reference_quaternion
                )
            )
        )
    return translation_errors, orientation_errors_deg


def error_summary(translation_errors, orientation_errors_deg):
    return {
        "count": len(translation_errors),
        "translation_rms_m": float(
            math.sqrt(np.mean(np.square(translation_errors)))
        ),
        "translation_max_m": float(max(translation_errors)),
        "orientation_rms_deg": float(
            math.sqrt(np.mean(np.square(orientation_errors_deg)))
        ),
        "orientation_max_deg": float(max(orientation_errors_deg)),
    }


def cross_validate(samples, method, fold_count=5):
    if len(samples) < fold_count * 2:
        raise ValueError("not enough samples for %d-fold validation" % fold_count)
    all_indices = list(range(len(samples)))
    candidates = []
    translation_errors = []
    orientation_errors_deg = []
    folds = []
    for fold in range(fold_count):
        test_indices = all_indices[fold::fold_count]
        test_set = set(test_indices)
        train_indices = [index for index in all_indices if index not in test_set]
        candidate = compute_candidate(samples, method, train_indices)
        reference = mean_transform(board_transforms(samples, candidate, train_indices))
        test_transforms = board_transforms(samples, candidate, test_indices)
        fold_translation, fold_orientation = errors_against_reference(
            test_transforms, reference
        )
        candidates.append(candidate)
        translation_errors.extend(fold_translation)
        orientation_errors_deg.extend(fold_orientation)
        folds.append(
            {
                "fold": fold + 1,
                "train_indices_1based": [index + 1 for index in train_indices],
                "test_indices_1based": [index + 1 for index in test_indices],
                "summary": error_summary(fold_translation, fold_orientation),
            }
        )
    return {
        "summary": error_summary(translation_errors, orientation_errors_deg),
        "candidate_spread": summarize_transforms(candidates),
        "folds": folds,
    }


def candidate_record(samples, method, thresholds):
    candidate = compute_candidate(samples, method)
    transforms = board_transforms(samples, candidate)
    in_sample = summarize_transforms(transforms)
    reference = mean_transform(transforms)
    translation_errors, orientation_errors = errors_against_reference(
        transforms, reference
    )
    worst = sorted(
        [
            {
                "index_1based": index + 1,
                "translation_error_m": translation_errors[index],
                "orientation_error_deg": orientation_errors[index],
            }
            for index in range(len(samples))
        ],
        key=lambda item: (
            item["translation_error_m"], item["orientation_error_deg"]
        ),
        reverse=True,
    )
    cross_validation = cross_validate(samples, method)
    cv_summary = cross_validation["summary"]
    cross_validation["passes_thresholds"] = bool(
        cv_summary["translation_rms_m"] <= thresholds["translation_rms_m"]
        and cv_summary["translation_max_m"] <= thresholds["translation_max_m"]
        and cv_summary["orientation_rms_deg"] <= thresholds["orientation_rms_deg"]
        and cv_summary["orientation_max_deg"] <= thresholds["orientation_max_deg"]
    )
    return {
        "T_tool_camera": candidate.tolist(),
        "translation_m": candidate[:3, 3].tolist(),
        "quaternion_xyzw": quaternion_from_matrix(candidate).tolist(),
        "in_sample": in_sample,
        "cross_validation": cross_validation,
        "worst_samples": worst[:10],
    }


def load_samples(namespace):
    service_name = namespace.rstrip("/") + "/get_sample_list"
    rospy.wait_for_service(service_name, timeout=5.0)
    response = rospy.ServiceProxy(service_name, TakeSample)()
    hands = list(response.samples.hand_world_samples)
    markers = list(response.samples.camera_marker_samples)
    if len(hands) != len(markers):
        raise ValueError("robot/camera sample counts differ")
    return [
        {
            "T_base_tool": transform_message_to_matrix(hand),
            "T_camera_board": transform_message_to_matrix(marker),
        }
        for hand, marker in zip(hands, markers)
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--output", default="")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("handeye_sample_audit", anonymous=True, disable_signals=True)
    samples = load_samples(args.namespace)
    thresholds = {
        "translation_rms_m": 0.003,
        "translation_max_m": 0.005,
        "orientation_rms_deg": 1.0,
        "orientation_max_deg": 2.0,
    }
    report = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "namespace": args.namespace,
        "sample_count": len(samples),
        "fold_contract": "five deterministic folds; test indices fold::5",
        "thresholds": thresholds,
        "algorithms": {},
    }
    for name, method in ALGORITHMS.items():
        try:
            report["algorithms"][name] = candidate_record(
                samples, method, thresholds
            )
        except Exception as exc:
            report["algorithms"][name] = {"error": str(exc)}
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as handle:
            handle.write(encoded + "\n")
    print(encoded)


if __name__ == "__main__":
    main()
