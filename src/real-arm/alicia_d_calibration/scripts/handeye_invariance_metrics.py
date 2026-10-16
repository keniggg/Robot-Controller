#!/usr/bin/env python3
"""Small transform statistics helpers for passive hand-eye verification."""

import math

import numpy as np


def normalize_quaternion(quaternion):
    q = np.asarray(quaternion, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm <= 0.0 or not np.isfinite(norm):
        raise ValueError("invalid quaternion norm")
    return q / norm


def quaternion_from_matrix(matrix):
    m = np.asarray(matrix, dtype=float)
    if m.shape == (4, 4):
        m = m[:3, :3]
    if m.shape != (3, 3):
        raise ValueError("matrix must be 3x3 or 4x4")

    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion([x, y, z, w])


def matrix_from_quaternion(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def transform_from_translation_quaternion(translation, quaternion):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = matrix_from_quaternion(quaternion)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def average_quaternion(quaternions):
    quats = [normalize_quaternion(q) for q in quaternions]
    if not quats:
        raise ValueError("at least one quaternion is required")
    reference = quats[0]
    aligned = []
    for quat in quats:
        aligned.append(-quat if float(np.dot(reference, quat)) < 0.0 else quat)
    accum = np.zeros((4, 4), dtype=float)
    for quat in aligned:
        accum += np.outer(quat, quat)
    values, vectors = np.linalg.eigh(accum)
    return normalize_quaternion(vectors[:, int(np.argmax(values))])


def quaternion_angle_rad(left, right):
    q1 = normalize_quaternion(left)
    q2 = normalize_quaternion(right)
    dot = abs(float(np.dot(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def mean_transform(transforms):
    matrices = [np.asarray(t, dtype=float).reshape(4, 4) for t in transforms]
    if not matrices:
        raise ValueError("at least one transform is required")
    translations = np.array([m[:3, 3] for m in matrices], dtype=float)
    quaternions = [quaternion_from_matrix(m) for m in matrices]
    return transform_from_translation_quaternion(
        np.mean(translations, axis=0),
        average_quaternion(quaternions),
    )


def summarize_transforms(transforms):
    matrices = [np.asarray(t, dtype=float).reshape(4, 4) for t in transforms]
    if not matrices:
        raise ValueError("at least one transform is required")
    mean = mean_transform(matrices)
    mean_translation = mean[:3, 3]
    mean_quaternion = quaternion_from_matrix(mean)
    translation_errors = [
        float(np.linalg.norm(matrix[:3, 3] - mean_translation)) for matrix in matrices
    ]
    orientation_errors = [
        quaternion_angle_rad(quaternion_from_matrix(matrix), mean_quaternion)
        for matrix in matrices
    ]
    return {
        "count": len(matrices),
        "mean_translation_m": mean_translation.tolist(),
        "mean_quaternion_xyzw": mean_quaternion.tolist(),
        "translation_rms_m": float(math.sqrt(np.mean(np.square(translation_errors)))),
        "translation_max_m": float(max(translation_errors)),
        "orientation_rms_rad": float(math.sqrt(np.mean(np.square(orientation_errors)))),
        "orientation_max_rad": float(max(orientation_errors)),
        "orientation_rms_deg": float(
            math.degrees(math.sqrt(np.mean(np.square(orientation_errors))))
        ),
        "orientation_max_deg": float(math.degrees(max(orientation_errors))),
    }
