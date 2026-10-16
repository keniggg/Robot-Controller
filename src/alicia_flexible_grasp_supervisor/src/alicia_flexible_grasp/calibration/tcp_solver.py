import math

import numpy as np


class TcpCalibrationError(ValueError):
    pass


def rotation_distance_rad(first, second):
    relative = np.asarray(first, dtype=float).T.dot(np.asarray(second, dtype=float))
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) * 0.5))
    return math.acos(cosine)


def max_orientation_separation_rad(samples):
    rotations = [np.asarray(sample['rotation'], dtype=float) for sample in samples]
    return max(
        (rotation_distance_rad(a, b) for index, a in enumerate(rotations) for b in rotations[index + 1:]),
        default=0.0,
    )


def solve_tcp_pivot(samples, min_samples=6, min_orientation_separation_rad=0.35):
    """Solve p_fixed = p_i + R_i * t_tcp for TCP translation and pivot point."""
    if len(samples) < int(min_samples):
        raise TcpCalibrationError('need at least %d samples' % int(min_samples))

    orientation_span = max_orientation_separation_rad(samples)
    if orientation_span < float(min_orientation_separation_rad):
        raise TcpCalibrationError(
            'orientation span %.1f deg is below %.1f deg'
            % (math.degrees(orientation_span), math.degrees(float(min_orientation_separation_rad)))
        )

    rows = []
    values = []
    for sample in samples:
        rotation = np.asarray(sample['rotation'], dtype=float)
        translation = np.asarray(sample['translation'], dtype=float).reshape(3)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise TcpCalibrationError('sample contains an invalid rotation matrix')
        if not np.all(np.isfinite(translation)):
            raise TcpCalibrationError('sample contains an invalid translation')
        rows.append(np.hstack((rotation, -np.eye(3))))
        values.append(-translation)

    matrix = np.vstack(rows)
    vector = np.concatenate(values)
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, vector, rcond=None)
    if int(rank) < 6:
        raise TcpCalibrationError('sample poses are rank deficient; use more diverse wrist orientations')

    tcp_translation = solution[:3]
    fixed_point = solution[3:]
    errors = []
    for sample in samples:
        predicted = np.asarray(sample['translation'], dtype=float) + np.asarray(sample['rotation'], dtype=float).dot(tcp_translation)
        errors.append(float(np.linalg.norm(predicted - fixed_point)))

    rms_error = math.sqrt(sum(error * error for error in errors) / len(errors))
    condition = float(singular_values[0] / singular_values[-1]) if singular_values[-1] > 0.0 else float('inf')
    return {
        'tcp_translation': tcp_translation,
        'fixed_point': fixed_point,
        'errors': errors,
        'rms_error': rms_error,
        'max_error': max(errors),
        'sample_count': len(samples),
        'rank': int(rank),
        'condition': condition,
        'orientation_span_rad': orientation_span,
    }
