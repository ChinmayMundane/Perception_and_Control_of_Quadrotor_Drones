"""Projection uses optical axes: x right, y down, z forward, in metres."""

import cv2
import numpy as np


def camera_matrix(info, image_shape, rectified=False):
    """Reject ambiguous calibration rather than inventing camera intrinsics."""
    if (info.height, info.width) != tuple(image_shape[:2]):
        raise ValueError('CameraInfo dimensions must match the incoming image')
    if info.binning_x > 1 or info.binning_y > 1 or any((
            info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)):
        raise ValueError('Use full-frame CameraInfo without ROI or binning')
    if rectified:
        projection = np.asarray(info.P, dtype=np.float64).reshape(3, 4)
        if not np.allclose(projection[:, 3], 0):
            raise ValueError('Expected a monocular projection matrix (P[:,3] = 0)')
        matrix = projection[:, :3].copy()
    else:
        matrix = np.asarray(info.K, dtype=np.float64).reshape(3, 3)
        if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            raise ValueError('Unsupported raw distortion model: ' + info.distortion_model)
        if not np.all(np.isfinite(info.D)):
            raise ValueError('Camera distortion contains non-finite values')
    if (not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or not np.allclose([matrix[0, 1], matrix[1, 0]], 0)):
        raise ValueError('CameraInfo must contain calibrated pinhole intrinsics')
    return matrix


def undistort_rgb(rgb, info, matrix, rectified=False):
    # Keep the same K and image dimensions, so projection needs no rescaling.
    if rectified or not info.D or np.allclose(info.D, 0):
        return rgb
    return cv2.undistort(rgb, matrix, np.asarray(info.D), None, matrix)


def filter_depth(depth, scale=1.0, offset=0.0, min_depth=0.1, max_depth=10.0):
    """Apply a fixed calibration; invalid pixels stay unknown (NaN)."""
    if (not np.isfinite([scale, offset, min_depth, max_depth]).all()
            or scale <= 0 or min_depth <= 0 or max_depth <= min_depth):
        raise ValueError('Invalid depth calibration or range')
    source = np.asarray(depth, dtype=np.float32)
    if source.ndim != 2:
        raise ValueError('Depth must be an HxW array')
    result = source * scale + offset
    valid = (np.isfinite(source) & (source > 0) & np.isfinite(result)
             & (result >= min_depth) & (result <= max_depth))
    return np.where(valid, result, np.nan).astype(np.float32)


def depth_to_points(depth, matrix, stride=4, max_points=2000):
    """Back-project axial depth. If capped, keep the nearest sampled points."""
    if stride < 1 or (max_points is not None and max_points < 1):
        raise ValueError('stride and max_points must be positive')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0)
    z, u, v = z[valid], u[valid], v[valid]
    points = np.column_stack(((u - matrix[0, 2]) * z / matrix[0, 0],
                              (v - matrix[1, 2]) * z / matrix[1, 1], z))
    if max_points is not None and len(points) > max_points:
        distance_squared = np.einsum('ij,ij->i', points, points)
        points = points[np.argpartition(distance_squared, max_points - 1)[:max_points]]
    return np.asarray(points, dtype=np.float32)


def select_planar_obstacles(points, min_height=-0.25, max_height=0.25,
                            angle_bin_degrees=1.0, max_points=720):
    """Select nearest body-frame obstacle per bearing within the flight-height band.

    Keep the selected original z until after any world transform. The publisher
    sets z=0 in the final output frame, preserving the correct world x/y.
    """
    if (not np.isfinite([min_height, max_height, angle_bin_degrees]).all()
            or min_height > max_height or not 0 < angle_bin_degrees <= 180
            or max_points < 1):
        raise ValueError('Invalid planar height band, angle bin or point limit')
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    ranges = np.hypot(points[:, 0], points[:, 1])
    valid = (np.isfinite(points).all(axis=1) & (ranges > 1e-6)
             & (points[:, 2] >= min_height) & (points[:, 2] <= max_height))
    points, ranges = points[valid], ranges[valid]
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)
    count = int(np.ceil(360.0 / angle_bin_degrees))
    bearings = np.arctan2(points[:, 1], points[:, 0])
    bins = np.floor((bearings + np.pi) / (2 * np.pi) * count).astype(np.int64) % count
    # Sort by range first so unique() keeps the closest point in each angular bin.
    order = np.argsort(ranges, kind='stable')
    _, first = np.unique(bins[order], return_index=True)
    selected = order[first]
    if len(selected) > max_points:
        selected = selected[np.argsort(ranges[selected], kind='stable')[:max_points]]
    return points[selected].copy()


def transform_points(points, translation, quaternion):
    """Apply a TF transform including camera translation, not just rotation."""
    q = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
    translation = np.asarray(translation, dtype=np.float64)
    if not np.isfinite(translation).all():
        raise ValueError('Invalid TF translation')
    return np.asarray(points @ rotation.T + translation, dtype=np.float32)
