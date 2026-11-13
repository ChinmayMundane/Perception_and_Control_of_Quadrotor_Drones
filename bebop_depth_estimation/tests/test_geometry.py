from types import SimpleNamespace

import numpy as np
import pytest

from bebop_depth_estimation.geometry import (
    camera_matrix, depth_to_points, filter_depth, select_planar_obstacles,
    transform_points, undistort_rgb)


def calibration():
    return SimpleNamespace(
        width=3, height=3, binning_x=0, binning_y=0,
        roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0),
        K=[2, 0, 1, 0, 2, 1, 0, 0, 1],
        P=[2, 0, 1, 0, 0, 2, 1, 0, 0, 0, 1, 0],
        distortion_model='plumb_bob', D=[0, 0, 0, 0, 0])


def test_known_plane_projects_in_metres():
    matrix = camera_matrix(calibration(), (3, 3, 3))
    points = depth_to_points(np.full((3, 3), 2.0), matrix, stride=1)
    np.testing.assert_allclose(points[4], [0, 0, 2])
    np.testing.assert_allclose(points[0], [-1, -1, 2])
    np.testing.assert_allclose(points[8], [1, 1, 2])


def test_optical_to_body_axes_and_camera_offset():
    # ROS optical -> body: forward=z, left=-x, up=-y, camera 10 cm ahead.
    points = np.array([[0, 0, 2], [1, 0, 2], [0, 1, 2]], dtype=np.float32)
    actual = transform_points(points, [0.1, 0, 0], [-0.5, 0.5, -0.5, 0.5])
    np.testing.assert_allclose(actual, [[2.1, 0, 0], [2.1, -1, 0], [2.1, 0, -1]], atol=1e-6)


def test_unknown_pixels_stay_unknown_and_calibration_is_fixed():
    depth = np.array([[0, -1, np.nan, np.inf, 0.01, 2, 20]], dtype=np.float32)
    actual = filter_depth(depth, scale=0.5, offset=0.1, min_depth=0.2, max_depth=10)
    assert np.isnan(actual[0, :5]).all()
    assert np.isnan(actual[0, -1])
    assert actual[0, 5] == pytest.approx(1.1)
    points = depth_to_points(actual, np.eye(3), stride=1)
    assert points.shape == (1, 3)
    assert np.isfinite(points).all()


def test_point_limit_keeps_nearest_sampled_points():
    points = depth_to_points(np.array([[3.0, 1.0, 2.0]]), np.diag([100, 100, 1]),
                             stride=1, max_points=2)
    assert sorted(points[:, 2]) == [1, 2]


@pytest.mark.parametrize('change', ['missing', 'dimensions', 'binning', 'roi', 'distortion'])
def test_invalid_calibration_is_rejected(change):
    info = calibration()
    if change == 'missing':
        info.K = [0] * 9
    elif change == 'dimensions':
        info.width = 4
    elif change == 'binning':
        info.binning_x = 2
    elif change == 'roi':
        info.roi.x_offset = 1
    else:
        info.distortion_model = 'equidistant'
    with pytest.raises(ValueError):
        camera_matrix(info, (3, 3, 3))


def test_rectified_image_uses_projection_calibration():
    info = calibration()
    info.P[0] = 4
    assert camera_matrix(info, (3, 3), rectified=True)[0, 0] == 4
    image = np.zeros((3, 3, 3), dtype=np.uint8)
    np.testing.assert_array_equal(undistort_rgb(image, info, np.eye(3), True), image)


def test_no_points_for_unknown_depth():
    assert depth_to_points(np.full((2, 2), np.nan), np.eye(3)).shape == (0, 3)


def test_invalid_transform_is_rejected():
    with pytest.raises(ValueError):
        transform_points(np.ones((1, 3)), [0, 0, 0], [0, 0, 0, 0])


def test_planar_slice_rejects_floor_ceiling_and_keeps_nearest_per_bearing():
    points = np.array([[1, 0, -1], [1, 0, 1], [3, 0, 0.1], [2, 0, 0.2],
                       [2, 1, 0], [np.nan, 0, 0], [0, 0, 0]])
    selected = select_planar_obstacles(points)
    assert selected.shape == (2, 3)
    assert set(map(tuple, selected[:, :2])) == {(2, 0), (2, 1)}
    assert selected[0, 2] == pytest.approx(0.2)  # Preserve z until after world TF.


def test_planar_point_limit_and_empty_band():
    points = np.array([[4, 0, 0], [0, 2, 0], [-1, 0, 0]])
    selected = select_planar_obstacles(points, max_points=1)
    np.testing.assert_allclose(selected, [[-1, 0, 0]])
    assert select_planar_obstacles(points, min_height=2, max_height=3).shape == (0, 3)


@pytest.mark.parametrize('options', [dict(min_height=1, max_height=0),
                                    dict(angle_bin_degrees=0), dict(max_points=0)])
def test_invalid_planar_options_are_rejected(options):
    with pytest.raises(ValueError):
        select_planar_obstacles(np.empty((0, 3)), **options)
