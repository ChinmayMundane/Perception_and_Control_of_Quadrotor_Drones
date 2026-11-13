"""Exercise ROS message handling without a running ROS master or model weights."""

import importlib.util
from pathlib import Path
import threading
import time
from unittest.mock import Mock, patch

import numpy as np
import pytest
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo


spec = importlib.util.spec_from_file_location(
    'depth_node', str(Path(__file__).resolve().parents[1] / 'scripts/depth_to_cloud.py'))
node_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node_module)


def fixture_node():
    node = node_module.DepthCloudNode.__new__(node_module.DepthCloudNode)
    node.bridge = CvBridge()
    node.target_frame, node.rectified, node.compressed = 'base_link', False, False
    node.max_age, node.tf_timeout, node.stride, node.max_points = 1.0, 0.05, 1, 100
    node.depth_options = {}
    node.publish_2d, node.body_frame, node.planar_frame = True, 'base_link', 'base_link'
    node.planar_options = dict(min_height=-0.25, max_height=0.25,
                               angle_bin_degrees=1.0, max_points=720)
    node.planar_pub = Mock()
    node.model = Mock()
    node.model.predict.return_value = np.full((3, 3), 2, dtype=np.float32)
    node.cloud_pub, node.depth_pub, node.tf_buffer = Mock(), Mock(), Mock()
    tf = TransformStamped()
    tf.transform.translation.x = 0.1
    tf.transform.rotation.x, tf.transform.rotation.y = -0.5, 0.5
    tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5
    node.tf_buffer.lookup_transform.return_value = tf
    info = CameraInfo(width=3, height=3)
    info.header.frame_id = 'camera_optical'
    info.K = [2, 0, 1, 0, 2, 1, 0, 0, 1]
    image = node.bridge.cv2_to_imgmsg(np.zeros((3, 3, 3), dtype=np.uint8), 'rgb8')
    image.header.frame_id, image.header.stamp = 'camera_optical', rospy.Time(100)
    return node, image, info


@pytest.fixture(autouse=True)
def no_master():
    with patch.object(rospy.Time, 'now', return_value=rospy.Time(100, 100000000)), \
            patch.object(rospy, 'logwarn_throttle'), patch.object(rospy, 'loginfo_throttle'):
        yield


def test_cloud_message_preserves_capture_stamp_frame_and_xyz():
    node, image, info = fixture_node()
    node.process(image, info, time.monotonic())
    cloud = node.cloud_pub.publish.call_args[0][0]
    assert cloud.header.stamp == image.header.stamp
    assert cloud.header.frame_id == 'base_link'
    points = np.array(list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'))))
    np.testing.assert_allclose(points[4], [2.1, 0, 0], atol=1e-6)
    depth = node.depth_pub.publish.call_args[0][0]
    assert depth.encoding == '32FC1'
    assert depth.header == image.header
    np.testing.assert_allclose(node.bridge.imgmsg_to_cv2(depth), 2)
    assert node.tf_buffer.lookup_transform.call_args[0][2] == image.header.stamp


@pytest.mark.parametrize('failure', ['stale', 'zero_stamp', 'missing_info', 'empty', 'slow_inference'])
def test_unknown_or_stale_frames_do_not_publish_clouds(failure):
    node, image, info = fixture_node()
    if failure == 'stale':
        image.header.stamp = rospy.Time(90)
    elif failure == 'zero_stamp':
        image.header.stamp = rospy.Time()
    elif failure == 'missing_info':
        info = None
    elif failure == 'empty':
        node.model.predict.return_value[:] = np.nan
    else:
        node.fresh = Mock(side_effect=[True, False])
    node.process(image, info, time.monotonic())
    node.cloud_pub.publish.assert_not_called()


def test_missing_tf_does_not_relabel_optical_points_as_body_points():
    node, image, info = fixture_node()
    node.tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException('No TF')
    with pytest.raises(tf2_ros.LookupException):
        node.process(image, info, time.monotonic())
    node.cloud_pub.publish.assert_not_called()


def test_frame_mismatch_is_rejected():
    node, image, info = fixture_node()
    info.header.frame_id = 'another_camera'
    with pytest.raises(ValueError):
        node.process(image, info, time.monotonic())


def test_pending_frame_is_replaced_with_latest():
    node, old_image, info = fixture_node()
    node.lock, node.ready, node.info = threading.Lock(), threading.Event(), info
    node.on_image(old_image)
    new_image = Mock()
    node.on_image(new_image)
    assert node.pending[0] is new_image
    assert node.pending[1] is info


def test_compressed_input():
    node, image, info = fixture_node()
    node.compressed = True
    compressed = node.bridge.cv2_to_compressed_imgmsg(np.zeros((3, 3, 3), dtype=np.uint8))
    compressed.header = image.header
    node.process(compressed, info, time.monotonic())
    node.cloud_pub.publish.assert_called_once()


def test_planar_cloud_uses_horizontal_body_plane_and_capture_time():
    node, image, info = fixture_node()
    node.process(image, info, time.monotonic())
    cloud = node.planar_pub.publish.call_args[0][0]
    assert cloud.header.frame_id == 'base_link'
    assert cloud.header.stamp == image.header.stamp
    points = np.array(list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'))))
    assert len(points) == 3  # Only the centre row lies inside the body-height band.
    np.testing.assert_allclose(points[:, 0], 2.1)
    np.testing.assert_allclose(points[:, 2], 0)


def test_planar_world_coordinates_are_transformed_before_flattening():
    node, image, info = fixture_node()
    node.planar_frame = 'odom'
    body_tf = node.tf_buffer.lookup_transform.return_value
    world_tf = TransformStamped()
    world_tf.transform.translation.x = 10
    world_tf.transform.translation.y = 20
    world_tf.transform.rotation.z = np.sqrt(0.5)
    world_tf.transform.rotation.w = np.sqrt(0.5)
    node.tf_buffer.lookup_transform.side_effect = [body_tf, world_tf]
    node.process(image, info, time.monotonic())
    cloud = node.planar_pub.publish.call_args[0][0]
    assert cloud.header.frame_id == 'odom'
    points = np.array(list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'))))
    np.testing.assert_allclose(points[:, 1], 22.1, atol=1e-5)
    np.testing.assert_allclose(points[:, 2], 0)
    assert node.tf_buffer.lookup_transform.call_args_list[1].args[:3] == (
        'odom', 'base_link', image.header.stamp)


def test_planar_selection_happens_before_3d_point_cap():
    node, image, info = fixture_node()
    node.max_points = 1
    node.process(image, info, time.monotonic())
    assert node.cloud_pub.publish.call_args[0][0].width == 1
    assert node.planar_pub.publish.call_args[0][0].width == 3


def test_empty_planar_band_does_not_publish_fake_free_space():
    node, image, info = fixture_node()
    node.planar_options.update(min_height=10, max_height=11)
    node.process(image, info, time.monotonic())
    node.planar_pub.publish.assert_not_called()
    node.cloud_pub.publish.assert_called_once()
