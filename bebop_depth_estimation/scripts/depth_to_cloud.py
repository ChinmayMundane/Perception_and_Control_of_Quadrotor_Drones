#!/usr/bin/env python3
"""ROS1 image + CameraInfo -> CPU/CUDA metric depth -> PointCloud2 in a TF frame."""

import copy
import threading
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import Header

from bebop_depth_estimation.geometry import (
    camera_matrix, depth_to_points, filter_depth, select_planar_obstacles,
    transform_points, undistort_rgb)
from bebop_depth_estimation.inference import MetricDepth


def cloud_message(points, stamp, frame):
    points = np.ascontiguousarray(points, dtype='<f4')
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12 * len(points)
    msg.data = points.tobytes()
    msg.is_dense = True
    return msg


class DepthCloudNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.pending = None
        self.info = None
        self.target_frame = rospy.get_param('~target_frame', 'base_link').lstrip('/')
        self.rectified = rospy.get_param('~image_rectified', False)
        self.compressed = rospy.get_param('~compressed', False)
        self.rate = float(rospy.get_param('~max_rate', 5.0))
        self.max_age = float(rospy.get_param('~max_age', 1.0))
        self.tf_timeout = float(rospy.get_param('~tf_timeout', 0.05))
        self.stride = int(rospy.get_param('~pixel_stride', 4))
        self.max_points = int(rospy.get_param('~max_points', 2000))
        self.publish_2d = rospy.get_param('~publish_2d', True)
        self.body_frame = rospy.get_param('~body_frame', 'base_link').lstrip('/')
        self.planar_frame = rospy.get_param('~planar_frame', 'base_link').lstrip('/')
        self.planar_options = dict(
            min_height=float(rospy.get_param('~slice_min_height', -0.25)),
            max_height=float(rospy.get_param('~slice_max_height', 0.25)),
            angle_bin_degrees=float(rospy.get_param('~angle_bin_degrees', 1.0)),
            max_points=int(rospy.get_param('~max_planar_points', 720)))
        if self.publish_2d and (not self.body_frame or not self.planar_frame):
            raise ValueError('2D output requires body_frame and planar_frame')
        select_planar_obstacles(np.empty((0, 3)), **self.planar_options)
        self.depth_options = dict(
            scale=float(rospy.get_param('~depth_scale', 1.0)),
            offset=float(rospy.get_param('~depth_offset', 0.0)),
            min_depth=float(rospy.get_param('~min_depth', 0.1)),
            max_depth=float(rospy.get_param('~max_depth', 10.0)))
        if (not np.isfinite([self.rate, self.max_age, self.tf_timeout]).all()
                or min(self.rate, self.max_age) <= 0 or self.tf_timeout < 0
                or self.stride < 1 or self.max_points < 1):
            raise ValueError('Invalid rate, age, TF timeout, stride or point limit')
        filter_depth(np.ones((1, 1)), **self.depth_options)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.depth_pub = rospy.Publisher(
            rospy.get_param('~depth_topic', '/bebop/depth/image'), Image, queue_size=1)
        self.cloud_pub = rospy.Publisher(
            rospy.get_param('~cloud_topic', '/bebop/depth/points'), PointCloud2, queue_size=1)
        self.planar_pub = (rospy.Publisher(
            rospy.get_param('~planar_topic', '/bebop/depth/points_2d'), PointCloud2, queue_size=1)
            if self.publish_2d else None)
        requested_device = rospy.get_param('~device', 'auto')
        rospy.loginfo('Loading Depth Anything V2 metric (device=%s)...', requested_device)
        self.model = MetricDepth(
            rospy.get_param('~model_path'), int(rospy.get_param('~input_size', 518)),
            int(rospy.get_param('~num_threads', 4)), device=requested_device)
        if requested_device == 'auto' and self.model.device.type == 'cpu':
            rospy.loginfo('CUDA unavailable in this environment; auto selected CPU. '
                          'GPU use requires CUDA-enabled PyTorch and an NVIDIA driver.')
        self.info_sub = rospy.Subscriber(
            rospy.get_param('~camera_info_topic', '/bebop/camera_info'),
            CameraInfo, self.on_info, queue_size=1)
        self.image_sub = rospy.Subscriber(
            rospy.get_param('~image_topic', '/bebop/image_raw'),
            CompressedImage if self.compressed else Image,
            self.on_image, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.loginfo('Depth ready on %s; waiting for images, calibration and TF to %s',
                      self.model.device, self.target_frame or 'the optical frame')
        rospy.logwarn('Monocular depth is estimated geometry; validate scale and latency before CBF use.')

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def on_image(self, msg):
        # Subscriber callbacks only replace the pending image; inference has one worker.
        with self.lock:
            self.pending = (msg, self.info, time.monotonic())
        self.ready.set()

    def fresh(self, stamp, received):
        # Wall-clock check also bounds latency if /clock is paused.
        age = (rospy.Time.now() - stamp).to_sec()
        return (-0.05 <= age <= self.max_age
                and time.monotonic() - received <= self.max_age)

    def points_in_frame(self, points, source, target, stamp):
        if source == target:
            return points
        started = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, stamp, rospy.Duration(self.tf_timeout)).transform
            t, q = transform.translation, transform.rotation
            return transform_points(points, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        finally:
            self.frame_transform_seconds += time.monotonic() - started

    def process(self, msg, info, received):
        if info is None:
            rospy.logwarn_throttle(5, 'Waiting for CameraInfo; cannot project without calibration')
            return
        if msg.header.stamp == rospy.Time() or not self.fresh(msg.header.stamp, received):
            rospy.logwarn_throttle(5, 'Dropping zero-stamped or stale camera image')
            return
        frame = msg.header.frame_id.lstrip('/')
        if not frame or (info.header.frame_id and frame != info.header.frame_id.lstrip('/')):
            raise ValueError('Image and CameraInfo must refer to the same camera optical frame')
        rgb = (self.bridge.compressed_imgmsg_to_cv2(msg, 'rgb8') if self.compressed
               else self.bridge.imgmsg_to_cv2(msg, 'rgb8'))
        matrix = camera_matrix(info, rgb.shape, self.rectified)
        rgb = undistort_rgb(rgb, info, matrix, self.rectified)
        start = time.monotonic()
        self.frame_transform_seconds = 0.0
        depth = filter_depth(self.model.predict(rgb), **self.depth_options)
        inference_seconds = time.monotonic() - start
        # Do not cap the 3D cloud before selecting the planar height band: nearby
        # floor/ceiling points could otherwise remove the relevant wall samples.
        points = depth_to_points(depth, matrix, self.stride, max_points=None)
        if len(points) == 0:
            # The existing CBF interprets an empty cloud as free space.
            rospy.logwarn_throttle(5, 'No valid estimated depth; suppressing cloud (unknown space)')
            return
        cloud_frame = self.target_frame or frame
        planar_cloud = None
        body_points = None
        if self.publish_2d:
            body_points = self.points_in_frame(points, frame, self.body_frame, msg.header.stamp)
            planar = select_planar_obstacles(body_points, **self.planar_options)
            if len(planar):
                planar = self.points_in_frame(
                    planar, self.body_frame, self.planar_frame, msg.header.stamp).copy()
                planar[:, 2] = 0.0
                planar_cloud = cloud_message(planar, msg.header.stamp, self.planar_frame)
            else:
                rospy.logwarn_throttle(5, 'No valid depth in planar height band; suppressing 2D cloud')
        cloud_points = (body_points if body_points is not None and cloud_frame == self.body_frame
                        else self.points_in_frame(points, frame, cloud_frame, msg.header.stamp))
        if len(cloud_points) > self.max_points:
            ranges = np.einsum('ij,ij->i', cloud_points, cloud_points)
            keep = np.argpartition(ranges, self.max_points - 1)[:self.max_points]
            cloud_points = cloud_points[keep]
        cloud = cloud_message(cloud_points, msg.header.stamp, cloud_frame)
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = copy.deepcopy(msg.header)
        if not self.fresh(msg.header.stamp, received):
            rospy.logwarn_throttle(5, 'Inference exceeded max_age; dropping result. Lower input_size or rate.')
            return
        self.depth_pub.publish(depth_msg)
        self.cloud_pub.publish(cloud)
        if planar_cloud is not None:
            self.planar_pub.publish(planar_cloud)
        rospy.loginfo_throttle(
            5, 'Depth (%s): %.3f s processing [inference %.3f s, transforms %.3f s], '
            '%.3f s image age, %d points', self.model.device, time.monotonic() - start,
            inference_seconds, self.frame_transform_seconds,
            (rospy.Time.now() - msg.header.stamp).to_sec(), len(cloud_points))

    def run(self):
        next_start = 0.0
        last_result = time.monotonic()
        while not rospy.is_shutdown():
            # Use wall time so a paused simulator cannot trap this worker in Rate.sleep.
            self.ready.wait(timeout=0.1)
            delay = next_start - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 0.05))
                continue
            with self.lock:
                item, self.pending = self.pending, None
                self.ready.clear()
            if item is None:
                if time.monotonic() - last_result > self.max_age:
                    rospy.logwarn_throttle(5, 'No fresh camera input; no new cloud is being published')
                continue
            next_start = time.monotonic() + 1.0 / self.rate
            try:
                self.process(*item)
            except (ValueError, CvBridgeError, cv2.error, RuntimeError,
                    tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                rospy.logwarn_throttle(5, 'Depth frame dropped: %s', error)
            last_result = time.monotonic()


if __name__ == '__main__':
    rospy.init_node('bebop_depth_estimation')
    try:
        DepthCloudNode().run()
    except (ValueError, ImportError, OSError, RuntimeError) as error:
        rospy.logfatal('Cannot start depth node: %s', error)
        raise SystemExit(1)
