#!/usr/bin/env python3
"""Extract a time-aligned ROS1 bag snapshot without starting ROS playback.

Example: python3 extract_data.py cbf.bag --time 11.5 --output data1.npz
Time is seconds from the bag start. Messages are aligned using their recorded
timestamps. No ROS master, subscriptions, playback, or /clock is used.
"""
import argparse
import io
import json
from pathlib import Path

import numpy as np
import rosbag
import roslib.message
from cv_bridge import CvBridge


EXCLUDED_PREFIXES = (
    '/vrpn_client_node/', '/camera/color/estimated/',
    '/depthimage_to_laserscan_est/', '/bebop/depth/',
)
EXCLUDED_TOPICS = {'/estimated_depth_scan', '/estimated_noisy_cloud'}
TF_TOPICS = {'/tf', '/tf_static'}
BINARY_TYPES = {'sensor_msgs/Image', 'sensor_msgs/CompressedImage',
                'theora_image_transport/Packet'}


def excluded(topic):
    return topic in EXCLUDED_TOPICS or topic.startswith(EXCLUDED_PREFIXES)


def sample_time(message, recorded):
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is not None and stamp.to_sec() > 0:
        return stamp.to_sec(), 'header'
    return recorded.to_sec(), 'bag_record'


def candidate(message, recorded, connection, policy):
    stamp, source = sample_time(message, recorded)
    return {'message': message, 'recorded': recorded, 'connection': connection,
            'time': stamp, 'time_source': source, 'policy': policy}


def summary(item, anchor, bag_start):
    return {'type': item['message']._type, 'selection': item['policy'],
            'md5sum': item['message']._md5sum,
            'message_definition': item['message']._full_text,
            'timestamp_source': item['time_source'],
            'selected_time_s': item['time'],
            'selected_offset_s': item['time']-bag_start,
            'offset_from_image_s': item['time']-anchor,
            'original_bag_record_time_s': item['recorded'].to_sec()}


def serialize(message):
    buffer = io.BytesIO()
    message.serialize(buffer)
    return np.frombuffer(buffer.getvalue(), dtype=np.uint8).copy()


def stamp_pair(header):
    return np.array([header.stamp.secs, header.stamp.nsecs], dtype=np.int64)


def extract(bag_path, offset, output, image_topic='/bebop/image_raw',
            camera_info_topic='/bebop/camera_info'):
    bag_path, output = Path(bag_path).expanduser(), Path(output).expanduser()
    if not bag_path.is_file():
        raise ValueError('Bag file not found: '+str(bag_path))
    if output.suffix != '.npz' or output.exists():
        raise ValueError('Output must be a new .npz file: '+str(output))
    if not float('-inf') < offset < float('inf') or offset < 0:
        raise ValueError('Time must be a finite, nonnegative number of seconds')

    with rosbag.Bag(str(bag_path)) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        if start+offset > end:
            raise ValueError('Time is outside this {:.3f}-second recording'.format(end-start))
        topics = bag.get_type_and_topic_info().topics
        if image_topic not in topics or topics[image_topic].msg_type != 'sensor_msgs/Image':
            raise ValueError('An uncompressed sensor_msgs/Image topic is required: '+image_topic)
        target = start+offset
        image_item, image_score = None, None
        # Scan header times, rather than assuming header and recording times match.
        for _, msg, recorded, connection in bag.read_messages(
                topics=[image_topic], return_connection_header=True):
            if msg.header.stamp.to_sec() <= 0:
                continue
            item = candidate(msg, recorded, connection, 'nearest_image')
            score = (abs(item['time']-target), item['time'])
            if image_score is None or score < image_score:
                image_item, image_score = item, score
        if image_item is None:
            raise ValueError('No stamped images were found on '+image_topic)
        image = image_item['message']
        anchor = image_item['time']
        selected = {image_topic: image_item}
        transforms = {topic: {} for topic in TF_TOPICS}
        allowed = [topic for topic in topics if topic != image_topic and not excluded(topic)]
        latched_topics = set()

        def inspect_connection(topic, datatype, md5sum, definition, header):
            # Inspect connection metadata only; returning False avoids reading payloads.
            if header.get('latching') in ('1', b'1'):
                latched_topics.add(topic)
            return False

        for _ in bag.read_messages(topics=allowed, connection_filter=inspect_connection):
            pass

        for topic, msg, recorded, connection in bag.read_messages(
                topics=allowed, return_connection_header=True):
            if topic in TF_TOPICS:
                for transform in msg.transforms:
                    stamp = transform.header.stamp.to_sec()
                    stamp = stamp if stamp > 0 else recorded.to_sec()
                    if stamp > anchor:
                        continue
                    key = (transform.header.frame_id, transform.child_frame_id)
                    old = transforms[topic].get(key)
                    if old is None or stamp >= old['time']:
                        one = type(msg)()
                        one.transforms = [transform]
                        entry = candidate(one, recorded, connection, 'latest_transform_at_or_before_image')
                        entry['time'] = stamp
                        entry['time_source'] = 'transform_header' if transform.header.stamp.to_sec() > 0 else 'bag_record'
                        transforms[topic][key] = entry
                continue
            if topic == camera_info_topic and (
                    msg.width != image.width or msg.height != image.height
                    or msg.header.frame_id != image.header.frame_id):
                continue
            # Inputs/events/settings are taken as-of the image; never borrow a future command.
            asof = (topic != camera_info_topic and (
                    topic in latched_topics or msg._type in ('geometry_msgs/Twist', 'std_msgs/Empty')
                    or msg._type.startswith('dynamic_reconfigure/')
                    or topic.endswith('/joy')))
            item = candidate(msg, recorded, connection,
                             'latest_at_or_before_image' if asof else 'nearest_sample')
            if asof:
                if recorded.to_sec() > anchor:
                    continue
                score = -recorded.to_sec()
            else:
                score = (abs(item['time']-anchor), item['time'])
            old = selected.get(topic)
            if old is None or score < old['score']:
                item['score'] = score
                selected[topic] = item

        manifest = {'source_bag': str(bag_path.resolve()), 'bag_start_time_s': start,
                    'bag_duration_s': end-start, 'requested_offset_s': offset,
                    'actual_image_offset_s': anchor-start,
                    'image_selection_error_s': anchor-target,
                    'image_topic': image_topic, 'camera_info_topic': camera_info_topic,
                    'image_header': {'secs': image.header.stamp.secs,
                                     'nsecs': image.header.stamp.nsecs,
                                     'frame_id': image.header.frame_id},
                    'excluded_topics': sorted(topic for topic in topics if excluded(topic)),
                    'topics_without_eligible_sample': sorted(
                        topic for topic in allowed if topic not in selected
                        and not transforms.get(topic)),
                    'topics': {}}
        arrays = {'rgb': CvBridge().imgmsg_to_cv2(image, 'rgb8').copy(),
                  'image_stamp': stamp_pair(image.header),
                  'camera_frame': np.array(image.header.frame_id),
                  'requested_offset_s': np.array(offset),
                  'actual_image_offset_s': np.array(anchor-start)}
        if camera_info_topic in selected:
            info = selected[camera_info_topic]['message']
            arrays.update(K=np.asarray(info.K).reshape(3,3), D=np.asarray(info.D),
                          R=np.asarray(info.R).reshape(3,3), P=np.asarray(info.P).reshape(3,4),
                          camera_info_stamp=stamp_pair(info.header),
                          distortion_model=np.array(info.distortion_model))
        for topic, entries in sorted(transforms.items()):
            if not entries:
                continue
            aggregate = type(next(iter(entries.values()))['message'])()
            aggregate.transforms = []
            details = []
            for key, item in sorted(entries.items()):
                aggregate.transforms.extend(item['message'].transforms)
                details.append(dict(summary(item, anchor, start),
                                    parent_frame=key[0], child_frame=key[1]))
            selected[topic] = {'message': aggregate, 'details': details}
        for index, (topic, item) in enumerate(sorted(selected.items())):
            message = item['message']
            key = 'topic_{:03d}'.format(index)
            arrays[key+'_data'] = serialize(message)
            if message._type not in BINARY_TYPES:
                arrays[key+'_yaml'] = np.array(roslib.message.strify_message(message))
            if 'details' in item:
                entry = {'type': message._type, 'md5sum': message._md5sum,
                         'message_definition': message._full_text,
                         'selection': 'latest_per_transform_at_or_before_image',
                         'transforms': item['details']}
            else:
                entry = summary(item, anchor, start)
            entry['data_key'] = key+'_data'
            if key+'_yaml' in arrays:
                entry['yaml_key'] = key+'_yaml'
            manifest['topics'][topic] = entry
            if topic == '/bebop/odom':
                position, rotation = message.pose.pose.position, message.pose.pose.orientation
                arrays['odom_position'] = np.array([position.x, position.y, position.z])
                arrays['odom_orientation_xyzw'] = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
                velocity = message.twist.twist.linear
                arrays['odom_linear_velocity'] = np.array([velocity.x, velocity.y, velocity.z])
            elif topic == '/bebop/joint_states':
                arrays['joint_names'] = np.asarray(message.name, dtype=str)
                arrays['joint_positions'] = np.asarray(message.position, dtype=float)
        arrays['snapshot_metadata_json'] = np.array(json.dumps(manifest))
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('xb') as stream:
            try:
                np.savez_compressed(stream, **arrays)
            except BaseException:
                stream.close()
                output.unlink()
                raise

    print('Saved:', output)
    print('Requested {:.6f} s; selected image {:.6f} s ({:+.6f} s)'.format(
        offset, anchor-start, anchor-target))
    print('Retained {} topics; excluded {} topics.'.format(
        len(manifest['topics']), len(manifest['excluded_topics'])))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--time', type=float, required=True, help='seconds from bag start')
    parser.add_argument('--output', type=Path, required=True, help='new .npz file')
    parser.add_argument('--image-topic', default='/bebop/image_raw')
    parser.add_argument('--camera-info-topic', default='/bebop/camera_info')
    args = parser.parse_args()
    try:
        extract(args.bag, args.time, args.output, args.image_topic, args.camera_info_topic)
    except (ValueError, OSError, rosbag.ROSBagException) as error:
        parser.exit(1, 'Extraction failed: {}\n'.format(error))


if __name__ == '__main__':
    main()
