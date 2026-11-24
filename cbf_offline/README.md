# CBF offline exercise

Use ROS Noetic with Python 3, NumPy and cv_bridge.

## 1. Download the recording

Download [cbf.bag](https://owncloud.ut.ee/owncloud/s/mzrF2eSDTjtXsLP) and place it in `~/bebop_drone_ws/src/cbf_offline`.

## 2. Explore the recording (optional)

With a ROS master running, play the recording:

```bash
source /opt/ros/noetic/setup.bash
rosbag play --clock --loop ~/bebop_drone_ws/src/cbf_offline/cbf.bag
```

Inspect topics from another terminal:

```bash
source /opt/ros/noetic/setup.bash
rostopic list
rostopic info /bebop/image_raw
rostopic info /bebop/camera_info
```

## 3. Extract a snapshot

```bash
cd ~/bebop_drone_ws/src/cbf_offline
source /opt/ros/noetic/setup.bash
python3 extract_data.py cbf.bag --time 11.5 --output data1.npz
```

Change `--time` to any offset in seconds from the bag start. Choose a new output filename each time. Extraction reads the bag directly; playback and a ROS master are not required.

The NPZ contains the RGB image, camera calibration, timestamps, available odometry/joint arrays, and other selected topics. `snapshot_metadata_json` contains metadata whose `topics` field maps each topic to its raw-data/YAML keys and records its time difference from the image.


Your task is to extract the information using given script (specifically, images at any time instant).then do depth estimation, plot the point cloud from depth estimation in rviz and show how the cbf velocity looks at those specific instants.
