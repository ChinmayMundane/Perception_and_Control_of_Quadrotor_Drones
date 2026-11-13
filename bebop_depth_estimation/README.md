# Depth Anything V2 Metric Indoor Small

Requires Ubuntu 20.04, ROS Noetic, Python 3.8, and the `bebop_drone_ws` workspace.
The indoor model is supplied in `src/bebop_depth_estimation/models/metric_indoor`;
no separate model download is needed.

## 1. Pull the latest code and build

```bash
cd ~/bebop_drone_ws/src
git pull --ff-only
cd ..
source /opt/ros/noetic/setup.bash
catkin build bebop_depth_estimation --no-deps
```

## 2. One-time environment setup

Skip this section if your `.venv-depth` environment already has these dependencies.

```bash
cd ~/bebop_drone_ws
sudo apt install python3-venv python3-opencv ros-noetic-cv-bridge \
  ros-noetic-tf2-ros ros-noetic-robot-state-publisher ros-noetic-xacro
python3 -m venv --system-site-packages .venv-depth
source .venv-depth/bin/activate
python -m pip install --upgrade 'pip<25.1'
```

Install **one** PyTorch build.

**Without an NVIDIA GPU:**

```bash
python -m pip install --upgrade 'torch==2.4.1+cpu' 'torchvision==0.19.1+cpu' \
  --index-url https://download.pytorch.org/whl/cpu
```

**With a compatible NVIDIA GPU and working NVIDIA driver:**

```bash
python -m pip install --upgrade 'torch==2.4.1+cu121' 'torchvision==0.19.1+cu121' \
  --index-url https://download.pytorch.org/whl/cu121
```

Then install the shared dependencies:

```bash
python -m pip install -r src/bebop_depth_estimation/requirements.txt
```

## 3. Run

Start the Bebop driver first. Then, in another terminal:

```bash
cd ~/bebop_drone_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bebop_depth_estimation depth_2d.launch \
  python:="$HOME/bebop_drone_ws/.venv-depth/bin/python"
```

The default device is `auto`: CUDA when available, otherwise CPU.

Depth image: `/bebop/depth/image`. 2D point cloud: `/bebop/depth/points_2d` (`base_link`).
