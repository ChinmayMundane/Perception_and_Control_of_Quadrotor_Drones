# Bebop and Jackal Robot Simulation

This repository provides a ROS 1 and Gazebo simulation for Parrot Bepop and a Clearpath Jackal ground robot

You can first learn to fly the Bebop in an empty world and then launch a
world containing both the Bebop and Jackal.

## 1. Supported system

Use the following software versions:

| Software | Version |
| --- | --- |
| Operating system | Ubuntu 20.04 Desktop (64-bit) |
| ROS | ROS 1 Noetic |
| Simulator | Gazebo Classic 11 |
| Python | Python 3.8 |
| Build system | Catkin Tools |

Ubuntu 20.04 is strongly recommended. ROS Noetic officially targets Ubuntu
20.04, and this project is not designed for a native ROS installation on
Ubuntu 22.04 or 24.04.

If Ubuntu is not installed, download the
[Ubuntu 20.04.6 Desktop image](https://releases.ubuntu.com/focal/) and install it
on a computer.

## 2. Install ROS 1 Noetic

Open a terminal with `Ctrl+Alt+T`.

First install basic tools and enable Ubuntu's Universe repository:

```bash
sudo apt update
sudo apt install -y curl git software-properties-common
sudo add-apt-repository universe
```

Install the official ROS repository configuration package:

```bash
curl -L -o /tmp/ros-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros-apt-source_1.2.0.focal_all.deb
sudo apt install -y /tmp/ros-apt-source.deb
sudo apt update
```

Install ROS Noetic Desktop Full and the additional packages used by this
project:

```bash
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  python3-cvxopt \
  python3-pip \
  python3-rosdep \
  python3-scipy \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-hector-gazebo-plugins \
  ros-noetic-jackal-description \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-twist-mux
```

Initialize `rosdep`:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` says that rosdep is already initialized, continue with
`rosdep update`.

Check the ROS installation:

```bash
source /opt/ros/noetic/setup.bash
echo $ROS_DISTRO
```

Expected output:

```text
noetic
```

## 3. Download the project

Create the Catkin workspace, then clone this repository into its `src`
directory:

```bash
mkdir -p ~/bepop_ws
cd ~/bepop_ws
git clone https://github.com/ChinmayMundane/Perception_and_Control_of_Quadrotor_Drones.git src
```

The repository should now be located at `~/bepop_ws/src`. All simulator
packages are included directly, so no separate submodule download is needed.
Continue with the dependency installation and build steps below from
`~/bepop_ws`.

## 4. Install project dependencies

Install Open3D with Ubuntu 20.04's system Python 3.8:

```bash
/usr/bin/python3.8 -m pip install --user --upgrade "pip>=20.3,<25.1"
/usr/bin/python3.8 -m pip install --user "numpy<2.0" "open3d==0.18.0"
/usr/bin/python3.8 -c "import open3d as o3d; print(o3d.__version__)"
```

The final command should print `0.18.0`. Use `/usr/bin/python3.8` exactly as
shown. A Conda environment or another Python installation can make the plain
`python3` command select an unsupported Python version.

```bash
cd ~/bepop_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="hector_sensors_gazebo hector_sensors_description hector_trajectory_server hector_geotiff hector_mapping hector_gazebo_worlds"
```

The skipped packages belong to optional Hector examples and are not required
for the simulations in this guide.

## 5. Build the workspace

```bash
cd ~/bepop_ws
source /opt/ros/noetic/setup.bash
catkin init
catkin build
source devel/setup.bash
```

The build is successful when the summary reports zero failed packages.

## 6. Source the workspace

Every new terminal used for this project must run:

```bash
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
```

If a command reports `Resource not found` or `Package not found`, first check
that these two commands were run in the current terminal.


## 7. Fly the Bebop in an empty world

Use two terminals.

### Terminal 1: start Gazebo and RViz

```bash
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
roslaunch bebop_gazebo bebop_empty_world.launch
```

Wait until Gazebo displays the quadrotor and RViz displays the robot and lidar
point cloud. Initial loading can take 10-30 seconds.

The world contains only a ground plane. Concentric lidar rings on the ground
are normal: the rotating lidar's downward beams are measuring the flat ground
at different angles.

### Terminal 2: start Bebop keyboard control

```bash
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
rosrun bebop_teleop bebop_teleop.py
```

Click this terminal before pressing the flight keys.


## 8. PID controller student task

For the assignment instructions, see the
[Bebop PID controller tutorial](bebop_pid_controller/README.md).

## 9. Run the CBF quadrotor navigation example

The CBF example starts Gazebo, RViz, the selected world, the Bebop, its lidar,
and the navigation controller. It then waits for the simulator and commands
the Bebop to take off automatically.

Open a sourced terminal and run:

```bash
cd ~/bepop_ws/src
bash run_cbf_quadrotor_navigation.sh env4
```

Wait for Gazebo and RViz to finish loading and for the Bebop to take off. In
RViz, select **2D Nav Goal** and click a destination. The controller will move
the Bebop toward the goal while using the lidar measurements to keep a safe
distance from the simulated obstacles.

Replace `env4` with another included world name when needed:

```text
env1 through env13
```

Do not include the `.world` extension. If no world name is supplied, the
script uses `env4`:

```bash
bash run_cbf_quadrotor_navigation.sh
```

Press `Ctrl+C` in the script terminal to stop the controller, Gazebo, and
RViz. Do not run keyboard teleoperation at the same time because both programs
send velocity commands to the Bebop.

## 10. Launch the Bebop and Jackal together

This simulation starts:

- the Bebop quadrotor;
- the four-wheel Jackal ground robot;
- Gazebo; and
- one RViz window showing both robots and the Bebop lidar.

### Terminal 1: start the combined simulation

```bash
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
roslaunch bebop_jackal_sim bebop_jackal_empty_world.launch
```

The robots start two metres apart. Wait until both models appear before
sending commands.

### Terminal 2: drive the Jackal

```bash
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  cmd_vel:=/jackal/cmd_vel
```

### Jackal keyboard controls

| Key | Action |
| --- | --- |
| `i` | Drive forward |
| `,` | Drive backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `q` / `z` | Increase / decrease all speeds |
| `Ctrl+C` | Stop and exit |

Keep this terminal focused while driving.

### Terminal 3: fly the Bebop

```bash
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
rosrun bebop_teleop bebop_teleop.py
```

The programs use separate command topics, so each program controls only its
own robot. For a classroom exercise, focus and operate one teleoperation
terminal at a time.



## Troubleshooting

### `Resource not found: bebop_gazebo`

The package is included in this repository. Check that the repository was
cloned into `~/bepop_ws/src`, then rebuild and source the workspace:

```bash
cd ~/bepop_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

### `Package 'bebop_teleop' not found`

```bash
cd ~/bepop_ws
catkin build bebop_teleop
source devel/setup.bash
```

### `Resource not found: jackal_description`

```bash
sudo apt update
sudo apt install ros-noetic-jackal-description
source /opt/ros/noetic/setup.bash
source ~/bepop_ws/devel/setup.bash
```

### Open3D version error or `No matching distribution found`

Check which Python is selected:

```bash
command -v python3
python3 --version
/usr/bin/python3.8 --version
```

For Ubuntu 20.04 and ROS Noetic, install and test Open3D using the system
Python rather than a Conda Python:

```bash
/usr/bin/python3.8 -m pip install --user --upgrade "pip>=20.3,<25.1"
/usr/bin/python3.8 -m pip install --user "numpy<2.0" "open3d==0.18.0"
/usr/bin/python3.8 -c "import open3d as o3d; print(o3d.__version__)"
```

If `/usr/bin/python3.8 -m pip` is unavailable, install it first:

```bash
sudo apt update
sudo apt install python3-pip
```

### Gazebo opens with a black window or crashes

Update the graphics driver and make sure 3D acceleration is enabled. In a
virtual machine, software rendering can be used as a fallback:

```bash
LIBGL_ALWAYS_SOFTWARE=1 roslaunch bebop_gazebo bebop_empty_world.launch
```

### A robot does not respond to the keyboard

- Click the terminal running the appropriate keyboard-control program.
- Check that Gazebo is still running.
- For the Bebop, press `1` before sending movement commands.
- Press `k` to stop before switching between teleoperation terminals.
