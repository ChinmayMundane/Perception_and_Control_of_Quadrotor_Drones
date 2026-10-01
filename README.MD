# Perception and Control of Quadrotor Drones (LOTI.05.091)

Repository associated with the course **Perception and Control of Quadrotor Drones (LOTI.05.091)**.

---
## Installation

**Note:** This code has only been tested with ROS-Noetic on Ubuntu 20.04. It is assumed that you already have a complete ROS installation.

### 1. Install required packages

```bash
sudo apt-get install libavahi-client-dev
sudo apt install ros-noetic-message-to-tf
sudo apt install ros-noetic-twist-mux
sudo apt install ros-noetic-pointcloud-to-laserscan
```

### 2. Create a new catkin workspace

```bash
cd
mkdir -p bebop_drone_ws/src
cd bebop_drone_ws
catkin init
```

### 3. Clone the repository
```bash
cd bebop_drone_ws/src
git clone https://github.com/ChinmayMundane/Perception_and_Control_of_Quadrotor_Drones.git -b bebop_drone .
```

### 4. Build the workspace

```bash
cd ~/bebop_drone_ws
catkin build
source devel/setup.bash
```

### 5. Set up environment variable

Add the following line to your `.bashrc` (replace `path to your bebop_drone_ws` with the actual path):

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{path_to_your_bebop_drone_ws}/devel/lib/parrot_arsdk
``` 
---
## Usage Instructions
### **1. Connect to motion capture (mocap) system via Ethernet and then run the VRPN client launch file**

```bash
source devel/setup.bash  # only run once in a new terminal
roslaunch vrpn_client_ros sample.launch server:=192.168.1.141
```

### **2. Connect to Bebop via Wi-Fi and then launch the driver**

```bash
source devel/setup.bash  # only run once in a new terminal
roslaunch bebop_driver bebop_with_vel_controller.launch
```

### **3. Take off & Land**

```bash
rostopic pub /bebop/takeoff std_msgs/Empty "{}"
```

**IMPORTANT:** Keep the following ready in a new terminal for emergency landing:

```bash
rostopic pub /bebop/land std_msgs/Empty "{}"
```

### 4. **Control Bebop drone**
**IMPORTANT:** For safety reasons, the velocity command given to the bebop should be clipped. The recommended value is 0.1 - 0.3. 

4.1. Send direct control command to Bebop cmd topic
```bash
rostopic pub -r 10 /bebop/velocity geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

4.2. Control with PID controller in Python

Your task today is to complete the TODOs in the file `your_controller/scripts/pid.py`. After finishing, run the launch file below to control the Bebop drone.

Launch the controller node:
```bash
source devel/setup.bash
roslaunch your_controller bebop_rviz_pid.launch
```

This launch file starts:
- the PID node from `your_controller/scripts/pid.py`
- RViz with the configuration file `your_controller/rviz/bebop.rviz`

