#!/bin/bash
WORLD_NAME=${1:-env4}
echo "Using world: $WORLD_NAME"
roslaunch cbf bebop_obs_world.launch world_name:=$WORLD_NAME > /dev/null 2>&1 &  
ROSLAUNCH_PID=$!

cleanup() {
  echo "Shutting down..."
  kill $ROSLAUNCH_PID

  echo "Waiting for Gazebo to exit..."
  wait $ROSLAUNCH_PID

  echo "Exiting."
  exit 0
}

trap cleanup SIGINT

sleep 10

rostopic pub /bebop/takeoff std_msgs/Empty "{}" --once > /dev/null 2>&1

sleep 1

roslaunch cbf cbf.launch

cleanup
