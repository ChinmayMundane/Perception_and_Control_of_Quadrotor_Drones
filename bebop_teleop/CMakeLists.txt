cmake_minimum_required(VERSION 3.0.2)
project(bebop_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bebop_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
