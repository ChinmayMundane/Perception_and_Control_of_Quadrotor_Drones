cmake_minimum_required(VERSION 3.0.2)
project(bebop_depth_estimation)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/depth_to_cloud.py
  scripts/prepare_depth_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md requirements.txt requirements-cpu.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
