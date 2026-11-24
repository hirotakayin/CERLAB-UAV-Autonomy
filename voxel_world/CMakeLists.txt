cmake_minimum_required(VERSION 3.0.2)
project(voxel_world)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs nav_msgs message_filters)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy sensor_msgs nav_msgs message_filters)

catkin_install_python(PROGRAMS
  scripts/voxelize_sdf.py
  scripts/compare_gazebo_scan.py
  scripts/replay_flight.py
  scripts/check_occmap_raycast.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
