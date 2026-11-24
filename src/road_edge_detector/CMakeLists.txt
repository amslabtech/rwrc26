cmake_minimum_required(VERSION 3.0.2)
project(road_edge_detector)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)

# src/road_edge_detector を Python パッケージとして使えるようにする
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/road_edge_detector_node.py
  scripts/evaluate_bag.py
  scripts/check_setup.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
