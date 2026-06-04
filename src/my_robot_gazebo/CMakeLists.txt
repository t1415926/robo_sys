cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gazebo_omni_base_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
