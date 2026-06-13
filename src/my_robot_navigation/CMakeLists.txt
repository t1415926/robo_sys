cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/diff_drive_interface_bridge.py
    scripts/pointcloud_to_occupancy_grid.py
    scripts/semantic_mask_editor.py
    scripts/send_goal_node.py
    scripts/simple_omni_base_node.py
    scripts/sim_localization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
