cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/astar_follow_path_bridge.py
    scripts/astar_planner_node.py
    scripts/image_to_plane_marker.py
    scripts/pcd_to_grid_map.py
    scripts/pointcloud_to_occupancy_grid.py
    scripts/semantic_mask_editor.py
    scripts/send_goal_node.py
    scripts/send_pose_goal_node.py
    scripts/simple_omni_base_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
