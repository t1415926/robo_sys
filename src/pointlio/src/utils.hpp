#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <common_lib.h>
#include "parameters.h"
#include "Estimator.h"
#include "ikd-Tree/ikd_Tree.h"
#include "scan_aligner.h"
#include "use-ikfom.h"


extern bool find_yaw;
extern M4F init_pose_curr;
extern M4F init_pose_last;
extern M4F lidar_pose_last;

void set_init_pose(const V3F& poseT, const M3F& poseR)
{
  init_pose_last.setIdentity();
  init_pose_curr.setIdentity();
  init_pose_last.block<3, 3>(0, 0) = poseR;
  init_pose_last.block<3, 1>(0, 3) = poseT;
  init_pose_curr.block<3, 3>(0, 0) = poseR;
  init_pose_curr.block<3, 1>(0, 3) = poseT;
  lidar_pose_last=init_pose_curr;
}
