// #ifndef PARAM_H
// #define PARAM_H
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include <string>
#include "preprocess.h"
#include <common_lib.h>
#include <pcl_conversions/pcl_conversions.h>
extern bool odom_only;
extern std::string odom_header_frame_id;
extern std::string odom_child_frame_id;

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;

extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern bool map_incremental_en;
extern int init_map_size, con_frame_num;
extern int max_lidar_buffer_size, path_pose_limit;
extern int map_update_frame_interval, map_update_min_effective_features;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern double max_lidar_buffer_time;
extern double map_update_min_dist, map_update_max_dist;
extern double map_update_min_angle_deg, map_update_max_angle_deg;
extern double map_update_max_vel, map_update_max_angular_vel;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len;
extern float DET_RANGE;
extern bool imu_en, gravity_align, non_station_start;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern std::vector<double> yaw_range;
extern std::vector<double> xy_range;
extern bool runtime_pos_log, pcd_save_en, path_en;
extern bool scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern double time_lag_imu_to_lidar;
// 新增部分：载入先验地图的全局变量
extern bool use_prior_map; // 通过配置文件控制的开关
extern bool downsample_prior_map; // 是否在加载先验地图时体素降采样
extern std::string prior_map_path; // 先验地图文件路径
extern vector<float> priorT;
extern vector<float> priorR;
//初始化位姿态
extern V3F prior_T;
extern M3F prior_R;
extern int prior_init_accumulate_frames;
extern int prior_init_coarse_max_iter;
extern int prior_init_fine_max_iter;
extern double prior_init_voxel_size;
extern double prior_init_search_voxel_size;
extern double prior_init_valid_ratio_min;
extern double prior_init_translation_converge;
extern double prior_init_rotation_converge;
void readParameters(shared_ptr<rclcpp::Node> &nh);
