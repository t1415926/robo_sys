#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <algorithm>
#include <limits>
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

#include "utils.hpp"

#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;

int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false, flg_first_scan = true;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

// Time Log Variables
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool lidar_pushed = false, flg_reset = false, flg_exit = false;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

MeasureGroup Measures;

sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

auto logger = rclcpp::get_logger("laserMapping");

void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(logger, "catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
                kf_output.x_.gravity(2)); // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}


void pointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I.normalized() * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I.normalized() * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

int points_cache_size = 0;

void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void lasermap_fov_segment() {
    cub_needrm.clear();

    V3D pos_LiD;
    if (use_imu_as_input) {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    } else {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.sec;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    double time_div = get_time_sec(msg->header.stamp);
    p_pre->process(msg, ptr);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar; //get_time_sec(msg->header.stamp);
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);   
    double time_div = get_time_sec(msg->header.stamp);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_div_i = *ptr_div;
                // cout << "ptr div i num:" << ptr_div_i->size() << endl;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar; //get_time_sec(msg->header.stamp);
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
        //std::cout<<"lidar::"<<lidar_buffer.empty()<<"  add_lidar_data\n";
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
    publish_count++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        RCLCPP_ERROR(logger, "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void trim_lidar_backlog() {
    if (lidar_pushed || lidar_buffer.empty() || time_buffer.empty()) return;

    size_t dropped = 0;
    double first_dropped_time = 0.0;
    size_t max_buffer_size = max_lidar_buffer_size > 0 ? static_cast<size_t>(max_lidar_buffer_size) : 0;

    while (lidar_buffer.size() > 1 && time_buffer.size() > 1) {
        bool over_size = max_lidar_buffer_size > 0 && lidar_buffer.size() > max_buffer_size;
        bool over_time = max_lidar_buffer_time > 0.0 &&
                         (time_buffer.back() - time_buffer.front()) > max_lidar_buffer_time;

        if (!over_size && !over_time) break;

        if (dropped == 0) first_dropped_time = time_buffer.front();
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        dropped++;
    }

    if (dropped > 0) {
        static double last_warn_time = 0.0;
        double now = omp_get_wtime();
        if (now - last_warn_time > 1.0) {
            RCLCPP_WARN(logger,
                        "Dropped %zu stale LiDAR frames to control backlog; first dropped %.6f, latest kept %.6f, kept %zu frames",
                        dropped, first_dropped_time, time_buffer.back(), lidar_buffer.size());
            last_warn_time = now;
        }
    }
}

bool sync_packages(MeasureGroup &meas) {
    trim_lidar_backlog();

    if (!imu_en) {
        if (!lidar_buffer.empty()) {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            if (meas.lidar->points.size() < 1) {
                cout << "lose lidar" << std::endl;
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() < 1) {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points) {
            if (pt.curvature > end_time) {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    } else if (!init_map) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;

double rotation_angle_rad(const M3D &delta_rot) {
    double cos_angle = (delta_rot.trace() - 1.0) * 0.5;
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;
    return acos(cos_angle);
}

void get_current_map_update_state(V3D &pos, M3D &rot, V3D &vel, V3D &omega) {
    if (use_imu_as_input) {
        pos << kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2);
        rot = kf_input.x_.rot.normalized().toRotationMatrix();
        vel << kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2);
        omega << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
        omega -= V3D(kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));
    } else {
        pos << kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2);
        rot = kf_output.x_.rot.normalized().toRotationMatrix();
        vel << kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2);
        omega << kf_output.x_.omg(0), kf_output.x_.omg(1), kf_output.x_.omg(2);
    }
}

bool should_update_incremental_map(bool is_initialize) {
    if (!map_incremental_en) return false;
    if (!use_prior_map) return true;
    if (!is_initialize) return false;

    static bool prev_pose_ready = false;
    static bool last_update_pose_ready = false;
    static int map_update_frame_id = 0;
    static int last_update_frame_id = 0;
    static V3D prev_pos(Zero3d), last_update_pos(Zero3d);
    static M3D prev_rot(Eye3d), last_update_rot(Eye3d);

    V3D pos, vel, omega;
    M3D rot;
    get_current_map_update_state(pos, rot, vel, omega);
    map_update_frame_id++;

    auto remember_prev_pose = [&]() {
        prev_pos = pos;
        prev_rot = rot;
        prev_pose_ready = true;
    };

    if (!last_update_pose_ready) {
        last_update_pos = pos;
        last_update_rot = rot;
        last_update_frame_id = map_update_frame_id;
        last_update_pose_ready = true;
        remember_prev_pose();
        return false;
    }

    if (map_update_frame_interval > 0 &&
        map_update_frame_id - last_update_frame_id < map_update_frame_interval) {
        remember_prev_pose();
        return false;
    }

    if (map_update_min_effective_features > 0 && effct_feat_num < map_update_min_effective_features) {
        remember_prev_pose();
        return false;
    }

    if (map_update_max_vel > 0.0 && vel.norm() > map_update_max_vel) {
        remember_prev_pose();
        return false;
    }

    if (map_update_max_angular_vel > 0.0 && omega.norm() > map_update_max_angular_vel) {
        remember_prev_pose();
        return false;
    }

    if (prev_pose_ready) {
        double step_dist = (pos - prev_pos).norm();
        double step_angle = rotation_angle_rad(rot * prev_rot.transpose());
        double max_angle_rad = map_update_max_angle_deg * PI_M / 180.0;

        if (map_update_max_dist > 0.0 && step_dist > map_update_max_dist) {
            remember_prev_pose();
            return false;
        }
        if (map_update_max_angle_deg > 0.0 && step_angle > max_angle_rad) {
            remember_prev_pose();
            return false;
        }
    }

    double dist_from_last_update = (pos - last_update_pos).norm();
    double angle_from_last_update = rotation_angle_rad(rot * last_update_rot.transpose());
    double min_angle_rad = map_update_min_angle_deg * PI_M / 180.0;
    bool dist_gate_enabled = map_update_min_dist > 0.0;
    bool angle_gate_enabled = map_update_min_angle_deg > 0.0;
    bool changed_enough = (!dist_gate_enabled && !angle_gate_enabled) ||
                          (dist_gate_enabled && dist_from_last_update >= map_update_min_dist) ||
                          (angle_gate_enabled && angle_from_last_update >= min_angle_rad);

    if (!changed_enough) {
        remember_prev_pose();
        return false;
    }

    last_update_pos = pos;
    last_update_rot = rot;
    last_update_frame_id = map_update_frame_id;
    remember_prev_pose();
    return true;
}

void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            /* If the nearest points is definitely outside the downsample box */
            if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min ||
                fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min ||
                fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min) {
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                continue;
            }
            /* Check if there is a point already in the downsample box */
            float dist = calc_dist<float>(feats_down_world->points[i], mid_point);
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                /* Those points which are outside the downsample box should not be considered. */
                if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min &&
                    fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min &&
                    fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
        } else {
            // PointToAdd.emplace_back(feats_down_world->points[i]);
            PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        }
    }
    if (ikdtree.Root_Node == nullptr) {
        PointVector init_dynamic_points;
        init_dynamic_points.reserve(PointToAdd.size() + PointNoNeedDownsample.size());
        init_dynamic_points.insert(init_dynamic_points.end(), PointToAdd.begin(), PointToAdd.end());
        init_dynamic_points.insert(init_dynamic_points.end(), PointNoNeedDownsample.begin(), PointNoNeedDownsample.end());
        if (!init_dynamic_points.empty()) {
            ikdtree.set_downsample_param(filter_size_map_min);
            ikdtree.Build(init_dynamic_points);
        }
        return;
    }

    if (!PointToAdd.empty()) ikdtree.Add_Points(PointToAdd, true);
    if (!PointNoNeedDownsample.empty()) ikdtree.Add_Points(PointNoNeedDownsample, false);
}

void publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
    
    if (odom_only) {return;}

    KD_TREE<PointType> &publish_tree =
            (prior_map_loaded && prior_ikdtree.Root_Node != nullptr) ? prior_ikdtree : ikdtree;
    if (publish_tree.Root_Node == nullptr) {return;}

    int size_init_ikdtree = publish_tree.size();
    PointCloudXYZI::Ptr laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector().swap(publish_tree.PCL_Storage);
    publish_tree.flatten(publish_tree.Root_Node, publish_tree.PCL_Storage, NOT_RECORD);

    laserCloudInit->points = publish_tree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_header_frame_id;
    if (!odom_only) {
        pubLaserCloudFullRes->publish(laserCloudmsg);
    }
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {

    if (odom_only) {return;}

    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            // if (i % 3 == 0)
            // {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // 
            // }
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_header_frame_id;
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body) {

    if (odom_only) {return;}

    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

template<typename T>
void set_posestamp(T &out) {
    if (!use_imu_as_input) {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        out.orientation.x = kf_output.x_.rot.coeffs()[0];
        out.orientation.y = kf_output.x_.rot.coeffs()[1];
        out.orientation.z = kf_output.x_.rot.coeffs()[2];
        out.orientation.w = kf_output.x_.rot.coeffs()[3];
    } else {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        out.orientation.x = kf_input.x_.rot.coeffs()[0];
        out.orientation.y = kf_input.x_.rot.coeffs()[1];
        out.orientation.z = kf_input.x_.rot.coeffs()[2];
        out.orientation.w = kf_input.x_.rot.coeffs()[3];
    }
}

template<typename T>
void set_twist(T &out) {
    if (!use_imu_as_input) {
        out.linear.x = kf_output.x_.vel(0);
        out.linear.y = kf_output.x_.vel(1);
        out.linear.z = kf_output.x_.vel(2);
        out.angular.x = kf_output.x_.omg(0);
        out.angular.y = kf_output.x_.omg(1);
        out.angular.z = kf_output.x_.omg(2);
    } else {
        out.linear.x = kf_input.x_.vel(0);
        out.linear.y = kf_input.x_.vel(1);
        out.linear.z = kf_input.x_.vel(2);
        out.angular.x = imu_last.angular_velocity.x;
        out.angular.y = imu_last.angular_velocity.y;
        out.angular.z = imu_last.angular_velocity.z;
    }
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
                      std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_br) {

    odomAftMapped.header.frame_id = odom_header_frame_id;
    odomAftMapped.child_frame_id = odom_child_frame_id;

    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    } else {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    set_twist(odomAftMapped.twist.twist);

    if (odom_only){
        Matrix3d cov = kf_output.get_P().block<3, 3>(0, 0);

        // Get the position components (first 3x3)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                odomAftMapped.pose.covariance[6 * i + j] = cov(i, j);
            }
        }

        odomAftMapped.pose.covariance[21] = 0.0;    // Covariance for roll
        odomAftMapped.pose.covariance[28] = 0.0;    // Covariance for pitch
        odomAftMapped.pose.covariance[35] = 0.05;   // Covariance for yaw

        odomAftMapped.twist.covariance[0] = 0.1;    // Covariance for linear velocity on x
        odomAftMapped.twist.covariance[7] = 0.1;    // Covariance for linear velocity on y
        odomAftMapped.twist.covariance[14] = 0.0;   // Covariance for linear velocity on z
        odomAftMapped.twist.covariance[21] = 0.0;  // Covariance for angular velocity (roll)
        odomAftMapped.twist.covariance[28] = 0.0;  // Covariance for angular velocity (pitch)
        odomAftMapped.twist.covariance[35] = 0.05;  // Covariance for angular velocity (yaw)
    }

    pubOdomAftMapped->publish(odomAftMapped);

    //static tf2_ros::TransformBroadcaster br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = odom_header_frame_id;
    transform.child_frame_id = odom_child_frame_id;

    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;

    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

    transform.header.stamp = odomAftMapped.header.stamp;

    tf_br->sendTransform(transform);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath) {

    if (odom_only) {return;}

    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = odom_header_frame_id;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.header.stamp = msg_body_pose.header.stamp;
        path.header.frame_id = odom_header_frame_id;
        path.poses.emplace_back(msg_body_pose);
        if (path_pose_limit > 0 && path.poses.size() > static_cast<size_t>(path_pose_limit)) {
            size_t excess = path.poses.size() - static_cast<size_t>(path_pose_limit);
            path.poses.erase(path.poses.begin(), path.poses.begin() + excess);
        }
        pubPath->publish(path);
    }
}






bool find_yaw = false;
bool init_pose_seeded = false;
M4F init_pose_curr = M4F::Zero();
M4F init_pose_last = M4F::Zero();
M4F lidar_pose_last = M4F::Identity();
deque<PointCloudXYZI::Ptr> init_lidar_frame_cache;

static std::vector<float> make_search_range(const std::vector<double>& range, bool force_zero)
{
    std::vector<float> values;
    if (range.size() < 3) {
        values.emplace_back(0.0f);
        return values;
    }

    double range_min = range[0];
    double step = std::abs(range[1]);
    double range_max = range[2];
    if (range_min > range_max) std::swap(range_min, range_max);

    if (step < 1e-6) {
        values.emplace_back(static_cast<float>(range_min));
    } else {
        for (double v = range_min; v <= range_max + 0.5 * step; v += step) {
            values.emplace_back(static_cast<float>(v));
        }
    }

    if (force_zero && range_min <= 0.0 && range_max >= 0.0) {
        values.emplace_back(0.0f);
    }

    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(),
                             [](float a, float b) { return std::abs(a - b) < 1e-5f; }),
                 values.end());
    if (values.empty()) values.emplace_back(0.0f);
    return values;
}

static void voxel_filter_cloud(const PointCloudXYZI::Ptr& input,
                               PointCloudXYZI::Ptr& output,
                               double leaf_size)
{
    output.reset(new PointCloudXYZI());
    if (!input || input->empty()) return;

    if (leaf_size <= 1e-6) {
        *output = *input;
        return;
    }

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(input);
    voxel.filter(*output);
}

static bool prepare_init_accumulated_scan(const PointCloudXYZI::Ptr& current_scan,
                                          PointCloudXYZI::Ptr& fine_scan,
                                          PointCloudXYZI::Ptr& coarse_scan)
{
    if (!current_scan || current_scan->empty()) return false;

    const int required_frames = std::max(1, prior_init_accumulate_frames);
    PointCloudXYZI::Ptr frame_copy(new PointCloudXYZI(*current_scan));
    init_lidar_frame_cache.emplace_back(frame_copy);
    while (static_cast<int>(init_lidar_frame_cache.size()) > required_frames) {
        init_lidar_frame_cache.pop_front();
    }

    if (static_cast<int>(init_lidar_frame_cache.size()) < required_frames) {
        RCLCPP_INFO(logger, "Accumulating initialization scans: %zu/%d",
                    init_lidar_frame_cache.size(), required_frames);
        return false;
    }

    PointCloudXYZI::Ptr accumulated(new PointCloudXYZI());
    for (const auto& frame : init_lidar_frame_cache) {
        *accumulated += *frame;
    }

    voxel_filter_cloud(accumulated, fine_scan, prior_init_voxel_size);
    voxel_filter_cloud(fine_scan, coarse_scan, prior_init_search_voxel_size);

    return fine_scan && coarse_scan && !fine_scan->empty() && !coarse_scan->empty();
}

static void clear_init_accumulated_scan()
{
    init_lidar_frame_cache.clear();
}

bool pose_init(const PointCloudXYZI::Ptr& fine_scan, const PointCloudXYZI::Ptr& coarse_scan,
               M4F& init_translasion, KD_TREE<PointType>& kdtree, vector<double>& YAW_RANGE)
{
  std::cout << "begin to init pose " << std::endl;

  //先验位姿
  if (!init_pose_seeded) {
    set_init_pose(prior_T,prior_R);
    init_pose_seeded = true;
  }

  bool init_done = false;
  if (!fine_scan || fine_scan->empty()) return false;

  if (use_prior_map)
  {
    double t1 = omp_get_wtime();
    float error_min = std::numeric_limits<float>::max(), validP_max = 0.0;
    M4F prior_with_min_error = M4F::Identity();
    const int saved_max_iter = ScanAligner::max_iter;

    if (find_yaw)
    {
      M4F prior_with_yaw = lidar_pose_last;
      std::pair<float, float> result;
      ScanAligner::max_iter = std::max(1, prior_init_fine_max_iter);
      result = ScanAligner::init_ppicp_method(kdtree, fine_scan, prior_with_yaw);
      error_min = result.first;
      validP_max = result.second;
      prior_with_min_error = prior_with_yaw;
    }
    else if (!find_yaw)
    {
      const PointCloudXYZI::Ptr search_scan =
          (coarse_scan && !coarse_scan->empty()) ? coarse_scan : fine_scan;
      const M4F base_pose = lidar_pose_last;
      std::pair<float, float> result;

      ScanAligner::max_iter = std::max(1, prior_init_coarse_max_iter);
      M4F prior_direct = base_pose;
      result = ScanAligner::init_ppicp_method(kdtree, search_scan, prior_direct);
      error_min = result.first;
      validP_max = result.second;
      prior_with_min_error = prior_direct;

      if (validP_max > prior_init_valid_ratio_min)
      {
        ScanAligner::max_iter = std::max(1, prior_init_fine_max_iter);
        result = ScanAligner::init_ppicp_method(kdtree, fine_scan, prior_with_min_error);
        error_min = result.first;
        validP_max = result.second;
        find_yaw = validP_max > prior_init_valid_ratio_min;
      }

      if (!find_yaw)
      {
      const std::vector<float> yaw_values = make_search_range(YAW_RANGE, true);
      const std::vector<float> xy_values = make_search_range(xy_range, true);
      const int total_search =
          static_cast<int>(yaw_values.size() * xy_values.size() * xy_values.size());

      ScanAligner::max_iter = std::max(1, prior_init_coarse_max_iter);
#ifdef MP_EN
      omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
      for (int i = 0; i < total_search; i++)
      {
        const int yaw_idx = i % static_cast<int>(yaw_values.size());
        const int xy_idx = i / static_cast<int>(yaw_values.size());
        const int x_idx = xy_idx % static_cast<int>(xy_values.size());
        const int y_idx = xy_idx / static_cast<int>(xy_values.size());

        float error = 0.0, validP = 0.0;
        std::pair<float, float> search_result;
        M4F prior_with_yaw = M4F::Identity();
        M3F rotation_yaw = M3F::Zero();
        const float yaw = yaw_values[yaw_idx];
        rotation_yaw << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
        prior_with_yaw.block<3, 1>(0, 3) = base_pose.block<3, 1>(0, 3);
        prior_with_yaw(0, 3) += xy_values[x_idx];
        prior_with_yaw(1, 3) += xy_values[y_idx];
        prior_with_yaw.block<3, 3>(0, 0) = rotation_yaw * base_pose.block<3, 3>(0, 0);

        search_result = ScanAligner::init_ppicp_method(kdtree, search_scan, prior_with_yaw);
        error = search_result.first;
        validP = search_result.second;

#ifdef MP_EN
#pragma omp critical
#endif
        {
          if (error < error_min)
          {
            error_min = error;
            prior_with_min_error = prior_with_yaw;
            validP_max = validP;
          }
        }
      }

      if (validP_max > prior_init_valid_ratio_min)
      {
        ScanAligner::max_iter = std::max(1, prior_init_fine_max_iter);
        result = ScanAligner::init_ppicp_method(kdtree, fine_scan, prior_with_min_error);
        error_min = result.first;
        validP_max = result.second;
        find_yaw = validP_max > prior_init_valid_ratio_min;
      }
      }
    }
    ScanAligner::max_iter = saved_max_iter;

    if (validP_max <= prior_init_valid_ratio_min) {
      std::cout << "init pose rejected by valid ratio: " << validP_max << std::endl;
      return false;
    }

    lidar_pose_last = prior_with_min_error;
    init_pose_curr.block<3, 3>(0, 0) = prior_with_min_error.block<3, 3>(0, 0) * Lidar_R_wrt_IMU.inverse().cast<float>();
    init_pose_curr.block<3, 1>(0, 3) =
        prior_with_min_error.block<3, 1>(0, 3) - init_pose_curr.block<3, 3>(0, 0) * Lidar_T_wrt_IMU.cast<float>();

    double t2 = omp_get_wtime();
   
    std::cout << "Init align time cost " << t2 - t1 << "s. " << std::endl
              << "Current pos:  " << std::endl
              << init_pose_curr.block<3, 1>(0, 3) << std::endl
              << "Current rot:  " << std::endl
              << init_pose_curr.block<3, 3>(0, 0) << std::endl
              << "Last pos:  " << std::endl
              << init_pose_last.block<3, 1>(0, 3) << std::endl
              << "Last rot:  " << std::endl
              << init_pose_last.block<3, 3>(0, 0) << std::endl
              << std::endl;

    V3F delta_rvec, delta_tvec;
    delta_rvec = rotationToEulerAngles(init_pose_curr.block<3, 3>(0, 0) * init_pose_last.block<3, 3>(0, 0).inverse());
    delta_tvec = init_pose_curr.block<3, 1>(0, 3) - init_pose_last.block<3, 1>(0, 3);
    // fout_init << "delta_tvec: " << delta_tvec.norm() << ", "
    //           << "delta_rvec: " << delta_rvec.norm() << std::endl;
    if (delta_rvec.norm() < prior_init_rotation_converge &&
        delta_tvec.norm() < prior_init_translation_converge &&
        find_yaw)
    {
      init_done = true;
    }
  }
  if(init_done){
    std::cout << "init pose sucess" << std::endl;
    init_translasion=init_pose_curr;
    return true;
  }
  std::cout << "init pose last change" << std::endl;
  init_pose_last = init_pose_curr;
  return false;
}




// 替代 Sophus::SO3f::hat() 的函数
inline Eigen::Matrix3f hat_operator(const Eigen::Vector3f& v) {
    Eigen::Matrix3f m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// 在调用前转换点云类型
pcl::PointCloud<pcl::PointXYZI>::Ptr convert_cloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    out_cloud->resize(in_cloud->size());
    
    #pragma omp parallel for
    for (size_t i = 0; i < in_cloud->size(); i++) {
        auto& p_in = in_cloud->points[i];
        auto& p_out = out_cloud->points[i];
        p_out.x = p_in.x;
        p_out.y = p_in.y;
        p_out.z = p_in.z;
        p_out.intensity = p_in.intensity;
    }
    
    return out_cloud;
}
float extract_yaw_from_rotation_matrix(const Eigen::Matrix3f& R) {
    // 确保矩阵是有效的旋转矩阵
    if (std::abs(R.determinant() - 1.0f) > 1e-3) {
        std::cerr << "Warning: Not a valid rotation matrix! Determinant = " 
                  << R.determinant() << std::endl;
    }
    
    // 提取 Yaw 角度（绕 Z 轴的旋转）
    float yaw = std::atan2(R(1, 0), R(0, 0));
    
    // 将角度标准化到 [-π, π] 范围
    if (yaw > M_PI) yaw -= 2 * M_PI;
    if (yaw < -M_PI) yaw += 2 * M_PI;
    
    return yaw;
}

using PointCloud = pcl::PointCloud<pcl::PointXYZINormal>;
using PointType = pcl::PointXYZINormal;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");
    readParameters(nh);
    cout << "lidar_type: " << lidar_type << endl;

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = odom_header_frame_id;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;

    /*** initialize variables ***/
    bool is_initialize=false;
    int is_initialize_times=0;

    // state_input in_initialize_state;
    // state_output out_initialize_state ;
    Eigen::Matrix<double, 24, 1> in_initialize_state = Eigen::Matrix<double, 24, 1>::Zero();
    Eigen::Matrix<double, 30, 1> out_initialize_state = Eigen::Matrix<double, 30, 1>::Zero();
    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    kf_input.change_P(P_init);
    
    
    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
    
    
    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox_ = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
    } else {
    sub_pcl = nh->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
     }
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200, imu_cbk);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_body;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    if (!odom_only){
        pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_registered", 100);
        pubLaserCloudFullRes_body = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_registered_body", 100);
        pubLaserCloudEffect = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/cloud_effected", 100);
        pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>
                ("/Laser_map", 100);
        pubPath = nh->create_publisher<nav_msgs::msg::Path>
                ("/path", 100);
    }

    // Choose topic name depending on odom_only value
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    if (odom_only){
        pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
                ("/odom_corrected", 100);
    } else {
        pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
                ("/aft_mapped_to_init", 100);
    }

    //auto plane_pub = nh->create_publisher<visualization_msgs::msg::Marker>
    //        ("/planner_normal", 1000);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    rclcpp::Rate rate(2000);

    double start_time,end_time;
    int loop_mark=0;
    start_time=omp_get_wtime();
    while (rclcpp::ok()) {
        if (flg_exit) break;
        loop_mark++;
        //ros::spinOnce();
        executor.spin_some(); // 处理当前可用的回调

        if (sync_packages(Measures)) {       //累积一帧数据
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                cout << "first lidar time" << first_lidar_time << endl;
            }

            if (flg_reset) {
                RCLCPP_WARN(logger, "reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            double t0, t1, t2, t3, t4, t5, match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, feats_undistort);

            if (feats_undistort->empty() || feats_undistort == nullptr) {
                continue;
            }
            if (imu_en) {
                if (!p_imu->gravity_align_) {
                    while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp)) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        // imu_deque.pop();
                    }
                    if (non_station_start) {
                        state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc *= -1;
                    } else {
                        state_in.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                        state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                        state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                    }
                    if (gravity_align) {
                        Eigen::Matrix3d rot_init;
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                        p_imu->Set_init(state_in.gravity, rot_init);
                        state_in.gravity = state_out.gravity = p_imu->gravity_;
                        state_in.rot = state_out.rot = rot_init;
                        state_in.rot.normalize();
                        state_out.rot.normalize();
                        state_out.acc = -rot_init.transpose() * state_out.gravity;
                    }
                    kf_input.change_x(state_in);
                    kf_output.change_x(state_out);
                }
            } else {
                if (!p_imu->gravity_align_) {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }
            }
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            time_seq = time_compressing<int>(feats_down_body);
            feats_down_size = feats_down_body->points.size();

            if(!is_initialize && prior_map_loaded && init_map){
                std::cout<<"\n        initialize        \n";

                M4F init_pose=M4F::Zero();
                PointCloudXYZI::Ptr init_fine_scan(new PointCloudXYZI());
                PointCloudXYZI::Ptr init_coarse_scan(new PointCloudXYZI());
                if(!prepare_init_accumulated_scan(feats_undistort, init_fine_scan, init_coarse_scan)){
                   continue;
                }
                if(!pose_init(init_fine_scan,init_coarse_scan,init_pose,prior_ikdtree,yaw_range)){
                   continue;
                }
                is_initialize=true;
                clear_init_accumulated_scan();

                //加入初始位姿

                // 计算位置增量
                in_initialize_state.segment<3>(0) = init_pose.block<3, 1>(0, 3).cast<double>();

                SO3 target_so3(init_pose.block<3, 3>(0, 0).cast<double>());
                target_so3.normalize();
                SO3 delta_rot = target_so3;
                in_initialize_state.segment<3>(3) = MTK::SO3<double>::log(delta_rot);

                // 计算位置增量
                out_initialize_state.segment<3>(0) = init_pose.block<3, 1>(0, 3).cast<double>();
                out_initialize_state.segment<3>(3) = MTK::SO3<double>::log(delta_rot);

                kf_output.x_.boxplus(out_initialize_state);
                kf_input.x_.boxplus(in_initialize_state);

                if (!publish_odometry_without_downsample) {
                    publish_odometry(pubOdomAftMapped, tf_broadcaster);
                }
                RCLCPP_WARN(logger,"\n        initialize done        \n");
                RCLCPP_WARN(logger,"\n        initialize done: %f %f %f      \n",kf_output.x_.pos(0),kf_output.x_.pos(1),kf_output.x_.pos(2));
                cout<<"\n"<<kf_output.x_.rot.normalized()<<"initialize done: \n";
                cout<<"\n"<<kf_output.x_.pos<<"\n";
                continue;
            }

            /*** initialize the map kdtree ***/
            if (!init_map) {

                // 优先尝试载入先验地图
                if (use_prior_map) {
                    pcl::PointCloud<PointType>::Ptr prior_cloud(new pcl::PointCloud<PointType>());
                    
                    // 从文件加载先验地图
                    if (pcl::io::loadPCDFile<PointType>(prior_map_path, *prior_cloud) == 0) 
                    {
                        const int prior_map_raw_size = static_cast<int>(prior_cloud->size());
                        if (downsample_prior_map && !prior_cloud->empty()) {
                            pcl::VoxelGrid<PointType> prior_map_filter;
                            pcl::PointCloud<PointType>::Ptr prior_cloud_downsampled(new pcl::PointCloud<PointType>());
                            prior_map_filter.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
                            prior_map_filter.setInputCloud(prior_cloud);
                            prior_map_filter.filter(*prior_cloud_downsampled);
                            prior_cloud.swap(prior_cloud_downsampled);
                        }
                        prior_ikdtree.Build(prior_cloud->points);
                        prior_map_loaded = true;
                        ikdtree.set_downsample_param(filter_size_map_min);
                        init_map = true;
                        
                        if (downsample_prior_map) {
                            RCLCPP_WARN(logger, "Prior map loaded successfully with %d points, downsampled to %d points",
                                        prior_map_raw_size, static_cast<int>(prior_cloud->size()));
                        } else {
                            RCLCPP_WARN(logger, "Prior map loaded successfully with %d points without downsampling",
                                        static_cast<int>(prior_cloud->size()));
                        }
                        publish_init_kdtree(pubLaserCloudMap);
                        continue;
                    }
                    else 
                    {
                        RCLCPP_WARN(logger,"Failed to load prior map, switching to online initialization");
                        use_prior_map = false;
                    }
                }


                if (ikdtree.Root_Node == nullptr) //
                    // if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                }

                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if (init_feats_world->size() < init_map_size) continue;
                ikdtree.Build(init_feats_world->points);
                init_map = true;
                publish_init_kdtree(pubLaserCloudMap); //(pubLaserCloudFullRes);
                continue;
            }
            

            
            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            crossmat_list.resize(feats_down_size);
            pbody_list.resize(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);

            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(feats_down_body->points[i].x,
                               feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                /*坐标系转换*/
                if (extrinsic_est_en) {
                    if (!use_imu_as_input) {
                        point_this = kf_output.x_.offset_R_L_I.normalized() * point_this + kf_output.x_.offset_T_L_I;
                    } else {
                        point_this = kf_input.x_.offset_R_L_I.normalized() * point_this + kf_input.x_.offset_T_L_I;
                    }
                } else {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);   //反对称矩阵
                crossmat_list[i] = point_crossmat;
            }

            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** point by point update ****/

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame) {
                        if (imu_en) {
                            while (time_current > get_time_sec(imu_next.header.stamp)) {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                                // imu_deque.pop();
                            }

                            angvel_avr
                                    << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr
                                    << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if (imu_en) {
                        bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        while (imu_comes) {
                            imu_upda_cov = true;
                            angvel_avr
                                    << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr
                                    << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = get_time_sec(imu_last.header.stamp); // big problem
                            imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            // if (!imu_comes)
                            {
                                double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;

                                if (dt_cov > 0.0) {
                                    time_update_last = get_time_sec(imu_last.header.stamp);
                                    double propag_imu_start = omp_get_wtime();

                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    kf_output.update_iterated_dyn_share_IMU();
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                        }
                    }

                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    // if(k == 0)
                    // {
                    //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " " << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
                    // }

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    if (prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (!imu_en && (dt_cov >= imu_time_inte)) // (point_cov_not_prop && imu_prop_cov)
                        {
                            double propag_cov_start = omp_get_wtime();
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            imu_upda_cov = false;
                            time_update_last = time_current;
                            propag_time += omp_get_wtime() - propag_cov_start;
                        }
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample) {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                     << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                     << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                     << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }

                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                    // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                }
            } else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    /*如果是第一帧 首先过滤不在时间区间内的数据 得到对应的角速度 和 线加速度*/
                    if (is_first_frame) {
                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // imu_deque.pop();
                        }
                        imu_prop_cov = true;
                        // imu_upda_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current;
                        // if(prop_at_freq_of_imu)
                        {
                            input_in.gyro << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;

                            input_in.acc << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                            // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
                            //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
                            //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

                            // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
                            //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
                            // 0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);

                            // angvel_avr -= state.bias_g;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }


                    /*不是第一帧 一样 获取区间内的数据*/

                    while (time_current > get_time_sec(imu_next.header.stamp)) // && !imu_deque.empty())
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro
                                << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc
                                << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                        
                        /*迭代 滤波器*/
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        double dt = get_time_sec(imu_last.header.stamp) - t_last;

                        // if(!prop_at_freq_of_imu)
                        // {       
                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = get_time_sec(imu_last.header.stamp); //time_current;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);
                        t_last = get_time_sec(imu_last.header.stamp);
                        imu_prop_cov = true;
                        // imu_upda_cov = true;
                    }

                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();

                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);

                    propag_time += omp_get_wtime() - propag_start;


                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample) {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                     << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                     << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }
            }
            // if(is_initialize){ //应该不需要持续增加
            //     kf_output.x_.boxplus(out_initialize_state);
            //     kf_input.x_.boxplus(in_initialize_state);
            // }
            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample) {
                publish_odometry(pubOdomAftMapped, tf_broadcaster);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();

            if (feats_down_size > 4 && should_update_incremental_map(is_initialize)) {
                map_incremental();
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);

            /*** Debug variables Logging ***/
            if (runtime_pos_log) {
                frame_num++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                { aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num; }
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
                       t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                       aver_time_propag);
                if (!publish_odometry_without_downsample) {
                    if (!use_imu_as_input) {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                 << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                 << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                 << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    } else {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                 << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                 << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        
        if(loop_mark>=1000){
            loop_mark=0;
        end_time=omp_get_wtime();
        std::cout<<"loop_fps:"<<1000/(end_time-start_time)<<"hz\n";
        start_time=omp_get_wtime();
        }
        rate.sleep();
    }
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        cout << "saved_DIR" << string(string(ROOT_DIR) + "PCD/") + file_name << endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
