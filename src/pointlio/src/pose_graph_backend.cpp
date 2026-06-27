#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "scan_aligner.h"

namespace {

double yawFromPose(const gtsam::Pose3 &pose) {
  const auto rpy = pose.rotation().rpy();
  return rpy.z();
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

gtsam::Pose3 odomToPose3(const nav_msgs::msg::Odometry &odom) {
  const auto &p = odom.pose.pose.position;
  const auto &q = odom.pose.pose.orientation;
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
      gtsam::Point3(p.x, p.y, p.z));
}

geometry_msgs::msg::Pose pose3ToMsg(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Pose msg;
  const auto t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

Eigen::Matrix4f pose3ToEigen(const gtsam::Pose3 &pose) {
  Eigen::Matrix4d matrix = pose.matrix();
  return matrix.cast<float>();
}

M4F pose3ToM4F(const gtsam::Pose3 &pose) {
  Eigen::Matrix4d matrix = pose.matrix();
  return matrix.cast<float>();
}

gtsam::Pose3 m4fToPose3(const M4F &matrix) {
  Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d trans = matrix.block<3, 1>(0, 3).cast<double>();
  return gtsam::Pose3(gtsam::Rot3(rot), gtsam::Point3(trans));
}

gtsam::Pose3 eigenToPose3(const Eigen::Matrix4f &matrix) {
  Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d trans = matrix.block<3, 1>(0, 3).cast<double>();
  return gtsam::Pose3(gtsam::Rot3(rot), gtsam::Point3(trans));
}

PointCloudXYZI::Ptr downsampleCloud(const PointCloudXYZI::ConstPtr &cloud, double voxel_size) {
  PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
  if (!cloud || cloud->empty()) return filtered;
  if (voxel_size <= 1e-4) {
    *filtered = *cloud;
    return filtered;
  }
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(cloud);
  voxel.filter(*filtered);
  return filtered;
}

}  // namespace

struct Keyframe {
  int id = 0;
  rclcpp::Time stamp;
  gtsam::Pose3 odom_pose;
  gtsam::Pose3 optimized_pose;
  PointCloudXYZI::Ptr cloud;
};

class PoseGraphBackend : public rclcpp::Node {
 public:
  PoseGraphBackend() : Node("point_lio_pose_graph_backend") {
    declareParams();
    readParams();
    loadPriorMap();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 100,
        std::bind(&PoseGraphBackend::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PoseGraphBackend::cloudCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("pose_graph/path", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("pose_graph/odom", 20);
    prior_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pose_graph/prior_local_map", 2);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(get_logger(), "Pose graph backend started. odom=%s cloud=%s map=%s odom_frame=%s",
                odom_topic_.c_str(), cloud_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str());
  }

 private:
  void declareParams() {
    declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "camera_init");
    declare_parameter<double>("keyframe.min_dist", 0.5);
    declare_parameter<double>("keyframe.min_angle_deg", 8.0);
    declare_parameter<double>("keyframe.min_time", 0.5);
    declare_parameter<int>("keyframe.max_count", 2000);
    declare_parameter<double>("cloud.voxel_size", 0.25);
    declare_parameter<int>("cloud.min_points", 80);
    declare_parameter<bool>("init.enabled", true);
    declare_parameter<int>("init.accumulate_frames", 3);
    declare_parameter<double>("init.voxel_size", 0.20);
    declare_parameter<double>("init.local_radius", 20.0);
    declare_parameter<double>("init.icp_max_corr", 1.5);
    declare_parameter<int>("init.icp_max_iter", 40);
    declare_parameter<double>("init.fitness_max", 0.40);
    declare_parameter<int>("init.coarse_max_iter", 3);
    declare_parameter<int>("init.fine_max_iter", 15);
    declare_parameter<double>("init.valid_ratio_min", 0.65);
    declare_parameter<double>("init.plane_dist", 0.10);
    declare_parameter<std::vector<double>>("init.prior_T", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("init.xy_range", {-0.5, 0.25, 0.5});
    declare_parameter<std::vector<double>>("init.yaw_range", {-0.3, 0.1, 0.3});
    declare_parameter<bool>("prior.enabled", true);
    declare_parameter<std::string>("prior.map_path", "");
    declare_parameter<double>("prior.map_voxel_size", 0.30);
    declare_parameter<double>("prior.local_radius", 15.0);
    declare_parameter<double>("prior.icp_max_corr", 1.0);
    declare_parameter<int>("prior.icp_max_iter", 30);
    declare_parameter<double>("prior.fitness_max", 0.30);
    declare_parameter<int>("prior.add_every_n_keyframes", 3);
    declare_parameter<double>("prior.noise_xyz", 0.20);
    declare_parameter<double>("prior.noise_rpy", 0.10);
    declare_parameter<bool>("loop.enabled", true);
    declare_parameter<int>("loop.min_separation", 30);
    declare_parameter<double>("loop.search_radius", 2.0);
    declare_parameter<double>("loop.icp_max_corr", 1.0);
    declare_parameter<int>("loop.icp_max_iter", 30);
    declare_parameter<double>("loop.fitness_max", 0.20);
    declare_parameter<int>("loop.add_every_n_keyframes", 5);
    declare_parameter<double>("loop.noise_xyz", 0.15);
    declare_parameter<double>("loop.noise_rpy", 0.08);
  }

  void readParams() {
    get_parameter("odom_topic", odom_topic_);
    get_parameter("cloud_topic", cloud_topic_);
    get_parameter("map_frame", map_frame_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("keyframe.min_dist", keyframe_min_dist_);
    get_parameter("keyframe.min_angle_deg", keyframe_min_angle_deg_);
    get_parameter("keyframe.min_time", keyframe_min_time_);
    get_parameter("keyframe.max_count", keyframe_max_count_);
    get_parameter("cloud.voxel_size", cloud_voxel_size_);
    get_parameter("cloud.min_points", cloud_min_points_);
    get_parameter("init.enabled", init_enabled_);
    get_parameter("init.accumulate_frames", init_accumulate_frames_);
    get_parameter("init.voxel_size", init_voxel_size_);
    get_parameter("init.local_radius", init_local_radius_);
    get_parameter("init.icp_max_corr", init_icp_max_corr_);
    get_parameter("init.icp_max_iter", init_icp_max_iter_);
    get_parameter("init.fitness_max", init_fitness_max_);
    get_parameter("init.coarse_max_iter", init_coarse_max_iter_);
    get_parameter("init.fine_max_iter", init_fine_max_iter_);
    get_parameter("init.valid_ratio_min", init_valid_ratio_min_);
    get_parameter("init.plane_dist", init_plane_dist_);
    get_parameter("init.prior_T", init_prior_t_);
    get_parameter("init.xy_range", init_xy_range_);
    get_parameter("init.yaw_range", init_yaw_range_);
    get_parameter("prior.enabled", prior_enabled_);
    get_parameter("prior.map_path", prior_map_path_);
    get_parameter("prior.map_voxel_size", prior_map_voxel_size_);
    get_parameter("prior.local_radius", prior_local_radius_);
    get_parameter("prior.icp_max_corr", prior_icp_max_corr_);
    get_parameter("prior.icp_max_iter", prior_icp_max_iter_);
    get_parameter("prior.fitness_max", prior_fitness_max_);
    get_parameter("prior.add_every_n_keyframes", prior_add_every_n_keyframes_);
    get_parameter("prior.noise_xyz", prior_noise_xyz_);
    get_parameter("prior.noise_rpy", prior_noise_rpy_);
    get_parameter("loop.enabled", loop_enabled_);
    get_parameter("loop.min_separation", loop_min_separation_);
    get_parameter("loop.search_radius", loop_search_radius_);
    get_parameter("loop.icp_max_corr", loop_icp_max_corr_);
    get_parameter("loop.icp_max_iter", loop_icp_max_iter_);
    get_parameter("loop.fitness_max", loop_fitness_max_);
    get_parameter("loop.add_every_n_keyframes", loop_add_every_n_keyframes_);
    get_parameter("loop.noise_xyz", loop_noise_xyz_);
    get_parameter("loop.noise_rpy", loop_noise_rpy_);
  }

  void loadPriorMap() {
    if (!prior_enabled_ || prior_map_path_.empty()) return;
    PointCloudXYZI::Ptr raw(new PointCloudXYZI());
    if (pcl::io::loadPCDFile<PointType>(prior_map_path_, *raw) != 0) {
      RCLCPP_WARN(get_logger(), "Failed to load prior map: %s. Prior factors disabled.",
                  prior_map_path_.c_str());
      prior_enabled_ = false;
      return;
    }
    prior_map_ = downsampleCloud(raw, prior_map_voxel_size_);
    prior_init_kdtree_.Build(prior_map_->points);
    prior_init_kdtree_ready_ = !prior_map_->empty();
    RCLCPP_INFO(get_logger(), "Loaded prior map %s: raw=%zu downsampled=%zu voxel=%.3f",
                prior_map_path_.c_str(), raw->size(), prior_map_->size(), prior_map_voxel_size_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    pcl::fromROSMsg(*msg, *cloud);
    latest_cloud_ = downsampleCloud(cloud, cloud_voxel_size_);
    latest_cloud_stamp_ = msg->header.stamp;
    if (init_enabled_ && !map_initialized_) {
      collectInitializationCloud(latest_cloud_, latest_cloud_stamp_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const gtsam::Pose3 odom_pose = odomToPose3(*msg);

    if (!latest_cloud_ || static_cast<int>(latest_cloud_->size()) < cloud_min_points_) return;

    if (init_enabled_ && !map_initialized_) {
      tryInitializeMapToOdom(msg->header.stamp);
      if (map_initialized_) {
        publishMapToOdom(msg->header.stamp);
        publishOptimizedOdom(*msg, map_to_odom_ * odom_pose);
      }
      return;
    }

    publishMapToOdom(msg->header.stamp);
    publishOptimizedOdom(*msg, map_to_odom_ * odom_pose);
    if (!shouldCreateKeyframe(msg->header.stamp, odom_pose)) return;

    addKeyframe(msg->header.stamp, odom_pose, latest_cloud_);
    optimizeAndPublish(msg->header.stamp, odom_pose);
  }

  bool shouldCreateKeyframe(const rclcpp::Time &stamp, const gtsam::Pose3 &odom_pose) const {
    if (keyframes_.empty()) return true;
    const auto &last = keyframes_.back();
    const double dt = (stamp - last.stamp).seconds();
    const double dist = (odom_pose.translation() - last.odom_pose.translation()).norm();
    const double dyaw = std::fabs(normalizeAngle(yawFromPose(odom_pose) - yawFromPose(last.odom_pose)));
    return dt >= keyframe_min_time_ &&
           (dist >= keyframe_min_dist_ || dyaw >= keyframe_min_angle_deg_ * M_PI / 180.0);
  }

  std::vector<double> expandRange(const std::vector<double> &range) const {
    if (range.size() != 3 || std::fabs(range[1]) < 1e-9) return {0.0};
    std::vector<double> values;
    const double min_v = range[0];
    const double step = std::fabs(range[1]);
    const double max_v = range[2];
    for (double v = min_v; v <= max_v + 1e-9; v += step) {
      values.push_back(v);
      if (values.size() > 200) break;
    }
    return values.empty() ? std::vector<double>{0.0} : values;
  }

  void collectInitializationCloud(const PointCloudXYZI::ConstPtr &cloud, const rclcpp::Time &stamp) {
    if (!cloud || cloud->empty()) return;
    PointCloudXYZI::Ptr cloud_copy(new PointCloudXYZI(*cloud));
    init_clouds_.push_back(cloud_copy);
    last_init_cloud_stamp_ = stamp;
    while (static_cast<int>(init_clouds_.size()) > std::max(1, init_accumulate_frames_)) {
      init_clouds_.pop_front();
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Accumulating pose graph init clouds: %zu/%d",
                         init_clouds_.size(), std::max(1, init_accumulate_frames_));
  }

  PointCloudXYZI::Ptr buildInitializationCloud() const {
    PointCloudXYZI::Ptr merged(new PointCloudXYZI());
    for (const auto &cloud : init_clouds_) {
      if (cloud) *merged += *cloud;
    }
    return downsampleCloud(merged, init_voxel_size_);
  }

  gtsam::Pose3 makeInitialCandidate(double dx, double dy, double yaw) const {
    gtsam::Pose3 base_pose = init_seed_valid_ ? init_seed_pose_ : configuredInitialPose();
    const auto base_t = base_pose.translation();
    const double x = base_t.x() + dx;
    const double y = base_t.y() + dy;
    const double z = base_t.z();
    const gtsam::Rot3 rot_delta = gtsam::Rot3::RzRyRx(0.0, 0.0, yaw);
    return gtsam::Pose3(
        rot_delta * base_pose.rotation(),
        gtsam::Point3(x, y, z));
  }

  gtsam::Pose3 configuredInitialPose() const {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (init_prior_t_.size() >= 3) {
      x = init_prior_t_[0];
      y = init_prior_t_[1];
      z = init_prior_t_[2];
    }
    return gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(x, y, z));
  }

  PointCloudXYZI::Ptr cropPriorAroundPose(const gtsam::Pose3 &center_pose, double radius) const {
    PointCloudXYZI::Ptr cropped(new PointCloudXYZI());
    if (!prior_map_ || prior_map_->empty()) return cropped;
    const auto t = center_pose.translation();
    pcl::CropBox<PointType> crop;
    crop.setInputCloud(prior_map_);
    crop.setMin(Eigen::Vector4f(t.x() - radius, t.y() - radius, t.z() - radius, 1.0f));
    crop.setMax(Eigen::Vector4f(t.x() + radius, t.y() + radius, t.z() + radius, 1.0f));
    crop.filter(*cropped);
    return cropped;
  }

  void tryInitializeMapToOdom(const rclcpp::Time &stamp) {
    if (!prior_enabled_ || !prior_map_ || prior_map_->empty() || !prior_init_kdtree_ready_) {
      map_initialized_ = true;
      RCLCPP_WARN_ONCE(get_logger(), "Prior map unavailable; pose graph starts with identity map->odom");
      return;
    }
    if (static_cast<int>(init_clouds_.size()) < std::max(1, init_accumulate_frames_)) return;

    PointCloudXYZI::Ptr fine_scan = buildInitializationCloud();
    PointCloudXYZI::Ptr coarse_scan = downsampleCloud(fine_scan, std::max(init_voxel_size_, init_local_radius_ * 0.0 + prior_map_voxel_size_));
    if (static_cast<int>(fine_scan->size()) < cloud_min_points_) {
      RCLCPP_WARN(get_logger(), "Initialization cloud too small: %zu", fine_scan->size());
      return;
    }

    bool found = false;
    double best_error = std::numeric_limits<double>::max();
    double best_valid_ratio = 0.0;
    double best_dx = 0.0;
    double best_dy = 0.0;
    double best_yaw = 0.0;
    gtsam::Pose3 best_pose;
    const auto xs = expandRange(init_xy_range_);
    const auto ys = expandRange(init_xy_range_);
    const auto yaws = expandRange(init_yaw_range_);
    const int total_candidates = static_cast<int>(xs.size() * ys.size() * yaws.size());
    int tested_candidates = 0;
    int valid_align_results = 0;
    RCLCPP_INFO(get_logger(),
                "Pose graph fast prior initialization started: fine_points=%zu coarse_points=%zu candidates=%d "
                "x=%zu y=%zu yaw=%zu valid_ratio_min=%.2f seed_valid=%d seed=(%.3f, %.3f, %.3f)",
                fine_scan->size(), coarse_scan->size(), total_candidates,
                xs.size(), ys.size(), yaws.size(), init_valid_ratio_min_,
                init_seed_valid_ ? 1 : 0,
                (init_seed_valid_ ? init_seed_pose_ : configuredInitialPose()).translation().x(),
                (init_seed_valid_ ? init_seed_pose_ : configuredInitialPose()).translation().y(),
                (init_seed_valid_ ? init_seed_pose_ : configuredInitialPose()).translation().z());

    const int saved_max_iter = ScanAligner::max_iter;
    const float saved_plane_dist = ScanAligner::plane_dist;
    ScanAligner::plane_dist = static_cast<float>(init_plane_dist_);

    for (double yaw : yaws) {
      for (double dx : xs) {
        for (double dy : ys) {
          tested_candidates++;
          const gtsam::Pose3 candidate = makeInitialCandidate(dx, dy, yaw);
          M4F candidate_pose = pose3ToM4F(candidate);
          ScanAligner::max_iter = std::max(1, init_coarse_max_iter_);
          const auto coarse_result =
              ScanAligner::init_ppicp_method(prior_init_kdtree_, coarse_scan, candidate_pose);
          const double error = static_cast<double>(coarse_result.first);
          const double valid_ratio = static_cast<double>(coarse_result.second);
          valid_align_results++;

          if (valid_ratio > best_valid_ratio ||
              (std::fabs(valid_ratio - best_valid_ratio) < 1e-6 && error < best_error)) {
            best_error = error;
            best_valid_ratio = valid_ratio;
            best_pose = m4fToPose3(candidate_pose);
            best_dx = dx;
            best_dy = dy;
            best_yaw = yaw;
            found = true;
            RCLCPP_INFO(get_logger(),
                        "Pose graph init new best: error=%.4f valid_ratio=%.3f seed=(dx=%.3f, dy=%.3f, yaw=%.3f) "
                        "pose=(%.3f, %.3f, %.3f) progress=%d/%d",
                        best_error, best_valid_ratio, best_dx, best_dy, best_yaw,
                        best_pose.translation().x(),
                        best_pose.translation().y(),
                        best_pose.translation().z(),
                        tested_candidates, total_candidates);
          } else if (tested_candidates == total_candidates || tested_candidates % 20 == 0) {
            RCLCPP_INFO(get_logger(),
                        "Pose graph init progress: %d/%d tested, valid_align=%d, "
                        "best_error=%.4f best_valid_ratio=%.3f best_seed=(%.3f, %.3f, %.3f)",
                        tested_candidates, total_candidates, valid_align_results,
                        best_error, best_valid_ratio, best_dx, best_dy, best_yaw);
          }
        }
      }
    }

    if (found) {
      M4F fine_pose = pose3ToM4F(best_pose);
      ScanAligner::max_iter = std::max(1, init_fine_max_iter_);
      const auto fine_result =
          ScanAligner::init_ppicp_method(prior_init_kdtree_, fine_scan, fine_pose);
      best_error = static_cast<double>(fine_result.first);
      best_valid_ratio = static_cast<double>(fine_result.second);
      best_pose = m4fToPose3(fine_pose);
      RCLCPP_INFO(get_logger(),
                  "Pose graph init fine refine: error=%.4f valid_ratio=%.3f pose=(%.3f, %.3f, %.3f)",
                  best_error, best_valid_ratio,
                  best_pose.translation().x(), best_pose.translation().y(), best_pose.translation().z());
    }

    ScanAligner::max_iter = saved_max_iter;
    ScanAligner::plane_dist = saved_plane_dist;

    if (!found || best_valid_ratio < init_valid_ratio_min_) {
      if (found) {
        init_seed_pose_ = best_pose;
        init_seed_valid_ = true;
      }
      RCLCPP_WARN(get_logger(),
                  "Pose graph prior initialization rejected. found=%d best_error=%.4f best_valid_ratio=%.3f threshold=%.3f "
                  "best_seed=(dx=%.3f, dy=%.3f, yaw=%.3f) best_pose=(%.3f, %.3f, %.3f). "
                  "Next init will continue around this best pose.",
                  found ? 1 : 0, best_error, best_valid_ratio, init_valid_ratio_min_,
                  best_dx, best_dy, best_yaw,
                  found ? best_pose.translation().x() : 0.0,
                  found ? best_pose.translation().y() : 0.0,
                  found ? best_pose.translation().z() : 0.0);
      return;
    }

    map_to_odom_ = best_pose;
    init_seed_pose_ = best_pose;
    init_seed_valid_ = true;
    map_initialized_ = true;
    init_clouds_.clear();
    RCLCPP_WARN(get_logger(),
                "Pose graph initialized map->odom from prior map. error=%.4f valid_ratio=%.3f "
                "best_seed=(dx=%.3f, dy=%.3f, yaw=%.3f) xyz=(%.3f, %.3f, %.3f)",
                best_error, best_valid_ratio,
                best_dx, best_dy, best_yaw,
                map_to_odom_.translation().x(),
                map_to_odom_.translation().y(),
                map_to_odom_.translation().z());
    publishMapToOdom(stamp);
  }

  void addKeyframe(const rclcpp::Time &stamp, const gtsam::Pose3 &odom_pose,
                   const PointCloudXYZI::Ptr &cloud) {
    Keyframe kf;
    kf.id = next_keyframe_id_++;
    kf.stamp = stamp;
    kf.odom_pose = odom_pose;
    kf.optimized_pose = map_to_odom_ * odom_pose;
    kf.cloud.reset(new PointCloudXYZI(*cloud));

    const auto symbol = gtsam::Symbol('x', kf.id);
    initial_.insert(symbol, kf.optimized_pose);

    if (keyframes_.empty()) {
      auto noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.10, 0.10, 0.10).finished());
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(symbol, kf.optimized_pose, noise));
    } else {
      const auto &last = keyframes_.back();
      const gtsam::Pose3 relative = last.odom_pose.between(kf.odom_pose);
      auto noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.03, 0.03, 0.05, 0.10, 0.10, 0.10).finished());
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          gtsam::Symbol('x', last.id), symbol, relative, noise));
    }

    keyframes_.push_back(kf);
    addPriorFactorIfAvailable(keyframes_.back());
    addLoopFactorIfAvailable(keyframes_.back());

    while (static_cast<int>(keyframes_.size()) > keyframe_max_count_) {
      keyframes_.pop_front();
    }
  }

  void addPriorFactorIfAvailable(const Keyframe &kf) {
    if (!prior_enabled_ || !prior_map_ || prior_map_->empty()) return;
    if (prior_add_every_n_keyframes_ > 1 && kf.id % prior_add_every_n_keyframes_ != 0) return;

    PointCloudXYZI::Ptr local_map = cropPriorLocalMap(kf.optimized_pose);
    if (!local_map || static_cast<int>(local_map->size()) < cloud_min_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Prior local map too small around current pose");
      return;
    }

    gtsam::Pose3 refined_pose;
    double fitness = 0.0;
    if (!alignToTarget(kf.cloud, local_map, kf.optimized_pose, prior_icp_max_corr_,
                       prior_icp_max_iter_, refined_pose, fitness)) {
      return;
    }
    if (fitness > prior_fitness_max_) {
      RCLCPP_WARN(get_logger(), "Reject prior factor kf=%d fitness=%.4f > %.4f",
                  kf.id, fitness, prior_fitness_max_);
      return;
    }

    auto base_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << prior_noise_rpy_, prior_noise_rpy_, prior_noise_rpy_,
         prior_noise_xyz_, prior_noise_xyz_, prior_noise_xyz_).finished());
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', kf.id), refined_pose, robust));
    RCLCPP_INFO(get_logger(), "Add prior factor kf=%d fitness=%.4f", kf.id, fitness);
    publishPriorDebugMap(local_map, kf.stamp);
  }

  void addLoopFactorIfAvailable(const Keyframe &kf) {
    if (!loop_enabled_) return;
    if (loop_add_every_n_keyframes_ > 1 && kf.id % loop_add_every_n_keyframes_ != 0) return;

    const Keyframe *candidate = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto &old : keyframes_) {
      if (kf.id - old.id < loop_min_separation_) continue;
      const double dist = (kf.optimized_pose.translation() - old.optimized_pose.translation()).norm();
      if (dist < loop_search_radius_ && dist < best_dist) {
        candidate = &old;
        best_dist = dist;
      }
    }
    if (!candidate) return;

    gtsam::Pose3 refined_current;
    double fitness = 0.0;
    if (!alignToTarget(kf.cloud, candidate->cloud, kf.optimized_pose, loop_icp_max_corr_,
                       loop_icp_max_iter_, refined_current, fitness)) {
      return;
    }
    if (fitness > loop_fitness_max_) {
      RCLCPP_WARN(get_logger(), "Reject loop factor kf=%d old=%d fitness=%.4f > %.4f",
                  kf.id, candidate->id, fitness, loop_fitness_max_);
      return;
    }

    const gtsam::Pose3 relative = candidate->optimized_pose.between(refined_current);
    auto base_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << loop_noise_rpy_, loop_noise_rpy_, loop_noise_rpy_,
         loop_noise_xyz_, loop_noise_xyz_, loop_noise_xyz_).finished());
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1.0), base_noise);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', candidate->id), gtsam::Symbol('x', kf.id), relative, robust));
    RCLCPP_INFO(get_logger(), "Add loop factor old=%d current=%d fitness=%.4f",
                candidate->id, kf.id, fitness);
  }

  PointCloudXYZI::Ptr cropPriorLocalMap(const gtsam::Pose3 &center_pose) const {
    PointCloudXYZI::Ptr cropped(new PointCloudXYZI());
    if (!prior_map_ || prior_map_->empty()) return cropped;
    const auto t = center_pose.translation();
    pcl::CropBox<PointType> crop;
    crop.setInputCloud(prior_map_);
    crop.setMin(Eigen::Vector4f(t.x() - prior_local_radius_, t.y() - prior_local_radius_,
                                t.z() - prior_local_radius_, 1.0f));
    crop.setMax(Eigen::Vector4f(t.x() + prior_local_radius_, t.y() + prior_local_radius_,
                                t.z() + prior_local_radius_, 1.0f));
    crop.filter(*cropped);
    return cropped;
  }

  bool alignToTarget(const PointCloudXYZI::ConstPtr &source, const PointCloudXYZI::ConstPtr &target,
                     const gtsam::Pose3 &initial_pose, double max_corr, int max_iter,
                     gtsam::Pose3 &refined_pose, double &fitness) const {
    if (!source || !target || source->empty() || target->empty()) return false;
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(max_corr);
    icp.setMaximumIterations(max_iter);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);

    PointCloudXYZI aligned;
    icp.align(aligned, pose3ToEigen(initial_pose));
    if (!icp.hasConverged()) return false;
    fitness = icp.getFitnessScore();
    refined_pose = eigenToPose3(icp.getFinalTransformation());
    return true;
  }

  void optimizeAndPublish(const rclcpp::Time &stamp, const gtsam::Pose3 &latest_odom_pose) {
    isam_.update(graph_, initial_);
    isam_.update();
    graph_.resize(0);
    initial_.clear();
    result_ = isam_.calculateEstimate();

    for (auto &kf : keyframes_) {
      const auto symbol = gtsam::Symbol('x', kf.id);
      if (result_.exists(symbol)) {
        kf.optimized_pose = result_.at<gtsam::Pose3>(symbol);
      }
    }
    if (!keyframes_.empty()) {
      map_to_odom_ = keyframes_.back().optimized_pose * latest_odom_pose.inverse();
    }
    publishPath(stamp);
    publishMapToOdom(stamp);
  }

  void publishPath(const rclcpp::Time &stamp) {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    for (const auto &kf : keyframes_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = kf.stamp;
      pose.header.frame_id = map_frame_;
      pose.pose = pose3ToMsg(kf.optimized_pose);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void publishMapToOdom(const rclcpp::Time &stamp) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    const auto t = map_to_odom_.translation();
    const auto q = map_to_odom_.rotation().toQuaternion();
    transform.transform.translation.x = t.x();
    transform.transform.translation.y = t.y();
    transform.transform.translation.z = t.z();
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(transform);
  }

  void publishOptimizedOdom(const nav_msgs::msg::Odometry &source_odom,
                            const gtsam::Pose3 &optimized_pose) {
    nav_msgs::msg::Odometry odom = source_odom;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = source_odom.child_frame_id;
    odom.pose.pose = pose3ToMsg(optimized_pose);
    odom_pub_->publish(odom);
  }

  void publishPriorDebugMap(const PointCloudXYZI::ConstPtr &cloud, const rclcpp::Time &stamp) {
    if (!cloud || prior_debug_pub_->get_subscription_count() == 0) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    prior_debug_pub_->publish(msg);
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  double keyframe_min_dist_ = 0.5;
  double keyframe_min_angle_deg_ = 8.0;
  double keyframe_min_time_ = 0.5;
  int keyframe_max_count_ = 2000;
  double cloud_voxel_size_ = 0.25;
  int cloud_min_points_ = 80;
  bool init_enabled_ = true;
  bool map_initialized_ = false;
  int init_accumulate_frames_ = 3;
  double init_voxel_size_ = 0.20;
  double init_local_radius_ = 20.0;
  double init_icp_max_corr_ = 1.5;
  int init_icp_max_iter_ = 40;
  double init_fitness_max_ = 0.40;
  int init_coarse_max_iter_ = 3;
  int init_fine_max_iter_ = 15;
  double init_valid_ratio_min_ = 0.65;
  double init_plane_dist_ = 0.10;
  std::vector<double> init_prior_t_;
  std::vector<double> init_xy_range_;
  std::vector<double> init_yaw_range_;
  bool init_seed_valid_ = false;
  gtsam::Pose3 init_seed_pose_;
  bool prior_enabled_ = true;
  std::string prior_map_path_;
  double prior_map_voxel_size_ = 0.30;
  double prior_local_radius_ = 15.0;
  double prior_icp_max_corr_ = 1.0;
  int prior_icp_max_iter_ = 30;
  double prior_fitness_max_ = 0.30;
  int prior_add_every_n_keyframes_ = 3;
  double prior_noise_xyz_ = 0.20;
  double prior_noise_rpy_ = 0.10;
  bool loop_enabled_ = true;
  int loop_min_separation_ = 30;
  double loop_search_radius_ = 2.0;
  double loop_icp_max_corr_ = 1.0;
  int loop_icp_max_iter_ = 30;
  double loop_fitness_max_ = 0.20;
  int loop_add_every_n_keyframes_ = 5;
  double loop_noise_xyz_ = 0.15;
  double loop_noise_rpy_ = 0.08;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_debug_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  PointCloudXYZI::Ptr latest_cloud_;
  rclcpp::Time latest_cloud_stamp_;
  rclcpp::Time last_init_cloud_stamp_;
  PointCloudXYZI::Ptr prior_map_;
  std::deque<PointCloudXYZI::Ptr> init_clouds_;
  KD_TREE<PointType> prior_init_kdtree_;
  bool prior_init_kdtree_ready_ = false;
  std::deque<Keyframe> keyframes_;
  int next_keyframe_id_ = 0;

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  gtsam::Values result_;
  gtsam::Pose3 map_to_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseGraphBackend>());
  rclcpp::shutdown();
  return 0;
}
