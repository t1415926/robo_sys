#pragma once
#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp" //#include <ekfom/ekfom.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <vector>
#include "map_manager.hpp"
#include "scan_aligner.hpp"

// 状态定义 (基于FAST-LIO的状态定义)
struct State_ikfom {
    Eigen::Quaterniond rot;         // 世界坐标系下的姿态
    Eigen::Vector3d pos;             // 世界坐标系下的位置
    Eigen::Vector3d vel;             // 世界坐标系下的速度
    Eigen::Vector3d bg;              // 陀螺仪偏置
    Eigen::Vector3d ba;              // 加速度计偏置
    Eigen::Vector3d grav;            // 重力矢量
    
    // LiDAR-IMU外参
    Eigen::Quaterniond offset_R_L_I;   // LiDAR到IMU的旋转
    Eigen::Vector3d offset_T_L_I;       // LiDAR到IMU的平移
    
    // 状态向量大小
    static constexpr size_t size = 30; // 姿态(4) + 位置(3) + 速度(3) + bg(3) + ba(3) + 重力(3) + 外参旋转(4) + 外参平移(3)
    
    State_ikfom() : 
        rot(Eigen::Quaterniond::Identity()), 
        pos(Eigen::Vector3d::Zero()),
        vel(Eigen::Vector3d::Zero()),
        bg(Eigen::Vector3d::Zero()),
        ba(Eigen::Vector3d::Zero()),
        grav(0, 0, -9.81),
        offset_R_L_I(Eigen::Quaterniond::Identity()),
        offset_T_L_I(Eigen::Vector3d::Zero()) {}
};

// 输入定义 (IMU输入)
struct Input_ikfom {
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    
    Input_ikfom() : 
        acc(Eigen::Vector3d::Zero()), 
        gyro(Eigen::Vector3d::Zero()) {}
};

class EKFLocalizer : public rclcpp::Node {
public:
    using PoseMsg = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using ImuMsg = sensor_msgs::msg::Imu;
    using OdometryMsg = nav_msgs::msg::Odometry;

    EKFLocalizer(rclcpp::Node::SharedPtr parent_node, MapManager::KdTreePtr map_tree) 
        : Node("ekf_localizer"), map_tree_(map_tree) {
        // 初始化参数
        this->declare_parameter<double>("gyr_cov", 0.1);
        this->declare_parameter<double>("acc_cov", 0.1);
        this->declare_parameter<double>("b_gyr_cov", 0.0001);
        this->declare_parameter<double>("b_acc_cov", 0.0001);
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("extrinsic_est", true);
        this->declare_parameter<std::vector<double>>("extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        this->declare_parameter<bool>("publish_tf", true);
        
        double gyr_cov = this->get_parameter("gyr_cov").as_double();
        double acc_cov = this->get_parameter("acc_cov").as_double();
        double b_gyr_cov = this->get_parameter("b_gyr_cov").as_double();
        double b_acc_cov = this->get_parameter("b_acc_cov").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        extrinsic_est_ = this->get_parameter("extrinsic_est").as_bool();
        
        auto extrinsic_T = this->get_parameter("extrinsic_T").as_double_array();
        auto extrinsic_R = this->get_parameter("extrinsic_R").as_double_array();
        
        // 订阅器和发布器
        cloud_sub_ = this->create_subscription<PointCloudMsg>(
            "points_in", rclcpp::SensorDataQoS(),
            std::bind(&EKFLocalizer::cloudCallback, this, std::placeholders::_1));
            
        imu_sub_ = this->create_subscription<ImuMsg>(
            "imu_in", rclcpp::SensorDataQoS(),
            std::bind(&EKFLocalizer::imuCallback, this, std::placeholders::_1));
            
        pose_pub_ = this->create_publisher<PoseWithCovarianceMsg>("global_pose", 10);
        odom_pub_ = this->create_publisher<OdometryMsg>("odometry", 10);
        
        // 初始位姿服务
        initial_pose_service_ = this->create_service<geometry_msgs::srv::SetPose>(
            "set_initial_pose",
            std::bind(&EKFLocalizer::setInitialPoseCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 初始化EKF
        initEKF();
        
        // 设置外参
        Eigen::Vector3d ext_T(extrinsic_T[0], extrinsic_T[1], extrinsic_T[2]);
        Eigen::Matrix3d ext_R;
        ext_R << extrinsic_R[0], extrinsic_R[1], extrinsic_R[2],
                 extrinsic_R[3], extrinsic_R[4], extrinsic_R[5],
                 extrinsic_R[6], extrinsic_R[7], extrinsic_R[8];
        Eigen::Quaterniond ext_rot(ext_R);
        
        state_.offset_T_L_I = ext_T;
        state_.offset_R_L_I = ext_rot;
        
        RCLCPP_INFO(this->get_logger(), "EKF Localizer initialized");
    }
    
    void cloudCallback(const PointCloudMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 转换点云到PCL格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // 简单下采样
        if (cloud->size() > 10000) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
            voxel_grid.setInputCloud(cloud);
            voxel_grid.setLeafSize(0.1, 0.1, 0.1); // 10cm voxel size
            voxel_grid.filter(*downsampled_cloud);
            cloud = downsampled_cloud;
        }
        
        // 第一次处理时执行初始化配准
        if (first_scan_ && map_tree_ && map_tree_->Root_Node) {
            bool init_success = initialScanAlignment(cloud);
            if (init_success) {
                RCLCPP_INFO(this->get_logger(), "Initial scan alignment successful!");
                ekfom_data_.converge = true;
                first_scan_ = false;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Initial scan alignment failed!");
                return;
            }
        }
        
        // 预测步骤：使用最新的IMU数据更新状态
        if (!imu_buffer_.empty()) {
            kf_.predict(dt_, Q_, input_);
        }
        
        // 定义观测模型 (地图约束)
        auto h_share_model = [this](State_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
            if (ekfom_data.converge) {
                this->observationModel(s, ekfom_data);
            }
        };
        
        // 执行EKF更新
        kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time_, h_share_model);
        
        // 保存当前状态
        state_ = kf_.get_x();
        
        // 发布定位结果
        publishPose();
        publishOdometry();
        
        if (publish_tf_) {
            publishTF();
        }
    }
    
    void imuCallback(const ImuMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 保存IMU数据
        imu_buffer_.push_back(*msg);
        
        // 保持最近50个IMU消息
        if (imu_buffer_.size() > 50) {
            imu_buffer_.pop_front();
        }
        
        // 更新输入
        input_.acc << msg->linear_acceleration.x, 
                     msg->linear_acceleration.y, 
                     msg->linear_acceleration.z;
        input_.gyro << msg->angular_velocity.x, 
                      msg->angular_velocity.y, 
                      msg->angular_velocity.z;
        
        // 更新时间增量
        if (last_imu_time_.seconds() == 0) {
            last_imu_time_ = msg->header.stamp;
            return;
        }
        
        dt_ = (rclcpp::Time(msg->header.stamp) - last_imu_time_).seconds();
        last_imu_time_ = msg->header.stamp;
        
        // 如果没有点云数据，只执行预测
        if (imu_only_mode_) {
            kf_.predict(dt_, Q_, input_);
            state_ = kf_.get_x();
            publishOdometry();
            
            if (publish_tf_) {
                publishTF();
            }
        }
    }
    
    void setInitialPoseCallback(const std::shared_ptr<geometry_msgs::srv::SetPose::Request> request,
                                std::shared_ptr<geometry_msgs::srv::SetPose::Response> response) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        const auto& pose = request->initial_pose.pose;
        state_.pos << pose.position.x, pose.position.y, pose.position.z;
        state_.rot = Eigen::Quaterniond(pose.orientation.w, 
                                      pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z);
        
        kf_.change_x(state_);
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set to [%.2f, %.2f, %.2f]", 
                   state_.pos.x(), state_.pos.y(), state_.pos.z());
    }
    
private:
    // 常量定义
    static constexpr double LASER_POINT_COV = 0.001; // 激光点云的噪声协方差
    
    // 初始化EKF
    void initEKF() {
        // 状态初始值
        state_ = State_ikfom();
        
        // 初始协方差矩阵
        Eigen::Matrix<double, State_ikfom::size, State_ikfom::size> P = Eigen::Matrix<double, State_ikfom::size, State_ikfom::size>::Identity();
        
        // 设置初始不确定性
        P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.01; // 位置
        P.block<4, 4>(3, 3) = Eigen::Matrix4d::Identity() * 0.01; // 姿态
        P.block<3, 3>(7, 7) = Eigen::Matrix3d::Identity() * 0.1;  // 速度
        P.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * 0.0001; // bg
        P.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * 0.0001; // ba
        P.block<3, 3>(16, 16) = Eigen::Matrix3d::Identity() * 0.01; // 重力
        P.block<3, 3>(19, 19) = Eigen::Matrix3d::Identity() * 0.001; // 外参平移
        P.block<3, 3>(22, 22) = Eigen::Matrix3d::Identity() * 0.001; // 外参旋转(使用欧拉角表示)
        
        // 过程噪声协方差矩阵
        Q_ = Eigen::Matrix<double, 12, 12>::Identity();
        Q_.block<3, 3>(0, 0) *= get_parameter("gyr_cov").as_double();
        Q_.block<3, 3>(3, 3) *= get_parameter("acc_cov").as_double();
        Q_.block<3, 3>(6, 6) *= get_parameter("b_gyr_cov").as_double();
        Q_.block<3, 3>(9, 9) *= get_parameter("b_acc_cov").as_double();
        
        // 初始化EKF
        kf_.init(state_, P, Q_);
        
        // 初始化其他变量
        first_scan_ = true;
        imu_only_mode_ = false;
        solve_H_time_ = 0.0;
    }
    
    // 初始扫描配准
    bool initialScanAlignment(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        // 从地图树中获取点云
        std::vector<pcl::PointXYZI> map_points;
        map_tree_->acquire_removed_points(map_points);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& pt : map_points) {
            map_cloud->points.push_back(pt);
        }
        
        // 创建扫描配准器
        ScanAligner aligner(ScanAligner::GICP);
        
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        if (!last_state_valid_) {
            // 如果不知道上次状态，使用初始外参估计位置
            Eigen::Affine3d lidar2map = state_.rot * state_.offset_R_L_I;
            init_guess.block<3, 3>(0, 0) = lidar2map.rotation().cast<float>();
            init_guess.block<3, 1>(0, 3) = lidar2map.translation().cast<float>();
        } else {
            // 使用上次状态作为初始估计
            Eigen::Affine3d lidar2map = state_.rot * state_.offset_R_L_I;
            init_guess.block<3, 3>(0, 0) = lidar2map.rotation().cast<float>();
            init_guess.block<3, 1>(0, 3) = lidar2map.translation().cast<float>();
        }
        
        // 执行配准
        Eigen::Matrix4f transformation;
        if (!aligner.align(cloud, map_cloud, transformation)) {
            return false;
        }
        
        // 更新状态
        Eigen::Matrix4d transformation_d = transformation.cast<double>();
        state_.rot = Eigen::Quaterniond(transformation_d.block<3, 3>(0, 0));
        state_.pos = transformation_d.block<3, 1>(0, 3);
        
        // 重置速度和偏置
        state_.vel.setZero();
        state_.bg.setZero();
        state_.ba.setZero();
        
        // 更新EKF状态
        kf_.change_x(state_);
        
        last_state_valid_ = true;
        return true;
    }
    
    // 观测模型 - 地图约束
    void observationModel(State_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
        // 这里简化了实现，实际应用中需要实现完整的点到平面约束
        // 这是一个简化的实现，实际应用中需要实现完整的点到平面约束
        
        // 1. 将点云从lidar坐标系转换到map坐标系
        pcl::PointCloud<pcl::PointXYZI> map_cloud;
        for (size_t i = 0; i < current_scan_->size(); i++) {
            const auto& pt = current_scan_->points[i];
            Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);
            
            // Transform to IMU frame
            Eigen::Vector3d p_imu = s.offset_R_L_I * p_lidar + s.offset_T_L_I;
            
            // Transform to map frame
            Eigen::Vector3d p_map = s.rot * p_imu + s.pos;
            
            pcl::PointXYZI map_pt;
            map_pt.x = p_map.x();
            map_pt.y = p_map.y();
            map_pt.z = p_map.z();
            map_cloud.points.push_back(map_pt);
        }
        
        // 2. 搜索最近邻并计算残差
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        double total_residual = 0.0;
        int valid_points = 0;
        
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(map_cloud.size(), State_ikfom::size);
        Eigen::VectorXd z(map_cloud.size());
        
        for (size_t i = 0; i < map_cloud.size(); i++) {
            const auto& pt = map_cloud.points[i];
            
            // 在ikd-Tree中搜索最近邻
            if (map_tree_->Nearest_Search(pt, 5, nearest_points_, distances_) > 1) {
                // 计算平面法向量
                Eigen::Matrix3d cov;
                Eigen::Vector3d mean;
                pcl::computeMeanAndCovarianceMatrix(nearest_points_, cov, mean);
                
                // 奇异值分解求法向量
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
                Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
                
                // 计算点到平面的距离
                Eigen::Vector3d pos(pt.x, pt.y, pt.z);
                double residual = normal.dot(pos - mean);
                
                // 更新残差
                total_residual += residual;
                valid_points++;
                
                // 简化: 不考虑雅可比矩阵计算
                // 实际中需要计算观测方程对状态的雅可比
                // H.row(i) = ...;
                
                z(i) = -residual;
            }
        }
        
        if (valid_points > 100) { // 确保足够多的有效点
            ekfom_data.valid = true;
            ekfom_data.h = z;
            ekfom_data.h_x = H;
        } else {
            ekfom_data.valid = false;
        }
    }
    
    // 发布位姿
    void publishPose() {
        auto pose_msg = PoseWithCovarianceMsg();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame_;
        
        // 设置位姿
        pose_msg.pose.pose.position.x = state_.pos.x();
        pose_msg.pose.pose.position.y = state_.pos.y();
        pose_msg.pose.pose.position.z = state_.pos.z();
        pose_msg.pose.pose.orientation.x = state_.rot.x();
        pose_msg.pose.pose.orientation.y = state_.rot.y();
        pose_msg.pose.pose.orientation.z = state_.rot.z();
        pose_msg.pose.pose.orientation.w = state_.rot.w();
        
        // 设置协方差（简化）
        for (int i = 0; i < 6; i++) {
            pose_msg.pose.covariance[i * 6 + i] = 0.01; // 对角线元素
        }
        
        pose_pub_->publish(pose_msg);
    }
    
    // 发布里程计
    void publishOdometry() {
        auto odom_msg = OdometryMsg();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_frame_;
        
        // 设置位姿
        odom_msg.pose.pose.position.x = state_.pos.x();
        odom_msg.pose.pose.position.y = state_.pos.y();
        odom_msg.pose.pose.position.z = state_.pos.z();
        odom_msg.pose.pose.orientation.x = state_.rot.x();
        odom_msg.pose.pose.orientation.y = state_.rot.y();
        odom_msg.pose.pose.orientation.z = state_.rot.z();
        odom_msg.pose.pose.orientation.w = state_.rot.w();
        
        // 设置速度
        odom_msg.twist.twist.linear.x = state_.vel.x();
        odom_msg.twist.twist.linear.y = state_.vel.y();
        odom_msg.twist.twist.linear.z = state_.vel.z();
        
        // 设置协方差（简化）
        for (int i = 0; i < 6; i++) {
            odom_msg.pose.covariance[i * 6 + i] = 0.01;
            odom_msg.twist.covariance[i * 6 + i] = 0.01;
        }
        
        odom_pub_->publish(odom_msg);
    }
    
    // 发布TF变换
    void publishTF() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = base_frame_;
        
        transform.transform.translation.x = state_.pos.x();
        transform.transform.translation.y = state_.pos.y();
        transform.transform.translation.z = state_.pos.z();
        transform.transform.rotation.x = state_.rot.x();
        transform.transform.rotation.y = state_.rot.y();
        transform.transform.rotation.z = state_.rot.z();
        transform.transform.rotation.w = state_.rot.w();
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    // 成员变量
    MapManager::KdTreePtr map_tree_;
    rclcpp::Subscription<PointCloudMsg>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
    rclcpp::Publisher<PoseWithCovarianceMsg>::SharedPtr pose_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
    rclcpp::Service<geometry_msgs::srv::SetPose>::SharedPtr initial_pose_service_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // EKF相关
    esekfom::esekf<State_ikfom, 12, Input_ikfom> kf_; // 12维噪声
    State_ikfom state_;
    Input_ikfom input_;
    
    // 数据缓冲区
    std::deque<ImuMsg> imu_buffer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_;
    
    // 其他状态
    bool first_scan_;
    bool last_state_valid_;
    bool imu_only_mode_;
    bool extrinsic_est_;
    bool publish_tf_;
    double solve_H_time_;
    double dt_;
    Eigen::Matrix<double, State_ikfom::size, State_ikfom::size> Q_;
    Eigen::Matrix<double, 12, 12> R_; // 观测噪声协方差
    
    // 时间管理
    rclcpp::Time last_imu_time_;
    
    // 临时存储最近邻搜索
    std::vector<pcl::PointXYZI> nearest_points_;
    std::vector<float> distances_;
    esekfom::dyn_share_datastruct<double> ekfom_data_;
    
    // 坐标系
    std::string base_frame_;
    std::string map_frame_;
    
    // 线程安全
    std::mutex data_mutex_;
};
