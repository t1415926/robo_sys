#include "post_mapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <atomic>

/*
测试说明 测试submap+icp的功能
订阅定位+噪声
使用icp去噪



目前icp只有yaw搜索 ，可以加上roll pitch的搜索
*/

// 使用智能指针管理全局资源
std::shared_ptr<KD_TREE<PointType>> global_map_ikdtree;
std::shared_ptr<pcl::PointCloud<PointType>> cloud_effected;
std::mutex cloud_mutex;  // 保护cloud_effected的访问
std::atomic<bool> cloud_ready{false};

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor() : Node("pointcloud_processor")
    {
        // 初始化全局变量
        if (!global_map_ikdtree) {
            global_map_ikdtree = std::make_shared<KD_TREE<PointType>>();
        }
        if (!cloud_effected) {
            cloud_effected = std::make_shared<pcl::PointCloud<PointType>>();
        }
        
        // 创建订阅器
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", 10,  // 减小队列大小
            std::bind(&PointCloudProcessor::cloudCallback, this, std::placeholders::_1));
                
        cloud_reg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "cloud_reg_pub_", 
            rclcpp::QoS(10).reliable()
        );
        cloud_subamap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "cloud_subamap_pub_", 
            rclcpp::QoS(10).reliable()
        );

        // TF2监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Processor node initialized");
    }
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_reg_pub_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subamap_pub_;
private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 空指针检查
        if (!msg) {
            RCLCPP_WARN(this->get_logger(), "Received null message");
            return;
        }
        
        // 转换ROS消息为PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        try {
            pcl::fromROSMsg(*msg, *cloud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud conversion failed: %s", e.what());
            return;
        }
        
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }
        
        // RCLCPP_INFO(this->get_logger(), "Received cloud with %ld points", cloud->size());
        
        // 处理点云
        processPointCloud(cloud);
    }
    
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    {
        if (!input_cloud || input_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid input cloud in processPointCloud");
            return;
        }
        
        std::lock_guard<std::mutex> lock(cloud_mutex);
        
        try {
            cloud_effected->points.resize(input_cloud->points.size());
            
            // 逐个点转换 
            for (size_t i = 0; i < input_cloud->points.size(); ++i) {
                 if (input_cloud->points[i].x* input_cloud->points[i].x+input_cloud->points[i].y* input_cloud->points[i].y+input_cloud->points[i].z* input_cloud->points[i].z>100){
                    continue;
                 }
                cloud_effected->points[i].x = input_cloud->points[i].x;
                cloud_effected->points[i].y = input_cloud->points[i].y;
                cloud_effected->points[i].z = input_cloud->points[i].z;
                cloud_effected->points[i].intensity = input_cloud->points[i].intensity;
            }
            cloud_effected->width = input_cloud->points.size();
            cloud_effected->height = 1;
            cloud_effected->is_dense = true;
            cloud_ready = true;
            // RCLCPP_INFO(this->get_logger(), "Processed %ld points", input_cloud->size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in processPointCloud: %s", e.what());
            cloud_ready = false;
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

bool loadPriorMap(const std::string& map_path, bool use_prior_map, 
                  float filter_size_map_min, 
                  std::shared_ptr<KD_TREE<PointType>>& ikdtree)
{
    if (!use_prior_map) {
        RCLCPP_INFO(rclcpp::get_logger("post_mapper"), "Not using prior map");
        return true;
    }
    
    pcl::PointCloud<PointType>::Ptr prior_cloud(new pcl::PointCloud<PointType>());
    
    try {
        if (pcl::io::loadPCDFile<PointType>(map_path, *prior_cloud) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("post_mapper"), 
                        "Failed to load prior map from: %s", map_path.c_str());
            return false;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("post_mapper"), 
                   "Loaded prior map with %ld points", prior_cloud->size());
        
        // 对先验地图进行下采样
        if (!prior_cloud->empty()) {
            pcl::VoxelGrid<PointType> voxel;
            voxel.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
            voxel.setInputCloud(prior_cloud);
            voxel.filter(*prior_cloud);
            
            RCLCPP_INFO(rclcpp::get_logger("post_mapper"), 
                       "Downsampled to %ld points", prior_cloud->size());
        }
        
        // 构建KD-Tree
        if (ikdtree) {
            ikdtree->Build(prior_cloud->points);
            RCLCPP_INFO(rclcpp::get_logger("post_mapper"), "KD-Tree built successfully");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("post_mapper"), "KD-Tree pointer is null");
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("post_mapper"), 
                    "Exception loading prior map: %s", e.what());
        return false;
    }
}



// 计算真实变换与估计变换的差异
void evaluateRegistration(const M4F& ground_truth, const M4F& estimated, 
                         const PointType& center_point) {
    // 平移误差
    V3F gt_translation = ground_truth.block<3, 1>(0, 3);
    V3F est_translation = estimated.block<3, 1>(0, 3);
    float translation_error = (gt_translation - est_translation).norm();
    
    // 旋转误差（角度差）
    M3F gt_rotation = ground_truth.block<3, 3>(0, 0);
    M3F est_rotation = estimated.block<3, 3>(0, 0);
    M3F rotation_diff = gt_rotation * est_rotation.transpose();
    float rotation_error = Eigen::AngleAxisf(rotation_diff).angle() * 180.0 / M_PI;
    
    std::cout << "=== 配准性能评估 ===" << std::endl;
    std::cout << "平移误差: " << translation_error << " 米" << std::endl;
    std::cout << "旋转误差: " << rotation_error << " 度" << std::endl;
    std::cout << "噪声水平: ±1米" << std::endl;
}

M4F calculateGroundTruthTransform(const geometry_msgs::msg::TransformStamped& raw_tf) {
    /**
     * 从TF变换中提取完整的真实变换矩阵（包含旋转和平移）
     */
    
    M4F ground_truth = M4F::Identity();
    
    // 提取平移分量 因为归中了，以当前坐标系平移零
    ground_truth(0, 3) = 0;
    ground_truth(1, 3) = 0;
    ground_truth(2, 3) = 0;
    
    // 提取旋转分量（四元数转换为旋转矩阵）
    Eigen::Quaternionf quat(
        raw_tf.transform.rotation.w,
        raw_tf.transform.rotation.x,
        raw_tf.transform.rotation.y,
        raw_tf.transform.rotation.z
    );
    
    ground_truth.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
    
    return ground_truth;
}

void evaluateNoiseRemovalPerformance(const M4F& noise_transform, const M4F& recovered_transform,
                                    float noise_x, float noise_y, float noise_z) {
    /**
     * 评估ICP去除噪声的能力
     * noise_transform: 添加的噪声变换
     * recovered_transform: ICP恢复的变换
     * 理想情况下：recovered_transform ≈ noise_transform^{-1}
     */
    
    // 计算添加的噪声大小
    float added_noise_magnitude = sqrt(noise_x*noise_x + noise_y*noise_y + noise_z*noise_z);
    
    // ICP恢复的变换应该接近于噪声变换的逆
    // 理想情况：T_recovered * T_noise ≈ I
    M4F compensation_transform = recovered_transform * noise_transform;
    
    // 计算补偿后的残差
    M4F identity = M4F::Identity();
    M4F residual_transform = compensation_transform;
    
    // 平移残差
    float translation_residual = residual_transform.block<3, 1>(0, 3).norm();
    
    // 旋转残差（角度）
    float rotation_residual = Eigen::AngleAxisf(residual_transform.block<3, 3>(0, 0)).angle() * 180.0 / M_PI;
    
    // 噪声去除比例
    float noise_removal_ratio = 1.0 - (translation_residual / added_noise_magnitude);
    
    std::cout << "=== 噪声去除能力评估 ===" << std::endl;
    std::cout << "添加的噪声大小: " << added_noise_magnitude << " 米" << std::endl;
    std::cout << "ICP恢复的平移: (" << recovered_transform(0,3) << ", " 
              << recovered_transform(1,3) << ", " << recovered_transform(2,3) << ") 米" << std::endl;
    std::cout << "补偿后残差 - 平移: " << translation_residual << " 米" << std::endl;
    std::cout << "补偿后残差 - 旋转: " << rotation_residual << " 度" << std::endl;
    std::cout << "噪声去除比例: " << noise_removal_ratio * 100 << "%" << std::endl;
    
    // 评估标准
    bool success = (translation_residual < 0.05) && (rotation_residual < 1.0);  // 5厘米, 1度
    
    std::cout << "去噪效果: " << (success ? "✓ 优秀" : "✗ 需要改进") << std::endl;
    std::cout << "ICP能够补偿 " << (noise_removal_ratio * 100) << "% 的初始噪声" << std::endl;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 初始化全局变量
    std::shared_ptr<pcl::PointCloud<PointType>> cloud_effected_timed;
    global_map_ikdtree = std::make_shared<KD_TREE<PointType>>();
    cloud_effected = std::make_shared<pcl::PointCloud<PointType>>();
    cloud_effected_timed = std::make_shared<pcl::PointCloud<PointType>>();
    

    std::string map_path="/home/dtc/rm_location/src/robots_localization/PCD/2025-08-18_09-54-00_scans.pcd";
    if (!loadPriorMap(map_path, true, 0.1, global_map_ikdtree)) {
        RCLCPP_ERROR(rclcpp::get_logger("post_mapper"), "Failed to load prior map, exiting");
        return -1;
    }
    
    auto node = std::make_shared<PointCloudProcessor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting main loop");
    
    // 主循环
    rclcpp::Rate rate(10);  // 10Hz
    
    while (rclcpp::ok()) {
        // 处理ROS回调
        executor.spin_some();
        
        // 检查是否有可用的点云数据
        if (!cloud_ready) {
            RCLCPP_INFO(node->get_logger(), "No-cloud");
            rate.sleep();
            continue;
        }
        *cloud_effected_timed=*cloud_effected;
        try {
            // 获取TF变换
            geometry_msgs::msg::TransformStamped raw_tf;
            try {
                raw_tf = node->tf_buffer_->lookupTransform(
                    "camera_init", 
                    "aft_mapped",
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1));
                    
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, 
                                   "TF lookup error: %s", ex.what());
                rate.sleep();
                continue;
            }
            
// 修改主循环中的局部地图提取部分
PointType center_point;
center_point.x = raw_tf.transform.translation.x;
center_point.y = raw_tf.transform.translation.y;
center_point.z = raw_tf.transform.translation.z;

// 保存真实中心点（无噪声）
PointType true_center = center_point;

// 添加噪声到中心点（模拟GPS误差）
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(-0.5, 0.5);

PointType noisy_center = true_center;
float noise_x = dis(gen);
float noise_y = dis(gen);
float noise_z = dis(gen);

noisy_center.x += noise_x;
noisy_center.y += noise_y;
noisy_center.z += noise_z;

// 计算噪声变换矩阵（这就是我们要去除的噪声）
M4F noise_transform = M4F::Identity();
noise_transform(0, 3) = noise_x;
noise_transform(1, 3) = noise_y;
noise_transform(2, 3) = noise_z;
// 提取旋转分量（四元数转换为旋转矩阵）
Eigen::Quaternionf quat(
    raw_tf.transform.rotation.w,
    raw_tf.transform.rotation.x,
    raw_tf.transform.rotation.y,
    raw_tf.transform.rotation.z
);

noise_transform.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix().inverse();



std::cout << "=== 噪声测试 ===" << std::endl;
std::cout << "真实中心: (" << true_center.x << ", " << true_center.y << ", " << true_center.z << ")" << std::endl;
std::cout << "带噪声中心: (" << noisy_center.x << ", " << noisy_center.y << ", " << noisy_center.z << ")" << std::endl;
// 提取局部地图（使用带噪声的中心点）
std::vector<PointType, Eigen::aligned_allocator<PointType>> points_near;
global_map_ikdtree->Radius_Search(noisy_center, 10, points_near);

// 创建局部点云（中心化 + 噪声）
pcl::PointCloud<PointType>::Ptr cloud_xyzi(new pcl::PointCloud<PointType>);
cloud_xyzi->width = points_near.size();
cloud_xyzi->height = 1;
cloud_xyzi->is_dense = true;
cloud_xyzi->points.resize(points_near.size());

for (size_t i = 0; i < points_near.size(); ++i) {
    cloud_xyzi->points[i].x = points_near[i].x - noisy_center.x ;
    cloud_xyzi->points[i].y = points_near[i].y - noisy_center.y ;
    cloud_xyzi->points[i].z = points_near[i].z - noisy_center.z ;
    cloud_xyzi->points[i].intensity = points_near[i].intensity;
}

// 执行ICP配准
M4F pos_mapped = mapper(cloud_effected_timed, cloud_xyzi);

// 计算真实的地图到扫描的变换（无噪声情况下的理想结果）
// M4F ground_truth_transform = calculateGroundTruthTransform(raw_tf);

// 评估配准性能
evaluateNoiseRemovalPerformance(noise_transform, pos_mapped, noise_x, noise_y, noise_z);



            std::cout<<"cloud_xyzi->points.size() ::"<<cloud_xyzi->points.size()<<"\n";
            std::cout<<"cloud_effected->points.size() ::"<<cloud_effected_timed->points.size()<<"\n";

            // // 执行ICP匹配
            // M4F pos_mapped = M4F::Zero();
            // {
            //     std::lock_guard<std::mutex> lock(cloud_mutex);
            //     if (!cloud_effected->empty()&&!cloud_xyzi->empty()) {
            //         pos_mapped=mapper(cloud_effected, cloud_xyzi);
            //     }
            // }

        // 转换为 sensor_msgs::msg::PointCloud2
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_effected, output);

        // 设置header，确保时间戳和帧ID正确
        output.header.stamp = node->get_clock()->now();
        output.header.frame_id = "camera_init";

        node->cloud_reg_pub_->publish(output);

        pcl::toROSMsg(*cloud_xyzi, output);

        // 设置header，确保时间戳和帧ID正确
        output.header.stamp = node->get_clock()->now();
        output.header.frame_id = "camera_init";

        node->cloud_subamap_pub_->publish(output);

            std::cout << "mapped pos:  " << std::endl
                << pos_mapped.block<3, 1>(0, 3) << std::endl
                << "mapped rot:  " << std::endl
                << pos_mapped.block<3, 3>(0, 0) << std::endl;
            std::cout<<global_map_ikdtree->size()<<"\n";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Exception in main loop: %s", e.what());
        }
        
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}