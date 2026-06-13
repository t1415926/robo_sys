#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ikd-Tree/ikd_Tree.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MapManager {
public:
    using PointT = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<PointT>;
    using KdTreePtr = std::shared_ptr<KD_TREE<PointT>>;

    explicit MapManager(rclcpp::Node::SharedPtr node) : node_(node) {
        // 获取参数
        node_->declare_parameter<std::string>("map_path", "");
        node_->declare_parameter<double>("filter_size_map", 0.5);
        
        // 处理相对路径
        auto map_path = node_->get_parameter("map_path").as_string();
        if (map_path.empty()) {
            auto pkg_path = ament_index_cpp::get_package_share_directory("prior_map_localization_ros2");
            map_path_ = pkg_path + "/maps/prior_map.pcd";
        } else {
            map_path_ = map_path;
        }
        
        filter_size_map_ = node_->get_parameter("filter_size_map").as_double();
    }
    
    void loadMap() {
        // 加载PCD地图文件
        PointCloud::Ptr map_cloud(new PointCloud);
        if (pcl::io::loadPCDFile<PointT>(map_path_, *map_cloud) == -1) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load map file: %s", map_path_.c_str());
            return;
        }
        
        // 构建ikd-Tree
        map_tree_ = std::make_shared<KD_TREE<PointT>>();
        map_tree_->set_downsample_param(filter_size_map_);
        map_tree_->Build(map_cloud->points);
        
        RCLCPP_INFO(node_->get_logger(), "Map loaded successfully with %d points", 
                   static_cast<int>(map_cloud->size()));
    }
    
    KdTreePtr getMapTree() const { return map_tree_; }

private:
    rclcpp::Node::SharedPtr node_;
    std::string map_path_;
    double filter_size_map_;
    KdTreePtr map_tree_;
};