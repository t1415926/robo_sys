#include <omp.h>
#include <mutex>
#include <cmath>
#include <random>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>  // 非线性ICP
#include <pcl/registration/gicp.h>    // 广义ICP
/*

接收：
里程计去畸变点云话题 ：cloud_effected
gps数据话题        ：gps/data
先验地图           :
gps映射关系矩阵     ：

实现功能
1.  gps-》pos/KeyFrame
2.  Trnasfrom=icp（KeyFrame，：cloud_effected）
3.  LocalMap += GlobalMap*Trnasfrom 

*/


/*
需要维护的全局变量
全局地图
转换 T
局部地图？
*/

std::vector<double> local_yaw_range = {-M_PI, 0.315, M_PI}; 
KD_TREE<PointType> local_map_ikdtree;


bool icp(const PointCloudXYZI::Ptr cloud_effected, M4F& init_translasion
                        , KD_TREE<PointType>& kdtree, vector<double>& YAW_RANGE)
{
    /*
    输入 全局地图关键帧kdtree 里程计去畸变点云cloud_effected 输出匹配的位姿态
    用法 将地图关键帧 成kdtree 输入cloud_effected 得到 init_translasion
    需要将二者的点云进行归一化 中心置于0
    */
  // std::cout << "begin to match map " << std::endl;

  //初始化
    M4F icp_cur = M4F::Identity();
    M4F icp_last = M4F::Identity();
    M4F pose_trans_last = M4F::Identity();
    bool find_yaw=false;

    std::cout << "输入点云大小: " << cloud_effected->size() << std::endl;
    std::cout << "KD树大小: " << kdtree.size() << std::endl;
    std::cout << "YAW_RANGE大小: " << YAW_RANGE.size() << std::endl;

  ScanAligner aliner=ScanAligner();
  bool init_done = false;
  if (true)
  {
    double t1 = omp_get_wtime();
    float error_min = 1000000.0, validP_max = 0.0;
    M4F prior_with_min_error = M4F::Zero();
    if (find_yaw)
    {
      M4F prior_with_yaw = pose_trans_last;
      std::pair<float, float> result;
      result = aliner.init_ppicp_method(kdtree, cloud_effected, prior_with_yaw);
      error_min = result.first;
      validP_max = result.second;
      prior_with_min_error = prior_with_yaw;
    }
    else if (!find_yaw)
    {
#ifdef MP_EN
      omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)((YAW_RANGE[2] - YAW_RANGE[0]) / YAW_RANGE[1]); i++)
      {
        float yaw = YAW_RANGE[0] + i * YAW_RANGE[1];
        // std::cout << "iter: " << i << ", yaw: " << yaw << std::endl;

        float error = 0.0, validP = 0.0;
        std::pair<float, float> result;
        M4F prior_with_yaw = M4F::Zero();
        M3F rotation_yaw = M3F::Zero();
        rotation_yaw << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
        prior_with_yaw.block<3, 1>(0, 3) = pose_trans_last.block<3, 1>(0, 3);
        prior_with_yaw.block<3, 3>(0, 0) = rotation_yaw * pose_trans_last.block<3, 3>(0, 0);

        result = aliner.init_ppicp_method(kdtree,cloud_effected, prior_with_yaw);
        error = result.first;
        validP = result.second;

#ifdef MP_EN
#pragma omp critical
#endif
        {
          if (error < error_min)
          {
            error_min = error;
            prior_with_min_error = prior_with_yaw;
            validP_max = validP;
             std::cout << "error_min: " << error_min << std::endl;
            std::cout << "validP_max: " << validP_max << std::endl;
            // std::cout<<prior_with_min_error<<"\n";
          }
        }
      }
      if (validP_max > 0.3)
      {
        find_yaw = true;
      }
    }
    pose_trans_last = prior_with_min_error;
    icp_cur.block<3, 3>(0, 0) = prior_with_min_error.block<3, 3>(0, 0) * Lidar_R_wrt_IMU.inverse().cast<float>();
    icp_cur.block<3, 1>(0, 3) =
    prior_with_min_error.block<3, 1>(0, 3) - icp_cur.block<3, 3>(0, 0) * Lidar_T_wrt_IMU.cast<float>();

    double t2 = omp_get_wtime();
   
    // std::cout << "Init align time cost " << t2 - t1 << "s. " << std::endl
    //           << "Current pos:  " << std::endl
    //           << icp_cur.block<3, 1>(0, 3) << std::endl
    //           << "Current rot:  " << std::endl
    //           << icp_cur.block<3, 3>(0, 0) << std::endl
    //           << "Last pos:  " << std::endl
    //           << icp_last.block<3, 1>(0, 3) << std::endl
    //           << "Last rot:  " << std::endl
    //           << icp_last.block<3, 3>(0, 0) << std::endl
    //           << std::endl;

    V3F delta_rvec, delta_tvec;
    delta_rvec = rotationToEulerAngles(icp_cur.block<3, 3>(0, 0) * icp_last.block<3, 3>(0, 0).inverse());
    delta_tvec = icp_cur.block<3, 1>(0, 3) - icp_last.block<3, 1>(0, 3);
    if (delta_rvec.norm() < 0.1 && find_yaw)   //delta_tvec.norm() < 0.2 &&  初始化限制
    {
      init_done = true;
    }
    std::cout <<"delta_rvec.norm() :::"<<delta_rvec.norm()<< std::endl;
  }
  
  if(init_done){
    std::cout << "init pose sucess" << std::endl;
    init_translasion=icp_cur;
    return true;
  }
  std::cout << "init pose last change" << std::endl;
  icp_last = icp_cur;
  return false;
}

M4F mapper(const pcl::PointCloud<PointType>::Ptr& cloud_effected,const pcl::PointCloud<PointType>::Ptr& key_frames){
    /*
    坐标归一化后 调用icp得到转化坐标 主要是 key_frames 需要local化
    */
  //  std::cout<<"before trans\n";
    M4F init_translasion=M4F::Zero();
    if (key_frames->points.size()==0){
      std::cout<<"no points!!!!!\n";
      return init_translasion;
    }

    try
    {
      local_map_ikdtree.Build(key_frames->points);
      // std::cout<<"before icp\n";
      icp(cloud_effected,init_translasion,local_map_ikdtree,local_yaw_range);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    return init_translasion;
}


// void mapper(const pcl::PointCloud<PointType>::Ptr& cloud_effected,const pcl::PointCloud<PointType>::Ptr& key_frames, M4F& init_translasion){
//     /*
//     坐标归一化后 调用icp得到转化坐标 主要是key_frames 需要local化
//     */
//    std::cout<<"before trans\n";
//   pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_icp;
//     m_icp.setMaximumIterations(50);
//     m_icp.setMaxCorrespondenceDistance(10);
//     m_icp.setTransformationEpsilon(1e-5);
//     m_icp.setEuclideanFitnessEpsilon(1e-5);
//     // m_icp.setRANSACIterations(0);
//     // pcl::PointCloud<PointType>::Ptr key_frames_transed(new pcl::PointCloud<PointType>()); 
//   pcl::PointCloud<PointType>::Ptr align_cloud(new pcl::PointCloud<PointType>());
    
//     m_icp.setInputSource(key_frames);
//     m_icp.setInputTarget(cloud_effected);
//   // 2. 检查点云是否为空
//   if (cloud_effected->empty() || key_frames->empty()) {
//       std::cout << "点云为空，跳过ICP" << std::endl;
//       return;
//   }

//     m_icp.align(*align_cloud);

//     // 修复：在调用getFitnessScore()之前检查点云状态
//     if (align_cloud->empty()) {
//         std::cout << "配准后点云为空，跳过分数计算" << std::endl;
//         return;
//     }

//     if ( m_icp.getFitnessScore() > 0.8){ //!m_icp.hasConverged() ||
//       std::cout << "配准失败: " << m_icp.getFitnessScore()<< std::endl;  
//       return;
//     }


//     init_translasion = m_icp.getFinalTransformation();
    
//     // KD_TREE<PointType> local_map_ikdtree;
//     // // for (const auto& point : key_frames->points) {

//     // //     PointType court_point;
//     // //     court_point.x = point.x;
//     // //     court_point.y = point.y;
//     // //     court_point.z = point.z;
//     // //     key_frames_transed->points.push_back(court_point);
//     // // }
//     // if (key_frames->points.size()==0){
//     //   std::cout<<"no points!!!!!\n";
//     //   return;
//     // }
//     // local_map_ikdtree.Build(key_frames->points);
//     // std::cout<<"before icp\n";
//     // icp(cloud_effected,init_translasion,local_map_ikdtree,yaw_range);

// }




void key_frame_generate(M4F& ref_pos,PointCloudXYZI::Ptr& key_frames){
    /*
    根据 gps 全局地图 生成用于匹配的关键帧
    需要输入 gps映射关系矩阵 全局地图 gps数据
    */
    

}

void sub_map_generate(M4F& ref_pos){
    /*
    根据位置 生成子图
    */

}


void map_update(){
    /*
    更新局部地图和全局地图
    局部地图 ： 里程计的地图 Estimator/ikdtree
    全局地图 :  先验地图+局部地图*T
    */
}












