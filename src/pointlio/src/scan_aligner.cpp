#pragma clang diagnostic push
#pragma ide diagnostic ignored "openmp-use-default-none"

#include "scan_aligner.h"
// using namespace common_lib;
// using namespace ikd_Tree;

ScanAligner::ScanAligner() {}

ScanAligner::~ScanAligner() {}

int ScanAligner::max_iter=15;
float ScanAligner::plane_dist=0.1;

std::pair<float, float> ScanAligner::init_icp_method(KD_TREE<PointType> &kdtree, PointCloudXYZI::Ptr scan,
                                                     M4F &predict_pose) {
    PointCloudXYZI::Ptr trans_cloud(new PointCloudXYZI());
    scan->width = scan->points.size();
    int knn_num = 1;
    M3F rotation_matrix;
    V3F translation;
    rotation_matrix = predict_pose.block<3, 3>(0, 0);
    translation = predict_pose.block<3, 1>(0, 3);
    float whole_dist = 0.0;
    float p_valid_proportion = 0.0;

    for (int iter = 0; iter < max_iter; iter++) {
        pcl::transformPointCloud(*scan, *trans_cloud, predict_pose);
        Eigen::Matrix<float, 6, 6> H;
        Eigen::Matrix<float, 6, 1> b;
        H.setZero();
        b.setZero();

        whole_dist = 0.0;
        int point_num = trans_cloud->size();
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (size_t i = 0; i < point_num; i++) {
            auto ori_point = scan->points[i];
            // pcl::isFinite()用来检查点云中是否有NaN值
            if (!pcl::isFinite(ori_point)) continue;
            auto trans_point = trans_cloud->points[i];
            std::vector<float> dist;
            // Eigen::aligned_allocator<PointType> 是一个内存分配器，它用于为存储在
            // std::vector 中的 PointType 对象分配内存。
            std::vector<PointType, Eigen::aligned_allocator<PointType>> points_near;
            // dist是临近点到搜索点trans_point的距离
            kdtree.Nearest_Search(trans_point, knn_num, points_near, dist);

            Eigen::Vector3f closest_point =
                Eigen::Vector3f(points_near[0].x, points_near[0].y, points_near[0].z);
            // 临近点距离的向量
            Eigen::Vector3f error_dist =
                Eigen::Vector3f(trans_point.x, trans_point.y, trans_point.z) - closest_point;

            Eigen::Matrix<float, 3, 6> J(Eigen::Matrix<float, 3, 6>::Zero());
            J.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
            J.block<3, 3>(0, 3) =
                -rotation_matrix * Sophus::SO3f::hat(Eigen::Vector3f(ori_point.x, ori_point.y, ori_point.z));
#ifdef MP_EN
#pragma omp critical
#endif
            {
                whole_dist += (trans_point.x - closest_point[0]) * (trans_point.x - closest_point[0]),
                    (trans_point.y - closest_point[1]) * (trans_point.y - closest_point[1]),
                    (trans_point.z - closest_point[2]) * (trans_point.z - closest_point[2]);
                H += J.transpose() * J;
                b += -J.transpose() * error_dist;
            }
        }

        // H.ldlt()返回一个对角线矩阵，它是用H的一个特征分解所得，solve(b)方法调用了对角线矩阵的求逆
        // H.ldlt().solve(b)是通过使用特征分解求线性方程组Hx=b
        Eigen::Matrix<float, 6, 1> delta_x = H.ldlt().solve(b);
        translation += delta_x.block<3, 1>(0, 0);
        rotation_matrix *= Sophus::SO3f::exp(delta_x.block<3, 1>(3, 0)).matrix();
        predict_pose.block<3, 3>(0, 0) = rotation_matrix;
        predict_pose.block<3, 1>(0, 3) = translation;
    }

    pcl::transformPointCloud(*scan, *trans_cloud, predict_pose);
    float p_num = (float)trans_cloud->size();
    float p_valid = 0.0;
    for (size_t i = 0; i < trans_cloud->size(); i++) {
        auto ori_point = scan->points[i];
        if (!pcl::isFinite(ori_point)) continue;
        auto trans_point = trans_cloud->points[i];
        std::vector<float> dist;
        std::vector<PointType, Eigen::aligned_allocator<PointType>> points_near;
        kdtree.Nearest_Search(trans_point, 1, points_near, dist);
        Eigen::Vector3f closest_point = Eigen::Vector3f(points_near[0].x, points_near[0].y, points_near[0].z);
        float p_dist = (trans_point.x - closest_point[0]) * (trans_point.x - closest_point[0]) +
                       (trans_point.y - closest_point[1]) * (trans_point.y - closest_point[1]) +
                       (trans_point.z - closest_point[2]) * (trans_point.z - closest_point[2]);
        if (p_dist < 0.05) {  // 根据地图分辨率
            p_valid += 1.0;
        }
    }
    p_valid_proportion = p_valid / p_num;

    std::cout << "whole dist: " << whole_dist << " ";
    std::cout << "p_valid_proportion: " << p_valid_proportion << std::endl;
    return std::make_pair(whole_dist, p_valid_proportion);
}

std::pair<float, float> ScanAligner::init_ppicp_method(KD_TREE<PointType> &kdtree, PointCloudXYZI::Ptr scan,
                                                       M4F &predict_pose) {
    PointCloudXYZI::Ptr trans_cloud(new PointCloudXYZI());
    scan->width = scan->points.size();
    M3F rotation_matrix;
    V3F translation;
    rotation_matrix = predict_pose.block<3, 3>(0, 0);
    translation = predict_pose.block<3, 1>(0, 3);
    float whole_dist = 0.0;
    float p_valid_proportion = 0.0;

    for (int iter = 0; iter < max_iter; iter++) {
        pcl::transformPointCloud(*scan, *trans_cloud, predict_pose);
        Eigen::Matrix<float, 6, 6> H;
        Eigen::Matrix<float, 6, 1> b;
        H.setZero();
        b.setZero();

        whole_dist = 0.0;
        int point_num = trans_cloud->size();
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (size_t i = 0; i < point_num; i++) {
            auto ori_point = scan->points[i];
            if (!pcl::isFinite(ori_point)) continue;
            auto trans_point = trans_cloud->points[i];
            std::vector<float> dist;
            std::vector<PointType, Eigen::aligned_allocator<PointType>> points_near;
            kdtree.Nearest_Search(trans_point, NUM_MATCH_POINTS, points_near, dist);

            VF(4)
            abcd;
            esti_plane(abcd, points_near, plane_dist);
            float error_dist =
                abcd[0] * trans_point.x + abcd[1] * trans_point.y + abcd[2] * trans_point.z + abcd[3];

            Eigen::Matrix<float, 1, 6> J(Eigen::Matrix<float, 1, 6>::Zero());
            J.block<1, 3>(0, 0) << abcd[0], abcd[1], abcd[2];
            Eigen::Matrix<float, 3, 3> tmp =
                -rotation_matrix * Sophus::SO3f::hat(Eigen::Vector3f(ori_point.x, ori_point.y, ori_point.z));
            J.block<1, 3>(0, 3) << abcd[0] * tmp(0, 0) + abcd[1] * tmp(1, 0) + abcd[2] * tmp(2, 0),
                abcd[0] * tmp(0, 1) + abcd[1] * tmp(1, 1) + abcd[2] * tmp(2, 1),
                abcd[0] * tmp(0, 2) + abcd[1] * tmp(1, 2) + abcd[2] * tmp(2, 2);
#ifdef MP_EN
#pragma omp critical
#endif
            {
                if (error_dist < 0.0) {
                    whole_dist += -error_dist;
                } else {
                    whole_dist += error_dist;
                }
                H += J.transpose() * J;
                b += -J.transpose() * error_dist;
            }
        }

        Eigen::Matrix<float, 6, 1> delta_x = H.ldlt().solve(b);
        translation += delta_x.block<3, 1>(0, 0);
        rotation_matrix *= Sophus::SO3f::exp(delta_x.block<3, 1>(3, 0)).matrix();
        predict_pose.block<3, 3>(0, 0) = rotation_matrix;
        predict_pose.block<3, 1>(0, 3) = translation;

        if (iter == max_iter - 1) {
            float p_num = (float)trans_cloud->size();
            float p_valid = 0.0;
            for (size_t i = 0; i < trans_cloud->size(); i++) {
                auto ori_point = scan->points[i];
                if (!pcl::isFinite(ori_point)) continue;
                auto trans_point = trans_cloud->points[i];
                std::vector<float> dist;
                std::vector<PointType, Eigen::aligned_allocator<PointType>> points_near;
                kdtree.Nearest_Search(trans_point, 1, points_near, dist);
                Eigen::Vector3f closest_point =
                    Eigen::Vector3f(points_near[0].x, points_near[0].y, points_near[0].z);
                float p_dist = (trans_point.x - closest_point[0]) * (trans_point.x - closest_point[0]) +
                               (trans_point.y - closest_point[1]) * (trans_point.y - closest_point[1]) +
                               (trans_point.z - closest_point[2]) * (trans_point.z - closest_point[2]);
                if (p_dist < 0.05) {  // 根据地图分辨率
                    p_valid += 1.0;
                }
            }
            p_valid_proportion = p_valid / p_num;
        }
    }

    // std::cout << "whole dist: " << whole_dist << " ";
    // std::cout << "p_valid_proportion: " << p_valid_proportion << std::endl;
    return std::make_pair(whole_dist, p_valid_proportion);
}

#pragma clang diagnostic pop