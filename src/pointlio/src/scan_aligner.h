#pragma once

#include <common_lib.h>
#include <omp.h>
#include <pcl/common/transforms.h>

#include <fstream>
#include <sophus/so3.hpp>

#include "ikd-Tree/ikd_Tree.h"

// using namespace common_lib;
// using namespace ikd_Tree;

class ScanAligner {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ScanAligner();

    ~ScanAligner();

    static int max_iter;
    static float plane_dist;

    static std::pair<float, float> init_icp_method(KD_TREE<PointType> &kdtree, PointCloudXYZI::Ptr scan,
                                                   M4F &predict_pose);
    static std::pair<float, float> init_ppicp_method(KD_TREE<PointType> &kdtree, PointCloudXYZI::Ptr scan,
                                                     M4F &predict_pose);
};