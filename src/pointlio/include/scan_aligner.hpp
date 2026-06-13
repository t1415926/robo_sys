#pragma once

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

class ScanAligner {
public:
    enum Method { ICP, NDT, GICP };
    
    ScanAligner(Method method = GICP, int max_iter = 10) 
        : method_(method), max_iter_(max_iter) {
        initAligner();
    }
    
    void setMethod(Method method) {
        method_ = method;
        initAligner();
    }
    
    void setMaxIterations(int max_iter) {
        max_iter_ = max_iter;
        aligner_->setMaximumIterations(max_iter_);
    }
    
    bool align(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
               Eigen::Matrix4f& transformation) {
        if (!aligner_) return false;
        
        aligner_->setInputSource(source);
        aligner_->setInputTarget(target);
        
        pcl::PointCloud<pcl::PointXYZI> aligned;
        aligner_->align(aligned);
        
        if (aligner_->hasConverged()) {
            transformation = aligner_->getFinalTransformation();
            return true;
        }
        return false;
    }
    
private:
    void initAligner() {
        switch (method_) {
        case ICP:
            aligner_ = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
            break;
        case NDT:
            aligner_ = std::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
            break;
        case GICP:
            aligner_ = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
            break;
        }
        if (aligner_) {
            aligner_->setMaximumIterations(max_iter_);
        }
    }
    
    Method method_;
    int max_iter_;
    std::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> aligner_;
};