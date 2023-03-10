#pragma once
// customized
#include "types.h"
// ROS2
#include "rclcpp/rclcpp.hpp"
// third party
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class KeyFrame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<KeyFrame>;

    KeyFrame(const unsigned int& id,
             const MatrixSE2& pose,
             const MatrixSE2& relative_measure,
             const pcl::PointCloud<pcl::PointXY>::Ptr& cloud)
        : id_(id),
          pose_(pose),
          relative_measure_(relative_measure),
          cloud_(cloud){};

    ~KeyFrame(){};

    unsigned int getId() const { return id_; }
    MatrixSE2 getPose() const { return pose_; }
    MatrixSE2 getRelativeMeasure() const { return relative_measure_; }
    pcl::PointCloud<pcl::PointXY>::Ptr getCloud() const { return cloud_; }

   private:
    unsigned int id_;
    MatrixSE2 pose_;
    MatrixSE2
        relative_measure_;  // the relative measure from the previous key frame
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_;
};
