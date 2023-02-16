#ifndef __KEY_FRAME_H
#define __KEY_FRAME_H

#include "types.h"

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>


class KeyFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<KeyFrame>;

    KeyFrame(const unsigned int& id, const ros::Time& stamp, const MatrixSE2& pose, const pcl::PointCloud<pcl::PointXY>::Ptr& cloud)
    : id_(id), stamp_(stamp), pose_(pose), cloud_(cloud), vertex_(nullptr) {};

    ~KeyFrame();

    unsigned int getId() const { return id_; }
    ros::Time getStamp() const { return stamp_; }
    MatrixSE2 getPose() const { return pose_; }
    pcl::PointCloud<pcl::PointXY>::Ptr getCloud() const { return cloud_; }
    std::shared_ptr<g2o::VertexSE2> getVertex() const { return vertex_; }

private:
    unsigned int id_;
    ros::Time stamp_;
    MatrixSE2 pose_;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_;
    std::shared_ptr<g2o::VertexSE2> vertex_;
};

#endif