#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "types.h"

typedef pcl::PointXY PointType;
typedef pcl::PointXYZ PointType3;

MatrixSE2 ang2Mat(double ang);

/* convert 3d point cloud to 2d point cloud */
void point3d2point2d(const pcl::PointCloud<PointType3>::Ptr& cloud_in,
                     pcl::PointCloud<PointType>::Ptr& cloud_out,
                     double prune_dist = 0.0);

Eigen::Affine3d pose_stamped_to_eigen(
    const geometry_msgs::msg::PoseStamped& pose_msg);

void convertVecToInfoMat(const std::vector<double>& vec, Eigen::MatrixXd& mat);