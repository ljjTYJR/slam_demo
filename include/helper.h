#pragma once

#include <Eigen/Eigen>
#include <math.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "types.h"

typedef pcl::PointXY PointType;
typedef pcl::PointXYZ PointType3;

MatrixSE2 ang2Mat(double ang) {
    MatrixSE2 mat;
    mat(0, 0) = cos(ang);
    mat(0, 1) = -sin(ang);
    mat(1, 0) = sin(ang);
    mat(1, 1) = cos(ang);
    mat(0, 2) = 0;
    mat(1, 2) = 0;
    mat(2, 0) = 0;
    return mat;
}

/* convert 3d point cloud to 2d point cloud */
void point3d2point2d(const pcl::PointCloud<PointType3> &cloud_in, pcl::PointCloud<PointType> &cloud_out) {
    cloud_out.points.resize(cloud_in.points.size());
    for (unsigned int i = 0; i < cloud_in.points.size(); ++i) {
        cloud_out.points[i].x = cloud_in.points[i].x;
        cloud_out.points[i].y = cloud_in.points[i].y;
    }
    return;
}

/* convert `geometry_msgs::msg::TransformStamped` to an Eigen Mat */
