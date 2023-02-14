#ifndef __HELPER_H__
#define __HELPER_H__

#include <Eigen/Eigen>
#include <math.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXY PointType;
typedef pcl::PointXYZ PointType3;

using namespace std;

/**
 * Convert a 2d rotation matrix to angle
 */
double rotToAngle(Eigen::Matrix2d r) {
    double cos_theta = r(0, 0);
    double sin_theta = r(1, 0);
    return atan2(sin_theta, cos_theta);
}

/**
 * Convert a 2d rotation matrix to angle(deg)
 */
double rotToAngleDeg(Eigen::Matrix2d r) {
    double cos_theta = r(0, 0);
    double sin_theta = r(1, 0);
    double rad = atan2(sin_theta, cos_theta);
    double deg = (rad / M_PI) * 180;
    if (deg < 0) {
        deg += 360;
    }
    return deg;
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


#endif