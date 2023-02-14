#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include <iostream>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

// libpointmatcher
#include "pointmatcher/PointMatcher.h"

typedef struct
{
    double theta;
    Eigen::Vector2d t;

} state2d;

typedef Eigen::Matrix<double, 3, 3> Matrix3d;

class Odometry {
public:
    Odometry();
    virtual ~Odometry();



private:
    state2d cur_pose_;
    state2d delta_pose_;


};

#endif /* ODOMETRY_HPP_ */