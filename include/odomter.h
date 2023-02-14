#ifndef __ODOMETER_H
#define __ODOMETER_H

// @brief: header file for odometer class
// @description: The odomter class is used to record the odom information and laser scan
#include <ros/ros.h>
#include <string>
#include <Eigen/Eigen>

#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef Eigen::Matrix<double, 3, 3> MatrixSE2;

class Odometer
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
    Odometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~Odometer();

    void init();

    void loadParameters();

    void advertisePublishers();

    void registerSubscribers();

    void laserWheelOdomSyncCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg, const nav_msgs::Odometry::ConstPtr& wheel_odom_msg);
    void readInLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void readInWheelOdom(const nav_msgs::Odometry::ConstPtr& wheel_odom_msg);

    void updateOdom();
    MatrixSE2 icpPointMatch(const pcl::PointCloud<pcl::PointXY>::Ptr& prev_scan,
                            const pcl::PointCloud<pcl::PointXY>::Ptr& cur_scan, const MatrixSE2& guess);

    void publishPose();
    void publishLaser(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

    void point3d2Point2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXY>::Ptr& cloud_out);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    // The publisher and subscriber

    // Parameters
    // frames
    std::string laser_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    // topics:subscribers
    std::string wheel_odom_topic_;
    std::string laser_topic_;
    // topics:publish
    std::string pose_topic_pub_;
    std::string laser_topic_pub_;
    std::string path_topic_pub_;
    // flags
    bool use_wheel_odom_;
    bool use_laser_;
    bool use_darko_cfg_;
    bool use_wheel_odom_prior_guess_;
    Eigen::Affine3d sensor_offset_;

    // advertisers and subscribers
    // ros::Subscriber wheel_odom_sub_;
    // ros::Subscriber laser_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>     *wheel_odom_sub_;
    message_filters::Synchronizer<LaserOdomSync>        *sync_wheelOdom_laser_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher laser_pub_;
    ros::Publisher path_pub_;
    ros::Time timer_;

    // data
    MatrixSE2 odom_;
    MatrixSE2 cur_wheel_odom_;
    MatrixSE2 prev_wheel_odom_;
    MatrixSE2 latest_transform_;
    std::vector<MatrixSE2> wheel_odom_mem_;
    std::vector<MatrixSE2> odom_mem_;

    bool odom_initialized_;

    // point cloud
    pcl::PointCloud<pcl::PointXY>::Ptr cur_scan_;
    pcl::PointCloud<pcl::PointXY>::Ptr prev_scan_;
};


#endif // __ODOMETER_H