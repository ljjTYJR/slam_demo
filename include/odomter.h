#ifndef __ODOMETER_H
#define __ODOMETER_H

// @brief: header file for odometer class
// @description: The odomter class is used to record the odom information and laser scan
#include "types.h"
#include "keyframe.h"

#include <iostream>
#include <string>
#include <tuple>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

class Odometer
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
    Odometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~Odometer();

    /* the odometer, to deal with the input message; the input can be wheel_odom or laser_msg or both */
    std::tuple<bool, unsigned int, pcl::PointCloud<pcl::PointXY>::Ptr> odomDealWithInputMessage(
        const sensor_msgs::LaserScan::ConstPtr& laser_msg,
        const nav_msgs::Odometry::ConstPtr& wheel_odom_msg);
    void odomDealWithInputMessage(const nav_msgs::Odometry::ConstPtr& wheel_odom_msg);
    void odomDealWithInputMessage(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

    void init();
    void loadParameters();
    void advertisePublishers();
    void registerSubscribers();
    void readInLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void readInWheelOdom(const nav_msgs::Odometry::ConstPtr& wheel_odom_msg);
    bool updateOdom();
    MatrixSE2 icpPointMatch(const pcl::PointCloud<pcl::PointXY>::Ptr& prev_scan,
                            const pcl::PointCloud<pcl::PointXY>::Ptr& cur_scan, const MatrixSE2& guess);
    void publishPose();
    void publishLaser(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void point3d2Point2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXY>::Ptr& cloud_out);

public:
    std::vector<MatrixSE2> wheel_odom_mem_;
    std::vector<MatrixSE2> laser_relative_pose_mem_;
    std::vector<std::shared_ptr<KeyFrame> > key_frames_buffer_;

private:

    void addNewKeyFrame(const MatrixSE2& pose, const MatrixSE2& relative_measure, const pcl::PointCloud<pcl::PointXY>::Ptr& cloud);
    bool transLargeEnough(const MatrixSE2& pose);

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    const double kMinDist_ = 0.30; //0.20m
    const double kMinRot_ = 3.0 * M_PI / 180.0; //2.5 degree

    // frames
    std::string laser_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    // topics:publish
    std::string pub_pose_topic_;
    std::string pub_laser_topic_;
    std::string pub_path_topic_;
    // flags
    bool use_wheel_odom_;
    bool use_laser_;
    bool use_darko_cfg_;
    bool use_wheel_odom_prior_guess_;
    Eigen::Affine3d sensor_offset_;

    // advertisers and subscribers
    ros::Publisher odom_pub_;
    ros::Publisher laser_pub_;
    ros::Publisher path_pub_;
    ros::Time timer_;

    // The latest data record
    MatrixSE2 latest_odom_;   //The latest pose of the robot in the map frame;
    MatrixSE2 prev_wheel_odom_;   //The latest pose of the robot in the map frame;
    MatrixSE2 prev_wheel_key_odom_;
    pcl::PointCloud<pcl::PointXY>::Ptr latest_scan_;
    pcl::PointCloud<pcl::PointXY>::Ptr prev_scan_;
    std::vector<MatrixSE2> odom_mem_;
    bool set_the_first_pose_;


};


#endif // __ODOMETER_H