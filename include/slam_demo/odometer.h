#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/multi_echo_laser_scan.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tuple>

#include "helper.h"
#include "keyframe.h"
#include "rclcpp/rclcpp.hpp"
#include "types.h"

class Odometer {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   public:
    Odometer(const rclcpp::Node::SharedPtr node);
    ~Odometer();

    /* the odometer, to deal with the input message; the input can be wheel_odom
     * or laser_msg or both */
    std::tuple<bool, unsigned int, pcl::PointCloud<pcl::PointXY>::Ptr> odomDealWithInputMessage(
        const sensor_msgs::msg::LaserScan::ConstPtr& laser_msg,
        const nav_msgs::msg::Odometry::ConstPtr& wheel_odom_msg);
    void odomDealWithInputMessage(const nav_msgs::msg::Odometry::ConstPtr wheel_odom_msg);
    void odomDealWithInputMessage(const sensor_msgs::msg::LaserScan::ConstPtr laser_msg);

    void readInLaserScan(const sensor_msgs::msg::LaserScan::ConstPtr& laser_msg);
    void readInWheelOdom(const nav_msgs::msg::Odometry::ConstPtr wheel_odom_msg);
    MatrixSE2 icpPointMatch(const pcl::PointCloud<pcl::PointXY>::Ptr& prev_scan,
                            const pcl::PointCloud<pcl::PointXY>::Ptr& cur_scan,
                            const MatrixSE2& guess);

   public:
    std::vector<MatrixSE2> wheel_odom_mem_;
    std::vector<MatrixSE2> laser_relative_pose_mem_;
    std::vector<std::shared_ptr<KeyFrame>> key_frames_buffer_;

   private:
    /* internal functions */
    void init();
    void declareParameters();
    void loadParameters();
    Eigen::Isometry3d getSensorOffset();
    void advertisePublishers();
    void registerSubscribers();

    void addNewKeyFrame(const MatrixSE2& pose,
                        const MatrixSE2& relative_measure,
                        const pcl::PointCloud<pcl::PointXY>::Ptr& cloud);
    bool transLargeEnough(const MatrixSE2& pose);
    bool updateOdom();
    void publishPosePathAndPointCloud();

    /* private data */
    rclcpp::Node::SharedPtr node_;

    const double kMinDist_ = 0.30;               // 0.20m
    const double kMinRot_ = 3.0 * M_PI / 180.0;  // 2.5 degree
    // parameters get from yaml file
    double farthest_point_dist = 10.0;
    // frames
    std::string sensor_link;
    std::string map_link;
    std::string robot_link;
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Time timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_map_to_robot_;

    // The latest data record
    MatrixSE2 latest_odom_;      // The latest pose of the robot in the map frame;
    MatrixSE2 prev_wheel_odom_;  // The latest pose of the robot in the map frame;
    MatrixSE2 prev_wheel_key_odom_;
    pcl::PointCloud<pcl::PointXY>::Ptr latest_scan_;
    pcl::PointCloud<pcl::PointXY>::Ptr prev_scan_;
    std::vector<MatrixSE2> odom_mem_;
    bool set_the_first_pose_;
};