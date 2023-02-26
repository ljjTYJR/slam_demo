#pragma once

// cunstomed
#include "types.h"
#include "odomter.h"
#include "keyframe.h"
#include "pose_graph.h"
#include "scan_context.h"
#include "visualization.h"
#include "helper.h"

// Third party
#include <Eigen/Dense>

// standard C/C++
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud.hpp>

class Slam {

public:
    Slam(const rclcpp::Node::SharedPtr node);
    ~Slam() {};

    void init();

    void mainLoop();

private:
    /* Types Declarations */
    using LaserSub              =   message_filters::Subscriber<sensor_msgs::msg::LaserScan>;
    using WheelOdomSub          =   message_filters::Subscriber<nav_msgs::msg::Odometry>;
    using LaserWheelSyncPolicy  =   message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>;

    /* The internal functions */
    void declareParameters();
    void loadParameters();
    void advertisePublishers();
    void registerSubscribers();
    void laserWheelOdomSyncCallback(const sensor_msgs::msg::LaserScan::ConstPtr& laser_msg,
                                    const nav_msgs::msg::Odometry::ConstPtr& wheel_odom_msg);
    bool optTopicCallback(const std_msgs::msg::String::SharedPtr msg);

    /* Private data */
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             opt_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr opt_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    res_point_cloud_pub_;
    LaserSub *laser_sub_;
    WheelOdomSub *wheel_odom_sub_;
    message_filters::Synchronizer<LaserWheelSyncPolicy> *sync_wheelOdom_laser_sub_;
    // NOTE: For subscribe to the optimization topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_opt_topic_;

    std::string sub_wheel_odom_topic_;
    std::string sub_laser_topic_;
    std::string pub_opt_path_topic_;
    std::string opt_path_frame_;
    std::string pub_opt_pose_topic_;
    std::string opt_pose_frame_;
    std::string res_point_cloud_topic_;

    Eigen::MatrixXd laser_odom_inf_matrix_;
    Eigen::MatrixXd loop_inf_matrix_;

    Eigen::Matrix3d se2_info_laser_mat_;

    // TODO: change the class object to the pointer
    Odometer odom_;
    PoseGraph pose_graph_;
    ScanContextManger scan_context_manger_;
    Visualization visualization_;

    std::vector<MatrixSE2> optimized_pose_;
};
