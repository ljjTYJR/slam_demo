#ifndef __SLAM_H
#define __SLAM_H

#include "types.h"
#include "odomter.h"
#include "keyframe.h"
#include "pose_graph.h"
#include "slam_demo/OptSrv.h"
#include "scan_context.h"
#include "visualization.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>

using LaserOdomSync = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry>;

class Slam {

public:
    Slam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~Slam() {};

    void init();

    void initParameters();

    void advertisePublishers();

    void registerSubscribers();

    void registerServices();

    void mainLoop();


private:
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;

    /**
     * The callback functions to deal with the input signal;
     * Get the offline optimization signal;
     * Get the laser scan and wheel odometry message;
     */
    bool optimize_signal_callback(slam_demo::OptSrv::Request& req, slam_demo::OptSrv::Response& res);
    void laserWheelOdomSyncCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg, const nav_msgs::Odometry::ConstPtr& wheel_odom_msg);

    /**
     * The parameters of the topics mangers
    */
    ros::Publisher  opt_path_pub_;
    ros::Publisher  opt_pose_pub_;
    ros::Publisher  res_point_cloud_pub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>     *wheel_odom_sub_;
    message_filters::Synchronizer<LaserOdomSync>        *sync_wheelOdom_laser_sub_;
    std::string sub_wheel_odom_topic_;
    std::string sub_laser_topic_;
    std::string pub_opt_path_topic_;
    std::string opt_path_frame_;
    std::string pub_opt_pose_topic_;
    std::string opt_pose_frame_;
    std::string res_point_cloud_topic_;


    Eigen::MatrixXd laser_odom_inf_matrix_;
    Eigen::MatrixXd loop_inf_matrix_;

    // todo: confuration of parameters
    Eigen::Matrix3d se2_info_laser_mat_;

    ros::ServiceServer opt_server_;

    // Todo: change the class object to the pointer
    Odometer odom_;
    PoseGraph pose_graph_;
    ScanContextManger scan_context_manger_;
    Visualization visualization_;

    std::vector<MatrixSE2> optimized_pose_;
};

#endif