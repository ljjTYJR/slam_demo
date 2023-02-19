#ifndef __SLAM_H
#define __SLAM_H

#include "types.h"
#include "odomter.h"
#include "keyframe.h"
#include "pose_graph.h"
#include "slam_demo/OptSrv.h"
#include "scan_context.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>

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

    bool optimize_signal_callback(slam_demo::OptSrv::Request& req, slam_demo::OptSrv::Response& res);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // publish the optimized pose
    ros::Publisher opt_path_pub_;
    ros::Publisher opt_pose_pub_;
    std::string opt_path_topic_;
    std::string opt_path_frame_;
    std::string opt_pose_topic_;
    std::string opt_pose_frame_;

    // todo: better way to maintain the keyframes
    unsigned int key_frame_num_;

    Eigen::MatrixXd laser_odom_inf_matrix_;
    Eigen::MatrixXd loop_inf_matrix_;

    // todo: confuration of parameters
    Eigen::Matrix3d se2_info_laser_mat_;

    ros::ServiceServer opt_server_;

    // Todo: change the class object to the pointer
    Odometer odom_;
    PoseGraph pose_graph_;
    ScanContextManger scan_context_manager_;

    std::vector<MatrixSE2> optimized_pose_;
};

#endif