#include "types.h"
#include "odomter.h"
#include "keyframe.h"
#include "slam.h"

#include <Eigen/Dense>

Slam::Slam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      key_frame_num_(4),
      odom_(nh, pnh),
      laser_odom_inf_matrix_(Eigen::MatrixXd::Identity(3, 3)),
      loop_inf_matrix_(Eigen::MatrixXd::Identity(3, 3))
       {
    init();
}

void Slam::init() {
    ROS_INFO("Slam::init()");
    initParameters();
    advertisePublishers();
    // registerSubscribers();

    mainLoop();

    return;
}

void Slam::initParameters() {
    for (int i = 0; i < laser_odom_inf_matrix_.rows(); ++i) {
        laser_odom_inf_matrix_(i, i) = 0.1;
    }

    pnh_.param("opt_pose_topic_", opt_pose_topic_, std::string("/opt_pose"));
    // todo: map frame?
    pnh_.param("opt_pose_frame_", opt_pose_frame_, std::string("/odom"));
    return;
}

void Slam::advertisePublishers() {
    opt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(opt_pose_topic_, 100);
    return;
}

void Slam::mainLoop() {

    return;
}
