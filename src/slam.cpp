#include "types.h"
#include "odomter.h"
#include "keyframe.h"
#include "slam.h"
#include "slam_demo/OptSrv.h"

#include <Eigen/Dense>
#include <ros/ros.h>
#include <unistd.h>

Slam::Slam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      odom_(nh, pnh),
      laser_odom_inf_matrix_(Eigen::MatrixXd::Identity(3, 3)),
      key_frame_num_(4),
      loop_inf_matrix_(Eigen::MatrixXd::Identity(3, 3)) {
    ROS_INFO("Slam::Slam()");
    init();
}

void Slam::init() {
    ROS_INFO("Slam::init()");
    initParameters();
    advertisePublishers();
    registerSubscribers();
    registerServices();
    mainLoop();

    return;
}

void Slam::initParameters() {
    for (int i = 0; i < laser_odom_inf_matrix_.rows(); ++i) {
        laser_odom_inf_matrix_(i, i) = 0.1;
    }

    pnh_.param("opt_path_topic_", opt_path_topic_, std::string("/opt_path"));
    pnh_.param("opt_path_frame_", opt_path_frame_, std::string("/odom"));
    pnh_.param("opt_pose_topic_", opt_pose_topic_, std::string("/opt_pose"));
    pnh_.param("opt_pose_frame_", opt_pose_frame_, std::string("/odom"));
    return;
}

void Slam::advertisePublishers() {
    opt_path_pub_ = nh_.advertise<nav_msgs::Path>(opt_path_topic_, 100);
    opt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(opt_pose_topic_, 100);
    return;
}

void Slam::registerSubscribers() {
    return;
}

bool Slam::optimize_signal_callback(slam_demo::OptSrv::Request& req, slam_demo::OptSrv::Response& res) {
    ROS_INFO("Slam::optimize_signal_callback()");
    g2o::VertexSE2* prev_node;
    g2o::VertexSE2* cur_node;
    // add vertices of the keyframes
    for (int i = 0; i < odom_.key_frames_buffer_.size(); ++i) {
        // the pose is added as the vertex
        if (i == 0) {
            cur_node = pose_graph_.addSE2Node(odom_.key_frames_buffer_[i]->getPose(), true);
        } else {
            cur_node = pose_graph_.addSE2Node(odom_.key_frames_buffer_[i]->getPose(), false);
            pose_graph_.addSE2Edge(prev_node, cur_node, odom_.key_frames_buffer_[i]->getRelativeMeasure(), laser_odom_inf_matrix_);
        }
        prev_node = cur_node;
    }
    // save the optimized pose
    pose_graph_.saveGraph("/home/ros/catkin_ws/src/darko/slam_demo/launch/before_path.g2o");
    pose_graph_.optimize(15);

    // publish the optimized pose
    ROS_INFO("The number of vertices: %d", pose_graph_.optimizer_->vertices().size());
    nav_msgs::Path path;
    for (unsigned int i = 0; i < pose_graph_.optimizer_->vertices().size(); ++i) {
        // get the estimated pose of each vertex
        auto vertex = pose_graph_.optimizer_->vertices().at(i);
        auto estimate = dynamic_cast<g2o::VertexSE2*>(vertex)->estimate();
        // convert the VertexSE2 pose to the matrix
        auto position = estimate.translation();
        auto orientation = estimate.rotation().angle();

        // publish the optimized pose
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = opt_pose_frame_;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = 0.0;
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
        // need to wait some time?
        path.poses.push_back(pose_msg);
    }
    path.header.stamp = ros::Time::now();
    path.header.frame_id = opt_path_frame_;
    opt_path_pub_.publish(path);
    ROS_INFO("Publish the optimized path");
    return true;
}

void Slam::registerServices() {
    opt_server_ = nh_.advertiseService("optimize_service", &Slam::optimize_signal_callback, this);
    return;
}

void Slam::mainLoop() {
    ROS_INFO("Enter the main loop of slam");
    return;
}
