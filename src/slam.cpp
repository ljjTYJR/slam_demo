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
      visualization_(nh),
      laser_odom_inf_matrix_(Eigen::MatrixXd::Identity(3, 3)),
      loop_inf_matrix_(Eigen::MatrixXd::Identity(3, 3)) {
    init();
}

void Slam::init() {
    ROS_INFO("Slam::init()");
    initParameters();
    advertisePublishers();
    registerSubscribers();
    registerServices();
    return;
}

void Slam::initParameters() {

    for (int i = 0; i < laser_odom_inf_matrix_.rows(); ++i) {
        laser_odom_inf_matrix_(i, i) = 0.1;
    }
    pnh_.param("sub_wheel_odom_topic_", sub_wheel_odom_topic_, std::string("wheel_odom"));
    pnh_.param("sub_laser_topic_", sub_laser_topic_, std::string("laser_topic"));
    pnh_.param("pub_opt_path_topic_", pub_opt_path_topic_, std::string("/opt_path"));
    pnh_.param("opt_path_frame_", opt_path_frame_, std::string("/odom"));
    pnh_.param("pub_opt_pose_topic_", pub_opt_pose_topic_, std::string("/opt_pose"));
    pnh_.param("opt_pose_frame_", opt_pose_frame_, std::string("/odom"));
    return;
}

void Slam::advertisePublishers() {
    opt_path_pub_ = nh_.advertise<nav_msgs::Path>(pub_opt_path_topic_, 100);
    opt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pub_opt_pose_topic_, 100);
    return;
}

/**
 * recivie the input sensor messgaes and process them
*/
void Slam::laserWheelOdomSyncCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg, const nav_msgs::Odometry::ConstPtr& wheel_odom_msg) {
    std::tuple<bool, unsigned int, pcl::PointCloud<pcl::PointXY>::Ptr> odom_res = odom_.odomDealWithInputMessage(laser_msg, wheel_odom_msg);
    if (std::get<0>(odom_res) == true) {
        ROS_INFO("The newest keyframe is %d", std::get<1>(odom_res));
        std::pair<int, double> sc_res = scan_context_manger_.addNewFrame(std::get<1>(odom_res), std::get<2>(odom_res));
        if (sc_res.first != -1) /* Find the loop closure */ {
            ROS_INFO("Find the loop closure, the corresponding keyframe id is %d", sc_res.first);

            int cur_id = std::get<1>(odom_res);
            int loop_id = sc_res.first;
            ROS_INFO("cur_id: %d, loop_id: %d", cur_id, loop_id);

            // cur_id is the size of the key_frames_buffer_, the index should be cur_id - 1
            visualization_.publishLineOfTwoPoses(odom_.key_frames_buffer_[cur_id]->getPose(), odom_.key_frames_buffer_[loop_id]->getPose(), opt_path_frame_, 10000);
        }
    }
    return;
}

void Slam::registerSubscribers() {
    wheel_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(pnh_, sub_wheel_odom_topic_, 100);
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_, sub_laser_topic_, 100);
    sync_wheelOdom_laser_sub_ = new message_filters::Synchronizer<LaserOdomSync>(LaserOdomSync(100), *laser_sub_, *wheel_odom_sub_);
    sync_wheelOdom_laser_sub_->registerCallback(boost::bind(&Slam::laserWheelOdomSyncCallback, this, _1, _2));
    ROS_INFO("Subscribed to %s and %s", sub_wheel_odom_topic_.c_str(), sub_laser_topic_.c_str());
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