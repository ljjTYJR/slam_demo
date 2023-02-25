// customed
#include "slam.h"
// Third party
#include <Eigen/Dense>
#include <boost/bind.hpp>
#include <boost/function.hpp>
// standard C/C++
#include <unistd.h>
#include <memory>

Slam::Slam(const rclcpp::Node::SharedPtr node)
    : node_(node),
      odom_(node),
      visualization_(node),
      laser_odom_inf_matrix_(Eigen::MatrixXd::Identity(3, 3)),
      loop_inf_matrix_(Eigen::MatrixXd::Identity(3, 3))
{
    init();
}

void Slam::init() {
    RCLCPP_INFO(node_->get_logger(), "Slam: All other modules initialized successfully, begin to initialize the Slam node module");
    declareParameters();
    loadParameters();
    advertisePublishers();
    registerSubscribers();
    // registerServices();
    return;
}

void Slam::declareParameters() {

    node_->declare_parameter<std::string>("sub_wheel_odom_topic_", "wheel_odom");
    node_->declare_parameter<std::string>("sub_laser_topic_", "laser_topic");
    node_->declare_parameter<std::string>("pub_opt_path_topic_", "opt_path");
    node_->declare_parameter<std::string>("pub_opt_pose_topic_", "opt_pose");
    node_->declare_parameter<std::string>("opt_path_frame_", "map_link");
    node_->declare_parameter<std::string>("opt_pose_frame_", "map_link");
    node_->declare_parameter<std::string>("res_point_cloud_topic_", "res_point_cloud");

    RCLCPP_INFO(node_->get_logger(), "Slam: declare all parameters");

    return;
}

void Slam::loadParameters() {

    for (int i = 0; i < laser_odom_inf_matrix_.rows(); ++i) {
        laser_odom_inf_matrix_(i, i) = 0.2;
    }
    for (int i = 0; i < loop_inf_matrix_.rows(); ++i) {
        loop_inf_matrix_(i, i) = 0.1;
    }

    node_->get_parameter("sub_wheel_odom_topic_", sub_wheel_odom_topic_);
    node_->get_parameter("sub_laser_topic_", sub_laser_topic_);
    node_->get_parameter("pub_opt_path_topic_", pub_opt_path_topic_);
    node_->get_parameter("pub_opt_pose_topic_", pub_opt_pose_topic_);
    node_->get_parameter("opt_path_frame_", opt_path_frame_);
    node_->get_parameter("opt_pose_frame_", opt_pose_frame_);
    node_->get_parameter("res_point_cloud_topic_", res_point_cloud_topic_);

    RCLCPP_INFO(node_->get_logger(), "Slam: load all parameters");
    return;
}

void Slam::advertisePublishers() {
    opt_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(pub_opt_path_topic_, 10);
    opt_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pub_opt_pose_topic_, 10);
    RCLCPP_INFO(node_->get_logger(), "Slam: advertise all publishers");
    // res_point_cloud_pub_ = node_->create_publisher<pcl::PointCloud<pcl::PointXYZ> >(res_point_cloud_topic_, 10);
    return;
}

void Slam::registerSubscribers() {
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(node_, sub_laser_topic_);
    wheel_odom_sub_ = new message_filters::Subscriber<nav_msgs::msg::Odometry>(node_, sub_wheel_odom_topic_);
    sync_wheelOdom_laser_sub_ = new message_filters::Synchronizer<LaserWheelSyncPolicy>(LaserWheelSyncPolicy(100), *laser_sub_, *wheel_odom_sub_);
    sync_wheelOdom_laser_sub_->registerCallback(boost::bind(&Slam::laserWheelOdomSyncCallback, this, _1, _2));
    RCLCPP_INFO(node_->get_logger(), "Slam: register all subscribers");
    return;
}

/**
 * recivie the input sensor messgaes and process them
*/
void Slam::laserWheelOdomSyncCallback(const sensor_msgs::msg::LaserScan::ConstPtr& laser_msg, const nav_msgs::msg::Odometry::ConstPtr& wheel_odom_msg) {
    std::tuple<bool, unsigned int, pcl::PointCloud<pcl::PointXY>::Ptr> odom_res = odom_.odomDealWithInputMessage(laser_msg, wheel_odom_msg);
    bool is_new_key_frame = std::get<0>(odom_res);
    unsigned int key_frame_id = std::get<1>(odom_res);
    if (is_new_key_frame == true) {
        /* add a new key frame */
        RCLCPP_INFO(node_->get_logger(), "add a new key frame");

        pose_graph_.addSE2Node(odom_.key_frames_buffer_.back()->getPose());
        pose_graph_.addSE2Edge(key_frame_id-1, key_frame_id, odom_.key_frames_buffer_.back()->getRelativeMeasure(), laser_odom_inf_matrix_);

        std::pair<int, double> sc_res = scan_context_manger_.addNewFrame(key_frame_id, std::get<2>(odom_res));
        unsigned int loop_id = sc_res.first;
        double rot_angle = sc_res.second;
        if (loop_id != -1) { /* Find the loop closure by the scene context */
            RCLCPP_INFO(node_->get_logger(), "key_frame_id: %d, loop_id: %d", key_frame_id, loop_id);
            bool isPossibleLoop = scan_context_manger_.isPossibleLoop(odom_.key_frames_buffer_[key_frame_id]->getPose(), odom_.key_frames_buffer_[loop_id]->getPose());
            if (!isPossibleLoop) {
                return;
            }
            visualization_.publishLineOfTwoPoses(odom_.key_frames_buffer_[key_frame_id]->getPose(), odom_.key_frames_buffer_[loop_id]->getPose(), opt_path_frame_, 10000);
             /* The `rot_angle` means rotate the `key_frame_id` to the `loop_id`*/
            MatrixSE2 prior_guess = ang2Mat(rot_angle);
            MatrixSE2 loop_est = odom_.icpPointMatch(odom_.key_frames_buffer_[loop_id]->getCloud(), odom_.key_frames_buffer_[key_frame_id]->getCloud(), prior_guess);
            pose_graph_.addSE2Edge(loop_id, key_frame_id, loop_est, loop_inf_matrix_);
        }
    }
    return;
}

/*
bool Slam::optimize_signal_callback(slam_demo::OptSrv::Request& req, slam_demo::OptSrv::Response& res) {
    RCLCPP_INFO("Slam::optimize_signal_callback()");
    // save the optimized pose
    pose_graph_.saveGraph("/home/ros/catkin_ws/src/darko/slam_demo/launch/before_path.g2o");
    pose_graph_.optimize(15);

    // publish the optimized pose
    RCLCPP_INFO("The number of vertices: %d", pose_graph_.optimizer_->vertices().size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_3d(new pcl::PointCloud<pcl::PointXYZ>()); // Create map
        pcl::PointCloud<pcl::PointXYZ>::Ptr key_frame_3d(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*odom_.key_frames_buffer_[i]->getCloud(), *key_frame_3d);
        // pose_msg to matrix
        Eigen::Matrix4f pose_mat;
        pose_mat << cos(orientation), -sin(orientation), 0, position.x(),
                    sin(orientation), cos(orientation), 0, position.y(),
                    0, 0, 1, 0,
                    0, 0, 0, 1;
        pcl::transformPointCloud(*key_frame_3d, *tmp_3d, pose_mat);

        *merged += *tmp_3d;
    }
    path.header.stamp = ros::Time::now();
    path.header.frame_id = opt_path_frame_;
    opt_path_pub_->publish(path);
    merged->header.frame_id = opt_path_frame_;
    pcl_conversions::toPCL(ros::Time::now(), merged->header.stamp);
    res_point_cloud_pub_->publish(*merged);
    return true;
}
*/

/*
void Slam::registerServices() {
    opt_server_ = nh_.advertiseService("optimize_service", &Slam::optimize_signal_callback, this);
    return;
}
*/