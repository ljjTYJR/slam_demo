#include "slam_demo/odometer.h"
#include "pointmatcher/PointMatcher.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rclcpp/rclcpp.hpp"

#include <iostream>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

Odometer::Odometer(const rclcpp::Node::SharedPtr node)
    : node_(node),
      sensor_offset_(Eigen::Affine3d::Identity()),
      latest_odom_(MatrixSE2::Identity()),
      set_the_first_pose_(false),
      latest_scan_(new pcl::PointCloud<pcl::PointXY>()),
      prev_scan_(new pcl::PointCloud<pcl::PointXY>()) {
    RCLCPP_INFO(node_->get_logger(),
                "The odometer is preparing to initialize.");
    init();
}

void Odometer::init() {
    declareParameters();
    loadParameters();
    advertisePublishers();
    return;
}

void Odometer::declareParameters() {
    // frames
    node_->declare_parameter<std::string>("sensor_link", "sensor_link");
    node_->declare_parameter<std::string>("robot_link", "robot_link");
    node_->declare_parameter<std::string>("map_link", "map_link");
    // topics
    node_->declare_parameter<std::string>("pub_pose_topic_", "pose");
    node_->declare_parameter<std::string>("pub_laser_topic_", "laser");
    node_->declare_parameter<std::string>("pub_path_topic_", "path");
    // flags
    node_->declare_parameter<bool>("use_wheel_odom_", true);
    node_->declare_parameter<bool>("use_laser_", true);
    node_->declare_parameter<bool>("use_darko_cfg_", true);
    node_->declare_parameter<bool>("use_wheel_odom_prior_guess_", true);
    // hyperparameters
    node_->declare_parameter<double>("farthest_point_dist", 10.0);
    return;
}

Eigen::Isometry3d Odometer::getSensorOffset() {
    tf2_ros::Buffer tf_buffer(node_->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    Eigen::Isometry3d transform_static;
    geometry_msgs::msg::TransformStamped t;
    bool listening = true;
    float wait_seconds = 1.0;
    while (listening) {
        try {
            RCLCPP_INFO(node_->get_logger(),
                        "trying to get the transform from %s to %s",
                        robot_link.c_str(),
                        sensor_link.c_str());
            t = tf_buffer.lookupTransform(
                robot_link, sensor_link, rclcpp::Time(0));
            RCLCPP_INFO(node_->get_logger(),
                        "t: %f, %f, %f, %f, %f, %f, %f",
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w);
        } catch (tf2::TransformException& ex) {
            RCLCPP_INFO(node_->get_logger(),
                        "waiting to get the transform from %s to %s",
                        robot_link.c_str(),
                        sensor_link.c_str());
            usleep(1000000 * wait_seconds);  // wait for 1 second
            continue;
        }
        listening = false;
    }
    transform_static = tf2::transformToEigen(t);
    std::cout << transform_static.matrix() << std::endl;
    return transform_static;
}

void Odometer::loadParameters() {
    // frames
    node_->get_parameter("sensor_link", sensor_link);
    node_->get_parameter("robot_link", robot_link);
    node_->get_parameter("map_link", map_link);
    // topics
    node_->get_parameter("pub_pose_topic_", pub_pose_topic_);
    node_->get_parameter("pub_laser_topic_", pub_laser_topic_);
    node_->get_parameter("pub_path_topic_", pub_path_topic_);
    // flags
    node_->get_parameter("use_wheel_odom_", use_wheel_odom_);
    node_->get_parameter("use_laser_", use_laser_);
    node_->get_parameter("use_darko_cfg_", use_darko_cfg_);
    node_->get_parameter("use_wheel_odom_prior_guess_",
                         use_wheel_odom_prior_guess_);
    // hyperparameters
    node_->get_parameter("farthest_point_dist", farthest_point_dist);

    sensor_offset_ = getSensorOffset().cast<double>().matrix();
    return;
}

void Odometer::advertisePublishers() {
    odom_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        pub_pose_topic_, 100);
    path_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>(pub_path_topic_, 10);
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        pub_laser_topic_, 10);
    tf_broadcaster_map_to_robot_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    RCLCPP_INFO(node_->get_logger(),
                "Advertised to %s, %s, %s",
                pub_pose_topic_.c_str(),
                pub_laser_topic_.c_str(),
                pub_path_topic_.c_str());
    return;
}

std::tuple<bool, unsigned int, pcl::PointCloud<pcl::PointXY>::Ptr>
Odometer::odomDealWithInputMessage(
    const sensor_msgs::msg::LaserScan::ConstPtr& laser_msg,
    const nav_msgs::msg::Odometry::ConstPtr& wheel_odom_msg) {
    bool update = false;
    unsigned int key_frame_cnt = 0;

    readInLaserScan(laser_msg);
    readInWheelOdom(wheel_odom_msg);

    update = updateOdom();
    if (update) {
        publishPosePathAndPointCloud();
        key_frame_cnt = key_frames_buffer_.size();
    }
    // Here, minus 1 because the index should start from 0
    return std::make_tuple(update, key_frame_cnt - 1, latest_scan_);
}

void Odometer::readInLaserScan(
    const sensor_msgs::msg::LaserScan::ConstPtr& laser_msg) {
    if (latest_scan_->points.size() != 0) {
        *prev_scan_ = *latest_scan_;
    }

    timer_ = laser_msg->header.stamp;
    static laser_geometry::LaserProjection laser_projector;
    sensor_msgs::msg::PointCloud2 pc_2;
    laser_projector.projectLaser(*laser_msg, pc_2);
    // clear the latest_scan_
    latest_scan_->points.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pc_2, *input_cloud);
    if (use_darko_cfg_) {
        /* transform the read in point cloud to the coordinate of the robot,
         * represented in the robot */
        pcl::transformPointCloud(*input_cloud, *input_cloud, sensor_offset_);
        point3d2point2d(input_cloud, latest_scan_, farthest_point_dist);
    }
    latest_scan_->width = latest_scan_->points.size();
    latest_scan_->height = 1;
    latest_scan_->is_dense = true;

    return;
}

void Odometer::readInWheelOdom(
    const nav_msgs::msg::Odometry::ConstPtr wheel_odom_msg) {
    Eigen::Isometry3d T;
    /* TODO: New function, to test? */
    tf2::fromMsg(wheel_odom_msg->pose.pose, T);
    Eigen::Matrix3d r3 = T.rotation();
    Eigen::Vector3d t = T.translation();
    MatrixSE2 new_pose = MatrixSE2::Identity();

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            new_pose(i, j) = r3(i, j);
        }
    }
    for (int i = 0; i < 2; i++) {
        new_pose(i, 2) = t(i);
    }
    if (wheel_odom_mem_.size() == 0) {
        RCLCPP_INFO(node_->get_logger(), "Read in the first wheel odom");
        prev_wheel_odom_ = new_pose;
    } else {
        prev_wheel_odom_ = wheel_odom_mem_.back();
    }
    wheel_odom_mem_.push_back(new_pose);
    return;
}

void Odometer::addNewKeyFrame(const MatrixSE2& pose,
                              const MatrixSE2& relative_measure,
                              const pcl::PointCloud<pcl::PointXY>::Ptr& cloud) {
    unsigned int key_frame_id = key_frames_buffer_.size();
    KeyFrame::Ptr new_key_frame(
        new KeyFrame(key_frame_id, pose, relative_measure, cloud));
    key_frames_buffer_.push_back(new_key_frame);
    return;
}

bool Odometer::transLargeEnough(const MatrixSE2& pose) {
    double distance = sqrt(pow(pose(0, 2), 2) + pow(pose(1, 2), 2));
    double angle = atan2(pose(1, 0), pose(0, 0));
    if (distance > kMinDist_ ||
        abs(angle) > kMinRot_) { /* atan2 is in [-pi,pi]*/
        return true;
    } else {
        return false;
    }
}

bool Odometer::updateOdom() {
    if (!key_frames_buffer_.size()) {
        RCLCPP_INFO(node_->get_logger(), "Add the first key frame");
        // new a new point cloud
        pcl::PointCloud<pcl::PointXY>::Ptr key_cloud(
            new pcl::PointCloud<pcl::PointXY>());
        *key_cloud = *latest_scan_;
        // use the latest odom to initialize a new pose
        latest_odom_ = wheel_odom_mem_.back();
        prev_wheel_key_odom_ = wheel_odom_mem_.back();
        // set it as the previous key frame odom
        odom_mem_.push_back(latest_odom_);
        MatrixSE2 key_pose = latest_odom_;
        addNewKeyFrame(key_pose, MatrixSE2::Identity(), key_cloud);
        return true;
    } else {
        // check whether the new scan is far enough from the latest key frame
        pcl::PointCloud<pcl::PointXY>::Ptr prev_key_scan =
            key_frames_buffer_.back()->getCloud();
        // The prior guess still uses the wheel odom guess
        MatrixSE2 prior_guess = MatrixSE2::Identity();
        if (use_wheel_odom_prior_guess_) {
            prior_guess =
                prev_wheel_key_odom_.inverse() * wheel_odom_mem_.back();
        }
        // TODO: In keyframes configuration, can we use the constant velocity
        // model to predict the pose?
        MatrixSE2 trans_scan_match =
            icpPointMatch(prev_key_scan, latest_scan_, prior_guess);
        // check whether the new scan is far enough from the latest key frame
        if (transLargeEnough(trans_scan_match)) {
            // new a new point cloud
            pcl::PointCloud<pcl::PointXY>::Ptr key_cloud(
                new pcl::PointCloud<pcl::PointXY>());
            *key_cloud = *latest_scan_;
            MatrixSE2 key_pose =
                key_frames_buffer_.back()->getPose() * trans_scan_match;
            addNewKeyFrame(key_pose, trans_scan_match, key_cloud);
            prev_wheel_key_odom_ = wheel_odom_mem_.back();
            odom_mem_.push_back(key_pose);
            return true;
        }
    }
    return false;
}

// Use the libpointmatcher to do the scan matching
// prev_scan = T * cur_scan
// Todo: adjust the types or re-write the registration
MatrixSE2 Odometer::icpPointMatch(
    const pcl::PointCloud<pcl::PointXY>::Ptr& prev_scan,
    const pcl::PointCloud<pcl::PointXY>::Ptr& cur_scan,
    const MatrixSE2& guess) {
    if (prev_scan->points.size() == 0 || cur_scan->points.size() == 0) {
        RCLCPP_ERROR(node_->get_logger(), "Empty scan");
        exit(1);
    }

    // todo: use the static variable to save the time of initialization
    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;
    name = "KDTreeMatcher";
    params["knn"] = "2";
    std::shared_ptr<PM::Matcher> kdtree =
        PM::get().MatcherRegistrar.create(name, params);
    params.clear();
    name = "TrimmedDistOutlierFilter";
    params["ratio"] = "0.75";
    std::shared_ptr<PM::OutlierFilter> trim =
        PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();
    name = "PointToPointErrorMinimizer";
    std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
        PM::get().ErrorMinimizerRegistrar.create(name);
    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "20";
    std::shared_ptr<PM::TransformationChecker> maxIter =
        PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();
    std::shared_ptr<PM::Transformation> rigidTrans =
        PM::get().TransformationRegistrar.create("RigidTransformation");
    std::shared_ptr<PM::Inspector> nullInspect =
        PM::get().InspectorRegistrar.create("NullInspector");
    icp.matcher = kdtree;
    icp.outlierFilters.push_back(trim);
    icp.errorMinimizer = pointToPoint;
    icp.transformationCheckers.push_back(maxIter);
    icp.transformations.push_back(rigidTrans);
    icp.inspector = nullInspect;

    /* convert the point cloud to libpointmatcher format */
    DP ref;
    DP data;
    ref.features.resize(3, prev_scan->points.size());
    for (unsigned int i = 0; i < prev_scan->points.size(); i++) {
        ref.features(0, i) = prev_scan->points[i].x;
        ref.features(1, i) = prev_scan->points[i].y;
        ref.features(2, i) = 1;
    }
    data.features.resize(3, cur_scan->points.size());
    for (unsigned int i = 0; i < cur_scan->points.size(); i++) {
        data.features(0, i) = cur_scan->points[i].x;
        data.features(1, i) = cur_scan->points[i].y;
        data.features(2, i) = 1;
    }
    /* set the initial guess */
    PM::TransformationParameters T =
        PM::TransformationParameters::Identity(3, 3);
    T(0, 2) = guess(0, 2);
    T(1, 2) = guess(1, 2);
    T(0, 0) = guess(0, 0);
    T(0, 1) = guess(0, 1);
    T(1, 0) = guess(1, 0);
    T(1, 1) = guess(1, 1);
    /* The registration result */
    PM::TransformationParameters T_final = icp(data, ref, T);
    MatrixSE2 T_final_eigen;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T_final_eigen(i, j) = T_final(i, j);
        }
    }
    return T_final_eigen;
}

void Odometer::publishPosePathAndPointCloud() {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = timer_;
    pose.header.frame_id = map_link;
    MatrixSE2 cur_pose = odom_mem_.back();
    double theta = atan2(cur_pose(1, 0), cur_pose(0, 0));
    pose.pose.orientation.w = cos(theta / 2);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(theta / 2);
    pose.pose.position.x = cur_pose(0, 2);
    pose.pose.position.y = cur_pose(1, 2);
    pose.pose.position.z = 0;
    odom_pub_->publish(pose);

    static nav_msgs::msg::Path path;
    path.header.frame_id = map_link;
    path.poses.push_back(pose);
    path_pub_->publish(path);

    // publish the point cloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*latest_scan_, *cloud_3d);
    Eigen::Affine3d pose_mat = pose_stamped_to_eigen(pose);
    pcl::transformPointCloud(*cloud_3d, *cloud_3d, pose_mat);
    pcl::toROSMsg(*cloud_3d, cloud_msg);
    cloud_msg.header.frame_id = map_link;
    cloud_msg.header.stamp = timer_;
    cloud_pub_->publish(cloud_msg);

    // send transfrom
    geometry_msgs::msg::TransformStamped tr;
    tr.header.stamp = timer_;
    tr.header.frame_id = map_link;
    tr.child_frame_id = robot_link;
    tr.transform.translation.x = cur_pose(0, 2);
    tr.transform.translation.y = cur_pose(1, 2);
    tr.transform.translation.z = 0;
    tr.transform.rotation.x = pose.pose.orientation.x;
    tr.transform.rotation.y = pose.pose.orientation.y;
    tr.transform.rotation.z = pose.pose.orientation.z;
    tr.transform.rotation.w = pose.pose.orientation.w;
    tf_broadcaster_map_to_robot_->sendTransform(tr);
}

Odometer::~Odometer() {}
