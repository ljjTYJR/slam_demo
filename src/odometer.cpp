#include "odomter.h"
#include "pointmatcher/PointMatcher.h"

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include <eigen_conversions/eigen_msg.h>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;


Odometer::Odometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
    nhp_(pnh),
    sensor_offset_(Eigen::Affine3d::Identity()),
    latest_odom_(MatrixSE2::Identity()),
    set_the_first_pose_(false),
    latest_scan_(new pcl::PointCloud<pcl::PointXY>()),
    prev_scan_(new pcl::PointCloud<pcl::PointXY>()) {
    ROS_INFO("The odometer is preparing to initialize.");
    init();
}

void Odometer::init() {
  loadParameters();
  advertisePublishers();
  return;
}

void Odometer::loadParameters() {
  // frames
  nhp_.param("laser_frame_", laser_frame_, std::string("laser_frame"));
  nhp_.param("odom_frame_", odom_frame_, std::string("odom_frame"));
  nhp_.param("base_frame_", base_frame_, std::string("base_frame"));
  // topics
  nhp_.param("pub_pose_topic_", pub_pose_topic_, std::string("pose"));
  nhp_.param("pub_laser_topic_", pub_laser_topic_, std::string("laser"));
  nhp_.param("pub_path_topic_", pub_path_topic_, std::string("path"));
  // flags
  nhp_.param("use_wheel_odom_", use_wheel_odom_, true);
  nhp_.param("use_laser_", use_laser_, true);
  nhp_.param("use_darko_cfg_", use_darko_cfg_, true);
  nhp_.param("use_wheel_odom_prior_guess_", use_wheel_odom_prior_guess_, true);


  tf::TransformListener listener;
  Eigen::Affine3d transform_static;
  tf::StampedTransform transform_stamp;
  bool listening = true;
  float wait_seconds = 1.0;
  while (listening) {
    try {
        ROS_INFO("trying to get the transform from %s to %s", base_frame_.c_str(), laser_frame_.c_str());
        listener.lookupTransform(base_frame_, laser_frame_, ros::Time(0), transform_stamp);
    } catch (tf::TransformException ex) {
        ROS_INFO("waiting to get the transform from %s to %s", base_frame_.c_str(), laser_frame_.c_str());
        usleep(1000000 * wait_seconds); // wait for 1 second
        continue;
    }
    listening = false;
  }
  tf::transformTFToEigen(transform_stamp, sensor_offset_);
  std::cout << "sensor_offset_ = " << sensor_offset_.matrix() << std::endl;
  return;
}

void Odometer::advertisePublishers() {
  odom_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pub_pose_topic_, 100);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(pub_laser_topic_, 100);
  path_pub_ = nh_.advertise<nav_msgs::Path>(pub_path_topic_, 100);
  ROS_INFO("Advertised to %s, %s, %s", pub_pose_topic_.c_str(), pub_laser_topic_.c_str(), pub_path_topic_.c_str());
  return;
}

std::tuple<bool, unsigned int, pcl::PointCloud<pcl::PointXY>::Ptr>
Odometer::odomDealWithInputMessage(const sensor_msgs::LaserScan::ConstPtr& laser_msg, const nav_msgs::Odometry::ConstPtr& wheel_odom_msg) {
  bool update = false;
  unsigned int key_frame_cnt = 0;

  readInLaserScan(laser_msg);
  readInWheelOdom(wheel_odom_msg);

  update = updateOdom();
  if (update) {
    publishPose();
    publishLaser(laser_msg);
    key_frame_cnt = key_frames_buffer_.size();
  }
  // Here, minus 1 because the index should start from 0
  return std::make_tuple(update, key_frame_cnt - 1, latest_scan_);
}

void Odometer::point3d2Point2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXY>::Ptr& output_cloud) {
  output_cloud->points.resize(input_cloud->points.size());
    for (unsigned int i = 0; i < input_cloud->points.size(); ++i) {
        output_cloud->points[i].x = input_cloud->points[i].x;
        output_cloud->points[i].y = input_cloud->points[i].y;
    }
  return;
}

void Odometer::readInLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {
  if (latest_scan_->points.size() != 0) {
    *prev_scan_ = *latest_scan_;
  }

  timer_ = laser_msg->header.stamp;
  static laser_geometry::LaserProjection laser_projector;
  sensor_msgs::PointCloud2 pc_2;
  laser_projector.projectLaser(*laser_msg, pc_2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(pc_2, *input_cloud);
  if (use_darko_cfg_) {
      /* transform the read in point cloud to the coordinate of the robot, represented in the robot */
      pcl::transformPointCloud(*input_cloud, *input_cloud, sensor_offset_);
      point3d2Point2d(input_cloud, latest_scan_);
  } else {
      // To do: test for 2d
      ROS_ERROR("Not implemented yet");
  }
  latest_scan_->width = latest_scan_->points.size();
  latest_scan_->height = 1;
  latest_scan_->is_dense = true;

  return;
}

void Odometer::readInWheelOdom(const nav_msgs::Odometry::ConstPtr& wheel_odom_msg) {
  Eigen::Affine3d T;
  tf::poseMsgToEigen(wheel_odom_msg->pose.pose, T);
  Eigen::Matrix3d r3 = T.rotation();
  Eigen::Vector3d t = T.translation();
  MatrixSE2 new_pose = MatrixSE2::Identity();

  for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
          new_pose(i,j) = r3(i,j);
      }
  }
  for (int i = 0; i < 2; i++) {
      new_pose(i,2) = t(i);
  }
  if (wheel_odom_mem_.size() == 0) {
    ROS_INFO("Read in the first wheel odom");
    prev_wheel_odom_ = new_pose;
  } else {
    prev_wheel_odom_ = wheel_odom_mem_.back();
  }
  wheel_odom_mem_.push_back(new_pose);
  return;
}

void Odometer::addNewKeyFrame(const MatrixSE2& pose, const MatrixSE2& relative_measure, const pcl::PointCloud<pcl::PointXY>::Ptr& cloud) {
  unsigned int key_frame_id = key_frames_buffer_.size();
  KeyFrame::Ptr new_key_frame(new KeyFrame(key_frame_id, pose, relative_measure, cloud));
  key_frames_buffer_.push_back(new_key_frame);
  return;
}

bool Odometer::transLargeEnough(const MatrixSE2& pose) {
  double distance = sqrt(pow(pose(0,2), 2) + pow(pose(1,2), 2));
  double angle = atan2(pose(1,0), pose(0,0));
  if (distance > kMinDist_ || abs(angle) > kMinRot_) {  /* atan2 is in [-pi,pi]*/
    return true;
  } else {
    return false;
  }
}

bool Odometer::updateOdom() {
  if (!key_frames_buffer_.size()) {
    ROS_INFO("Add the first key frame");
    // new a new point cloud
    pcl::PointCloud<pcl::PointXY>::Ptr key_cloud(new pcl::PointCloud<pcl::PointXY>());
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
    pcl::PointCloud<pcl::PointXY>::Ptr prev_key_scan = key_frames_buffer_.back()->getCloud();
    // The prior guess still uses the wheel odom guess
    MatrixSE2 prior_guess = MatrixSE2::Identity();
    if (use_wheel_odom_prior_guess_) {
      prior_guess = prev_wheel_key_odom_.inverse() * wheel_odom_mem_.back();
    }
    // TODO: In keyframes configuration, can we use the constant velocity model to predict the pose?
    MatrixSE2 trans_scan_match = icpPointMatch(prev_key_scan, latest_scan_, prior_guess);
    // check whether the new scan is far enough from the latest key frame
    if (transLargeEnough(trans_scan_match)) {
      // new a new point cloud
      pcl::PointCloud<pcl::PointXY>::Ptr key_cloud(new pcl::PointCloud<pcl::PointXY>());
      *key_cloud = *latest_scan_;
      MatrixSE2 key_pose = key_frames_buffer_.back()->getPose() * trans_scan_match;
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
MatrixSE2 Odometer::icpPointMatch(const pcl::PointCloud<pcl::PointXY>::Ptr& prev_scan,
                                  const pcl::PointCloud<pcl::PointXY>::Ptr& cur_scan, const MatrixSE2& guess) {

  if (prev_scan->points.size() == 0 || cur_scan->points.size() == 0) {
    ROS_ERROR("No point cloud received");
    exit(1);
  }

  // todo: use the static variable to save the time of initialization
  PM::ICP icp;
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;
  name = "KDTreeMatcher";
  params["knn"] = "2";
  std::shared_ptr<PM::Matcher> kdtree = PM::get().MatcherRegistrar.create(name, params);
  params.clear();
  name = "TrimmedDistOutlierFilter";
  params["ratio"] = "0.75";
  std::shared_ptr<PM::OutlierFilter> trim = PM::get().OutlierFilterRegistrar.create(name, params);
  params.clear();
  name = "PointToPointErrorMinimizer";
  std::shared_ptr<PM::ErrorMinimizer> pointToPoint = PM::get().ErrorMinimizerRegistrar.create(name);
  name = "CounterTransformationChecker";
  params["maxIterationCount"] = "20";
  std::shared_ptr<PM::TransformationChecker> maxIter = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();
  std::shared_ptr<PM::Transformation> rigidTrans = PM::get().TransformationRegistrar.create("RigidTransformation");
  std::shared_ptr<PM::Inspector> nullInspect = PM::get().InspectorRegistrar.create("NullInspector");
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
  for (unsigned int i = 0; i < prev_scan->points.size(); i++)
  {
      ref.features(0, i) = prev_scan->points[i].x;
      ref.features(1, i) = prev_scan->points[i].y;
      ref.features(2, i) = 1;
  }
  data.features.resize(3, cur_scan->points.size());
  for (unsigned int i = 0; i < cur_scan->points.size(); i++)
  {
      data.features(0, i) = cur_scan->points[i].x;
      data.features(1, i) = cur_scan->points[i].y;
      data.features(2, i) = 1;
  }
  /* set the initial guess */
  PM::TransformationParameters T = PM::TransformationParameters::Identity(3, 3);
  T(0, 2) = guess(0,2);
  T(1, 2) = guess(1,2);
  T(0, 0) = guess(0,0);
  T(0, 1) = guess(0,1);
  T(1, 0) = guess(1,0);
  T(1, 1) = guess(1,1);
  /* The registration result */
  PM::TransformationParameters T_final = icp(data, ref, T);
  MatrixSE2 T_final_eigen;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      T_final_eigen(i,j) = T_final(i,j);
    }
  }
  return T_final_eigen;
}

void Odometer::publishPose() {
  static nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = timer_;
  pose.header.frame_id = odom_frame_;
  MatrixSE2 cur_pose = odom_mem_.back();
  double theta = atan2(cur_pose(1, 0), cur_pose(0, 0));
  pose.pose.orientation.w = cos(theta / 2);
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = sin(theta / 2);
  pose.pose.position.x = cur_pose(0, 2);
  pose.pose.position.y = cur_pose(1, 2);
  pose.pose.position.z = 0;
  odom_pub_.publish(pose);

  path.header.frame_id = odom_frame_;
  path.poses.push_back(pose);
  path_pub_.publish(path);

  //send transfrom
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tr;
  tr.header.stamp = timer_;
  tr.header.frame_id = odom_frame_;
  tr.child_frame_id = base_frame_;
  tr.transform.translation.x = cur_pose(0, 2);
  tr.transform.translation.y = cur_pose(1, 2);
  tr.transform.translation.z = 0;
  tr.transform.rotation.x = pose.pose.orientation.x;
  tr.transform.rotation.y = pose.pose.orientation.y;
  tr.transform.rotation.z = pose.pose.orientation.z;
  tr.transform.rotation.w = pose.pose.orientation.w;
  br.sendTransform(tr);
}

// Todo: use the readin laser scan
void Odometer::publishLaser(const sensor_msgs::LaserScanConstPtr &msg) {
  static sensor_msgs::LaserScan laserscan;
  laserscan.header.stamp = timer_;
  laserscan.header.frame_id = laser_frame_;
  laserscan.angle_min = msg->angle_min;
  laserscan.angle_max = msg->angle_max;
  laserscan.angle_increment = msg->angle_increment;
  laserscan.time_increment = msg->time_increment;
  laserscan.scan_time = msg->scan_time;
  laserscan.range_min = msg->range_min;
  laserscan.range_max = msg->range_max;
  laserscan.ranges.resize(msg->ranges.size());
  laserscan.intensities.resize(msg->ranges.size());
  for (unsigned int i = 0; i < msg->ranges.size(); i++)
  {
      laserscan.ranges[i] = msg->ranges[i];
      laserscan.intensities[i] = msg->intensities[i];
  }
  laser_pub_.publish(laserscan);
}


Odometer::~Odometer()
{
}