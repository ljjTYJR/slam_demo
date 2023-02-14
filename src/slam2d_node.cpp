#include "slam2d.h"
#include "mcl.h"
#include "pgo_wrapper.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <signal.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <Eigen/Eigen>

#include <sensor_msgs/PointCloud2.h>

slam2d slam;
mcl P;
pgo graph;
ros::Publisher pub_pose, pub_path;
ros::Publisher pub_laserscan;
ros::Publisher pub_map2d;
ros::Publisher pub_point_cloud;
bool g_shutdown_requested = false;
void publish_pose(slam2d &slam);
void publish_map2d(slam2d &slam);

void publishLaserScan(const sensor_msgs::LaserScanConstPtr &msg)
{
    //publish laserscan
    sensor_msgs::LaserScan laserscan;
    laserscan.header.stamp = msg->header.stamp;
    laserscan.header.frame_id = slam.frame_sensor_link;
    laserscan.angle_min = msg->angle_min;
    laserscan.angle_max = msg->angle_max;
    laserscan.angle_increment = msg->angle_increment;
    laserscan.time_increment = msg->time_increment;
    laserscan.scan_time = msg->scan_time;
    laserscan.range_min = msg->range_min;
    laserscan.range_max = msg->range_max;
    laserscan.ranges.resize(msg->ranges.size());
    laserscan.intensities.resize(msg->ranges.size());
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        laserscan.ranges[i] = msg->ranges[i];
        laserscan.intensities[i] = msg->intensities[i];
    }
    pub_laserscan.publish(laserscan);
}

void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    slam.readin_scan_data(msg);
    slam.update();
    publish_pose(slam);
    publish_map2d(slam);
    publishLaserScan(msg);
    return;
}

void multiecho_laserscan_callback(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    slam.readin_scan_data(msg);
    slam.update();
    publish_pose(slam);
    publish_map2d(slam);
    // multiecho2laserscan(msg);
}

void publish_map2d(slam2d &slam)
{
    slam.map2d.header.stamp = ros::Time(slam.timestamp);
    pub_map2d.publish(slam.map2d);
}

void publish_pose(slam2d &slam)
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(slam.timestamp);
    pose.header.frame_id = "odom";
    MatrixSE2 cur_pose = slam.getLatestOdom();
    double theta = atan2(cur_pose(1, 0), cur_pose(0, 0));
    // pose.pose.orientation.w = sqrt(1 + cur_pose(1,1) + cur_pose(2,2)) / 2;
    // pose.pose.orientation.x = 0;
    // pose.pose.orientation.y = 0;
    // pose.pose.orientation.z = (cur_pose(1,0) - cur_pose(0,1)) / (4 * pose.pose.orientation.w);
    pose.pose.orientation.w = cos(theta / 2);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(theta / 2);
    pose.pose.position.x = cur_pose(0, 2);
    pose.pose.position.y = cur_pose(1, 2);
    pose.pose.position.z = 0;
    pub_pose.publish(pose);

    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    pub_path.publish(path);

    //send transfrom
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr;
    tr.header.stamp = ros::Time(slam.timestamp);
    tr.header.frame_id = "odom";
    tr.child_frame_id = "base_link";
    tr.transform.translation.x = cur_pose(0, 2);
    tr.transform.translation.y = cur_pose(1, 2);
    tr.transform.translation.z = 0;
    tr.transform.rotation.x = pose.pose.orientation.x;
    tr.transform.rotation.y = pose.pose.orientation.y;
    tr.transform.rotation.z = pose.pose.orientation.z;
    tr.transform.rotation.w = pose.pose.orientation.w;
    br.sendTransform(tr);
}

/* Get the transformation matrix by listening to the tf transformer */
void setRobotToSensorPose(ros::NodeHandle nh) {
    tf::TransformListener listener;
    Eigen::Affine3d transform_static;
    tf::StampedTransform transform_stamp;
    bool listening = true;
    float wait_seconds = 1.0;
    while (listening) {
        try {
            listener.lookupTransform(slam.frame_base_link, slam.frame_sensor_link, ros::Time(0), transform_stamp);
        } catch (tf::TransformException ex) {
            /* wait for one seconds */
            ros::Duration(wait_seconds).sleep();
            continue;
        }
        listening = false;
    }
    tf::transformTFToEigen(transform_stamp, transform_static);
    ROS_INFO_STREAM("The tf transformation is " << endl << transform_static.matrix() << endl);
    slam.setRobotToSensorTrans(transform_static);
    return;
}

/**
 * Init the ros node parameters
 * @nh: ros node
 */
void paramInit(ros::NodeHandle nh) {
    /* Frames id setting */
    nh.param<string>("frame_base_link", slam.frame_base_link, "base_link");
    nh.param<string>("frame_sensor_link", slam.frame_sensor_link, "sensor_link");
    /* Topics setting */
    nh.param<string>("topic_laser_scan", slam.topic_laser_scan, "/robot/front_laser/scan");
    nh.param<string>("topic_robot_odom", slam.topic_robot_odom, "/robot/throttle/odom");
    /* Control parameters setting */
    nh.param<bool>("bool_use_robot_odom", slam.bool_use_robot_odom, true);
    nh.param<bool>("bool_use_mcl", slam.bool_use_mcl, false);
    nh.param<bool>("bool_use_pose_graph", slam.bool_use_pose_graph, true);
    nh.param<bool>("bool_darko_cfg", slam.bool_darko_cfg, true);
    nh.param<bool>("bool_use_wheel_odom_as_prior", slam.bool_use_wheel_odom_as_prior, true);
    /* If use odom, need to sychorinze the pose of sensor */
    if (slam.bool_darko_cfg) {
        setRobotToSensorPose(nh);
    }
    return;
}

/**
 * Deal with the sychronized msgs: laser and odom
 */
void laserOdomSyncCallback(const sensor_msgs::LaserScanConstPtr& msg_laser, const nav_msgs::OdometryConstPtr& msg_odom) {
    slam.readin_scan_data(msg_laser);
    slam.updateStateByOdom(msg_odom);
    // slam.scan_map_match_random();
    slam.update_map();
    publish_pose(slam);
    publish_map2d(slam);
    publishLaserScan(msg_laser);
    return;
}

void laserOdomPoseGraphSyncCallback(const sensor_msgs::LaserScanConstPtr& msg_laser, const nav_msgs::OdometryConstPtr& msg_odom) {
    // TODO The better way to sample the odom information
    static int cnt = 0;
    static int pose_id = 0;
    if (cnt % 2 != 0) {
    } else {
        // Read the odom information and save the vertex information and edge information
        slam.readin_scan_data(msg_laser);
        slam.updateStateByOdom(msg_odom);

        // save the pose information to the pose graph
        pgo::pose_node node;
        node.id = pose_id;
        node.pose = SE2(slam.state.t(0), slam.state.t(1), slam.state.theta);
        graph.pgoPushBackNode(node);

        publish_pose(slam);
        publishLaserScan(msg_laser);

        pose_id++;
    }
    cnt++;
    return;
}

void laserOdomWheelOdomSyncCallback(const sensor_msgs::LaserScanConstPtr& msg_laser, const nav_msgs::OdometryConstPtr& msg_odom) {
    slam.readin_scan_data(msg_laser);
    ROS_INFO("read in the laser scan data");
    slam.updateWheelOdom(msg_odom);
    ROS_INFO("update the wheel odom");
    slam.update();
    ROS_INFO("update the odom");
    publish_pose(slam);
    ROS_INFO("publish the pose");
    publishLaserScan(msg_laser);
    ROS_INFO("publish the scan");
    return;
}

void laserOdomMCLSyncCallback(const sensor_msgs::LaserScanConstPtr& msg_laser, const nav_msgs::OdometryConstPtr& msg_odom) {
    // Initialize the mcl when the first frame comes in
    static int cnt = 0;
    if (!P.isInitialized()) {
        slam.readin_scan_data(msg_laser);
        slam.updateStateByOdom(msg_odom);
        double var[3] = {0.5, 0.5, 0.1};
        // Read in the first frame's states
        double cur_x = slam.state.t(0);
        double cur_y = slam.state.t(1);
        double cur_theta = slam.state.theta;
        int N = 60;
        P.init(N, cur_x, cur_y, cur_theta, var);
        if (P.isInitialized()) {
            ROS_INFO("The particles are initialized now.");
        }
        slam.update_map();
        goto UPDATE;
    } else {
        cnt++;
        if (cnt%10 != 0) {
            return;
        } else {
            cnt = 0;
        }
        slam.readin_scan_data(msg_laser);
        // Use particle filter to localize, the difference is how to choose the next 'best' delta pose.
        slam.updateStateByOdom(msg_odom);
        double delta[3] = {0,0,0};
        slam.getDeltaMove(delta);
        double var[] = {0.1, 0.1, 0.05};
        // Predict the particles' next noisy motion
        P.predict(var, delta);
        // Update the map, measure and choose the best particle
        int best_idx = 0;
        double best_score = 0;
        for (int i = 0; i < P.getNumParticles(); ++i) {
            Vector3d pose;
            pose(0) = P.particles[i].theta;
            pose(1) = P.particles[i].x;
            pose(2) = P.particles[i].y;
            P.particles[i].score = slam.scan_map_match_score(pose) * P.particles[i].weight;
            if (P.particles[i].score > best_score) {
                best_score = P.particles[i].score;
                best_idx = i;
            }
        }
        Vector3d pose;
        pose(0) = P.particles[best_idx].theta;
        pose(1) = P.particles[best_idx].x;
        pose(2) = P.particles[best_idx].y;
        slam.updateState(pose);
        P.updateWeights();
        slam.update_map();
    }
UPDATE:
    ROS_INFO("Update the ros info");
    publish_pose(slam);
    publish_map2d(slam);
    publishLaserScan(msg_laser);
    return;
}

void deal_graph() {
    pgo::poses_vector_t poses = graph.poses();
    SE2 previous_pose = SE2(0, 0, 0);
    for (pgo::pose_node &node : poses) {
        /* add vertices */
        if (node.id == 0) {
            graph.pgoAddVertex(node.id, node.pose, true);
        } else {
            graph.pgoAddVertex(node.id, node.pose);
        }
        /* add edges */
        if (node.id > 0) {
            graph.pgoAddEdge(node.id - 1, node.id, previous_pose.inverse() * node.pose, Eigen::Matrix3d::Identity() * 100);
        }
        previous_pose = node.pose;
    }
    // save the graph to the file
    string path_before = "/home/ros/catkin_ws/src/darko/slam2d/launch/graph_before.g2o";
    graph.pgoSave(path_before);

    graph.pgoOptimize(10);
    string path_after = "/home/ros/catkin_ws/src/darko/slam2d/launch/graph_after.g2o";
    graph.pgoSave(path_after);

    graph.pgoClear();

    return;
}

void sigint_handler(int signum) {
    cout << "Get the SIGINT signal" << endl;
    g_shutdown_requested = true;
    return;
}
void sigterm_handler(int signum) {
    cout << "Get the SIGTERM signal" << endl;
    g_shutdown_requested = true;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam2d", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigterm_handler);
    ros::NodeHandle nh("~");
    paramInit(nh);

    ROS_INFO("Initilization ends");

    if (slam.bool_use_robot_odom && slam.bool_use_pose_graph) {
        ROS_INFO("Use the robot odometry and conduct pose-graph optimization.");
        slam.sub_laserscan = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, slam.topic_laser_scan, 100);
        slam.sub_nav_odom  = new message_filters::Subscriber<nav_msgs::Odometry>(nh, slam.topic_robot_odom, 100);
        slam.sync_odom_laser = new Synchronizer<LaserOdomSync>(LaserOdomSync(100), *(slam.sub_laserscan), *(slam.sub_nav_odom));
        slam.sync_odom_laser->registerCallback(boost::bind(&laserOdomPoseGraphSyncCallback, _1, _2));
    } else if (slam.bool_use_robot_odom && !slam.bool_use_mcl) {
        ROS_INFO("Use the robot odometry.");
        /* TODO: publish point cloud2 topics ? */
        slam.sub_laserscan = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, slam.topic_laser_scan, 100);
        slam.sub_nav_odom  = new message_filters::Subscriber<nav_msgs::Odometry>(nh, slam.topic_robot_odom, 100);
        slam.sync_odom_laser = new Synchronizer<LaserOdomSync>(LaserOdomSync(100), *(slam.sub_laserscan), *(slam.sub_nav_odom));
        slam.sync_odom_laser->registerCallback(boost::bind(&laserOdomSyncCallback, _1, _2));
    } else if (slam.bool_use_robot_odom && slam.bool_use_mcl) {
        ROS_INFO("Use the robot odometry and use particle filter localization");
        slam.sub_laserscan = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, slam.topic_laser_scan, 100);
        slam.sub_nav_odom  = new message_filters::Subscriber<nav_msgs::Odometry>(nh, slam.topic_robot_odom, 100);
        slam.sync_odom_laser = new Synchronizer<LaserOdomSync>(LaserOdomSync(100), *(slam.sub_laserscan), *(slam.sub_nav_odom));
        slam.sync_odom_laser->registerCallback(boost::bind(&laserOdomMCLSyncCallback, _1, _2));
    } else if (slam.bool_use_wheel_odom_as_prior && !slam.bool_use_robot_odom) {
        /* Use the lidar odom but use the wheel odom as the initial guess*/
        ROS_INFO("Use the lidar odometry and use the wheel odometry as the initial guess");
        slam.sub_laserscan = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, slam.topic_laser_scan, 100);
        slam.sub_nav_odom  = new message_filters::Subscriber<nav_msgs::Odometry>(nh, slam.topic_robot_odom, 100);
        slam.sync_odom_laser = new Synchronizer<LaserOdomSync>(LaserOdomSync(100), *(slam.sub_laserscan), *(slam.sub_nav_odom));
        slam.sync_odom_laser->registerCallback(boost::bind(&laserOdomWheelOdomSyncCallback, _1, _2));
    } else {
        ROS_INFO("Use the lidar odometry only");
        /* Pay attention to the pointer and real entility */
        slam.sub_laserscan_single = nh.subscribe<sensor_msgs::LaserScan>(slam.topic_laser_scan, 100, laserscan_callback);
    }

    /* register topics */
    pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/laserscan", 100);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/est_pose", 100);
    pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 100);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 100);
    pub_map2d = nh.advertise<nav_msgs::OccupancyGrid>("/map", 100);
    // ros::spin();
    while(ros::ok() && !g_shutdown_requested) {
        ros::spinOnce();
    }

    if (slam.bool_use_pose_graph) {
        deal_graph();
    }
    return 0;
}