#ifndef __SLAM2D_H
#define __SLAM2D_H

/* The front-end odometer */

#include <helper.h>
#include "optimization.h"

#include <iostream>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

// libpointmatcher
#include "pointmatcher/PointMatcher.h"


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef Eigen::Matrix<double, 3, 3> MatrixSE2;

typedef struct
{
    double theta;
    Eigen::Vector2d t;

} state2d;

Eigen::Vector2d point2eigen(PointType p)
{
    Eigen::Vector2d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    return pp;
}

PointType eigen2point(Eigen::Vector2d pp)
{
    PointType p;
    p.x = pp(0);
    p.y = pp(1);
    return p;
}

class slam2d
{
private:
    laser_geometry::LaserProjection laser_projector;
    Eigen::Affine3d transform_robot_to_sensor;
public:
    slam2d(/* args */);
    ~slam2d();
    state2d state;
    state2d delta;
    state2d prev_state;
    state2d cur_wheel_odom_;
    state2d prev_wheel_odom_;
    double timestamp;
    nav_msgs::OccupancyGrid map2d;
    Mat cvmap2d;

    pcl::PointCloud<PointType> scan;
    pcl::PointCloud<PointType> scan_prev;

    /* Frames setting */
    string frame_base_link;
    string frame_sensor_link;
    /* Topics setting */
    string topic_laser_scan;
    string topic_robot_odom;
    /* Control parameters */
    bool bool_use_robot_odom;
    bool bool_use_pose_graph;
    bool bool_use_mcl;
    bool bool_darko_cfg;
    bool bool_use_wheel_odom_as_prior;

    /* if using odom */
    message_filters::Subscriber<sensor_msgs::LaserScan> *sub_laserscan;
    message_filters::Subscriber<nav_msgs::Odometry>     *sub_nav_odom;
    message_filters::Synchronizer<LaserOdomSync>        *sync_odom_laser;
    /* If only listen to the laser scan */
    ros::Subscriber sub_laserscan_single;
    bool cvmap_vis_enable = false;

    void readin_scan_data(const sensor_msgs::MultiEchoLaserScanConstPtr &msg);
    void readin_scan_data(const sensor_msgs::LaserScanConstPtr &msg);
    /* If using original odom of the robot, turn the read laser scan back to the robot */
    void readInLaserScanInRobotCor(const sensor_msgs::LaserScanConstPtr &msg);

    Vector2d world2map(Vector2d p);
    cv::Point2i world2map(cv::Point2f p);

    void scan_match();
    void scan_match_libpointmatcher();
    void scan_map_match_random();
    int scan_map_match_score(Vector3d pose);
    void update();
    void update_transform();

    void bresenham(Point2i p1, Point2i p2);
    void update_map();
    void cvmap2map();//convert cv map to map
    void setUseOdom(bool use_odom_flag);
    bool getUseOdom();
    void scanToMap();
    void updateStateByOdom(const nav_msgs::OdometryConstPtr& msg);
    void setRobotToSensorTrans(Eigen::Affine3d trans);
    void getDeltaMove(double delta_move[3]);
    void updateState(Vector3d pose);
    void updateWheelOdom(const nav_msgs::OdometryConstPtr& msg);

    MatrixSE2 getLatestWheelOdom();
    MatrixSE2 getLatestOdom();

    void addWheelOdom(MatrixSE2 odom);
    void addOdom(MatrixSE2 odom);
protected:
    // The final estimated odom states
    std::vector<MatrixSE2> odom_poses_;
    // The wheel odom states
    std::vector<MatrixSE2> wheel_odom_poses_;
    MatrixSE2 relative_wheel_odom_guess_;

    // The subscriber and publisher topics
    ros::Subscriber wheel_odom_sub_;
    ros::Subscriber laser_scan_sub_;

    bool relative_wheel_odom_init_ = false;

};

slam2d::slam2d()
{
    state.t = Vector2d::Zero();
    state.theta = 0;
    delta.t = Vector2d::Zero();
    delta.theta = 0;
    prev_state.t = Vector2d::Zero();
    prev_state.theta = 0;
    bool_use_robot_odom = false;
    bool_use_mcl = false;
    map2d.header.frame_id = "odom";
    map2d.info.width = 2000;
    map2d.info.height = 2000;
    map2d.info.resolution = 0.15;
    map2d.info.origin.orientation.w = 1;
    map2d.info.origin.orientation.x = 0;
    map2d.info.origin.orientation.y = 0;
    map2d.info.origin.orientation.z = 0;
    map2d.info.origin.position.x = -0.5 * map2d.info.width * map2d.info.resolution;
    map2d.info.origin.position.y = -0.5 * map2d.info.height * map2d.info.resolution;
    map2d.info.origin.position.z = 0;
    map2d.data.resize(map2d.info.width * map2d.info.height);
    cvmap2d = Mat(map2d.info.width, map2d.info.height, CV_8SC1, -1);
    cvmap2map();
    odom_poses_.push_back(MatrixSE2::Identity());
}

slam2d::~slam2d()
{
}

void slam2d::readin_scan_data(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    timestamp = msg->header.stamp.toSec();
    scan.points.resize(msg->ranges.size());
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        float dist = msg->ranges[i].echoes[0]; //only first echo used for slam2d
        float theta = msg->angle_min + i * msg->angle_increment;
        scan.points[i].x = dist * cos(theta);
        scan.points[i].y = dist * sin(theta);
    }
    scan.width = scan.points.size();
    scan.height = 1;
    scan.is_dense = true;
}

void slam2d::readin_scan_data(const sensor_msgs::LaserScanConstPtr &msg)
{
    timestamp = msg->header.stamp.toSec();
    sensor_msgs::PointCloud2 pc_2;
    laser_projector.projectLaser(*msg, pc_2);
    /* If use odom, need to sychronize the coordinate of the scan with the robot */
    if (bool_darko_cfg) {
        pcl::PointCloud<PointType3> scan3d;
        pcl::fromROSMsg(pc_2, scan3d);
        scan3d.width = scan3d.points.size();
        scan3d.height = 1;
        scan3d.is_dense = true;
        /* transform the read in point cloud to the coordinate of the robot, represented in the robot */
        pcl::transformPointCloud(scan3d, scan3d, transform_robot_to_sensor);
        point3d2point2d(scan3d, scan);
    } else {
        pcl::fromROSMsg(pc_2, scan);
    }
    scan.width = scan.points.size();
    scan.height = 1;
    scan.is_dense = true;
    return;
}

cv::Point2i slam2d::world2map(cv::Point2f p)
{
    cv::Point2i m;
    m.x = roundf(p.x / map2d.info.resolution + map2d.info.width * 0.5);
    m.y = roundf(p.y / map2d.info.resolution + map2d.info.height * 0.5);
    return m;
}

Vector2d slam2d::world2map(Vector2d p)
{
    Vector2d m;
    m = p / map2d.info.resolution;
    m(0) += map2d.info.width * 0.5;
    m(1) += map2d.info.height * 0.5;
    return m;
}

void slam2d::scan_match()
{
    double pose[3] = {0};
    /* very good way to skip the initial pose */
    if (scan.points.size() && scan_prev.points.size())
    {

        Problem problem;
        //solve delta with ceres constraints
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(scan.makeShared());
        int K = 2; // K nearest neighbor search
        std::vector<int> index(K);
        std::vector<float> distance(K);
        //1. project scan_prev to scan

        Eigen::Matrix2d R;
        R(0, 0) = cos(delta.theta); R(0, 1) = -sin(delta.theta);
        R(1, 0) = sin(delta.theta); R(1, 1) = cos(delta.theta);
        Eigen::Vector2d dt = delta.t;
        //find nearest neighur
        for (int i = 0; i < scan_prev.points.size(); i++)
        {
            PointType search_point = scan_prev.points[i];
            //project search_point to current frame
            PointType search_point_predict = eigen2point(R * point2eigen(search_point) + dt);
            if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K)
            {
                //add constraints
                Eigen::Vector2d p = point2eigen(search_point);
                Eigen::Vector2d p1 = point2eigen(scan.points[index[0]]);
                Eigen::Vector2d p2 = point2eigen(scan.points[index[1]]);
                ceres::CostFunction *cost_function = lidar_edge_error::Create(p, p1, p2);
                problem.AddResidualBlock(cost_function,
                                         new CauchyLoss(0.5),
                                         pose);
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.FullReport() << "\n";

        // printf("result: %lf, %lf, %lf\n", pose[0], pose[1], pose[2]);

        delta.theta = pose[0];
        delta.t(0) = pose[1];
        delta.t(1) = pose[2];

    }
}

/* /brief: scan matching using libpointmatcher
 *
 */
void slam2d::scan_match_libpointmatcher()
{
    double pose[3] = {0};
    /* Create the matcher module */
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;

    std::string name;
    /* set the params */
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
    /* copy the scan data to ref */
    data.features.resize(3, scan_prev.points.size());
    for (int i = 0; i < scan_prev.points.size(); i++)
    {
        data.features(0, i) = scan_prev.points[i].x;
        data.features(1, i) = scan_prev.points[i].y;
        data.features(2, i) = 1;
    }
    /* copy the scan data to data */
    ref.features.resize(3, scan.points.size());
    for (int i = 0; i < scan.points.size(); i++)
    {
        ref.features(0, i) = scan.points[i].x;
        ref.features(1, i) = scan.points[i].y;
        ref.features(2, i) = 1;
    }
    /* set the initial guess */
    PM::TransformationParameters T = PM::TransformationParameters::Identity(3, 3);
    if (bool_use_wheel_odom_as_prior && relative_wheel_odom_init_) {
        ROS_INFO("use wheel odom as prior");
        T(0, 2) = relative_wheel_odom_guess_(0,2);
        T(1, 2) = relative_wheel_odom_guess_(1,2);
        T(0, 0) = relative_wheel_odom_guess_(0,0);
        T(0, 1) = relative_wheel_odom_guess_(0,1);
        T(1, 0) = relative_wheel_odom_guess_(1,0);
        T(1, 1) = relative_wheel_odom_guess_(1,1);
    }
    /* The registration result */
    PM::TransformationParameters T_final = icp(ref, data, T);
    ROS_INFO("Run ICP with libpointmatcher");
    // PM::TransformationParameters T_final = icp(ref, data);
    // Update the pose
    MatrixSE2 latest_pose;
    if (odom_poses_.size() == 0) {
        // set and idtntity matrix
        latest_pose = MatrixSE2::Identity();
    } else {
        latest_pose = odom_poses_.back();
    }
    MatrixSE2 T_res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            T_res(i, j) = T_final(i, j);
    odom_poses_.push_back(latest_pose * T_res);
    return;
}

int slam2d::scan_map_match_score(Vector3d pose)
{
    int score = 0;
    Eigen::Matrix2d R;
    Vector2d t(pose(1), pose(2));
    double theta = pose(0);
    R(0, 0) = cos(theta); R(0, 1) = -sin(theta);
    R(1, 0) = sin(theta); R(1, 1) =  cos(theta);
    //printf("cols: %d, rows: %d\n", cvmap2d.cols, cvmap2d.rows);
    for (int i = 0; i < scan.points.size(); i++)
    {
        Vector2d p = point2eigen(scan.points[i]);
        Vector2d pp = world2map(R * p + t);
        //cout << "pp: " << pp.transpose() << endl;
        if ((pp(0) <= 1) || (pp(0) >= cvmap2d.cols) || (pp(1) <= 1) || (pp(1) >= cvmap2d.rows))
        {
            continue;
        } else
        {
            //get value from map
            int x = round(pp(0));
            int y = round(pp(1));
            if (cvmap2d.at<int8_t>(y * cvmap2d.cols + x) == 100)
            {
                score++;
            }
            //printf("i: %d, res:%f\n", i, residual[i]);
        }
    }
    //generate local map and compute local optimal?
    return score;
}

void slam2d::scan_map_match_random()
{
    Vector3d pose(state.theta, state.t(0), state.t(1));
    double eps = 1e-5;
    //search best mattch
    int N = 200;

    for (int i = 0; i < N; i++)
    {
        //random direction
        Vector3d d = Vector3d::Random();
        d(0) /= 10.0;
        d.normalize();
        double min_len = 0;
        double max_len = 0.2;
        //search best len
        while((max_len - min_len) > eps)
        {
            int score1 = scan_map_match_score(pose + d * min_len);
            int score2 = scan_map_match_score(pose + d * max_len);
            if (score1 >= score2)
            {
                max_len = (min_len + max_len) / 2.0;
            } else
            {
                min_len = (min_len + max_len) / 2.0;
            }
        }
        pose += d * min_len;
        Vector3d dx = d * min_len;
        int score = scan_map_match_score(pose);
        // printf("score: %d, min_len: %lf\n", score, min_len);
        // cout << "dx: " << dx.transpose() << endl;
    }
    //update to state
    state.theta = pose(0);
    state.t = pose.bottomRows(2);
}

void slam2d::update_transform()
{

    Eigen::Matrix2d dR;
    dR(0, 0) = cos(delta.theta); dR(0, 1) = -sin(delta.theta);
    dR(1, 0) = sin(delta.theta); dR(1, 1) = cos(delta.theta);


    Eigen::Vector2d dt_inv = -dR.transpose() * delta.t;
    Eigen::Matrix2d dR_inv = dR.transpose();
    Eigen::Matrix2d R;
    R(0, 0) = cos(state.theta); R(0, 1) = -sin(state.theta);
    R(1, 0) = sin(state.theta); R(1, 1) =  cos(state.theta);
    state.theta += (-delta.theta);
    state.t += R * dt_inv;
}

void slam2d::update()
{
    static int cnt = 0;
    if (scan.points.size() && scan_prev.points.size())
    {
        /* relative transform */
        // scan_match();
        scan_match_libpointmatcher();
        /* global transform */
        // update_transform();
        // scan_map_match_random();
        // update_map();
    }

    if (scan.points.size())
    {
        scan_prev = scan;
    }
    cnt++;
}

void slam2d::bresenham(Point2i p1, Point2i p2)
{
    //drawing a line from p1 to p2
    int dx = abs(p2.x - p1.x);
    int sx = (p2.x > p1.x) ? 1 : -1;
    int dy = abs(p2.y - p1.y);
    int sy = (p2.y > p1.y) ? 1 : -1;
    int err = (dx > dy ? dx : dy) / 2;
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    while (x1 != x2 && y1 != y2)
    {
        if (cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) == 100)
        {
            break;
        }
        else if (cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) == -1)
        {
            cvmap2d.at<int8_t>(y1 * cvmap2d.cols + x1) = 0;
        }
        int e2 = err;
        if (e2 > -dx)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y1 += sy;
        }
    }
}

void slam2d::update_map()
{
    //update map with scan and state
    cv::Point2f tt;
    tt.x = state.t(0);
    tt.y = state.t(1);
    cv::Point2i origin = world2map(tt);
    if (origin.x < 0 || origin.x >= cvmap2d.cols || origin.y < 0 || origin.y >= cvmap2d.rows) {
        ROS_ERROR("Map error");
        return;
    }
    Eigen::Matrix2d R;
    R(0, 0) = cos(state.theta); R(0, 1) = -sin(state.theta);
    R(1, 0) = sin(state.theta); R(1, 1) =  cos(state.theta);
    for (int i = 0; i < scan.points.size(); i++)
    {
        PointType p = scan.points[i];
        float dist = sqrtf(p.x * p.x + p.y * p.y);
        if (dist > 20) continue;
        Eigen::Vector2d pp = R * point2eigen(p) + state.t;
        Point2f ppp(pp(0), pp(1));

        cv:Point2i pt = world2map(ppp);

        if (pt.x < 0 || pt.x >= cvmap2d.cols || pt.y < 0 || pt.y >= cvmap2d.rows)
            return;

        bresenham(origin, pt);
        cvmap2d.at<int8_t>(pt.y * cvmap2d.cols + pt.x) = 100; //100->occupancied
    }
    cvmap2map();
}

void slam2d::updateStateByOdom(const nav_msgs::OdometryConstPtr& msg) {
    Eigen::Affine3d T;
    tf::poseMsgToEigen(msg->pose.pose, T);
    Eigen::Matrix3d r3 = T.rotation();
    Eigen::Vector3d t = T.translation();
    // Get the 2d rotation matrix
    Eigen::Matrix2d r2;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            r2(i,j) = r3(i,j);
        }
    }
    // Read the previous states
    prev_state.theta = state.theta;
    prev_state.t(0) = state.t(0);
    prev_state.t(1) = state.t(1);

    // Read the current states
    state.theta = rotToAngle(r2);
    state.t(0) = t(0);
    state.t(1) = t(1);
    return;
}

void slam2d::updateWheelOdom(const nav_msgs::OdometryConstPtr& msg) {
    Eigen::Affine3d T;
    tf::poseMsgToEigen(msg->pose.pose, T);
    Eigen::Matrix3d r3 = T.rotation();
    Eigen::Vector3d t = T.translation();
    // Get the 2d rotation matrix
    MatrixSE2 new_pose = MatrixSE2::Identity();
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            new_pose(i,j) = r3(i,j);
        }
    }
    for (int i = 0; i < 2; i++) {
        new_pose(i,2) = t(i);
    }
    // add the new pose
    if (wheel_odom_poses_.size() == 0) {
        wheel_odom_poses_.push_back(new_pose);
        return;
    }
    ROS_INFO("Adding a new pose to the wheel odom poses");
    MatrixSE2 prev_pose = wheel_odom_poses_.back();
    wheel_odom_poses_.push_back(new_pose);
    std::cout << "The previous pose is " << std::endl << prev_pose << std::endl;
    std::cout << "The new pose is " << std::endl << new_pose << std::endl;
    relative_wheel_odom_guess_ = prev_pose.inverse() * new_pose;
    std::cout << "The relative wheel odom guess is " << std::endl << relative_wheel_odom_guess_ << std::endl;
    relative_wheel_odom_init_ = true;
    return;
}

void slam2d::setRobotToSensorTrans(Eigen::Affine3d trans) {
    transform_robot_to_sensor = trans;
    ROS_INFO_STREAM("The set transform from robot to sensor" << endl << transform_robot_to_sensor.matrix());
    return;
}


void slam2d::cvmap2map()
{
    for (int i = 0; i < cvmap2d.rows; i++)
    {
        for(int j = 0; j < cvmap2d.cols; j++)
        {
            map2d.data[i * map2d.info.width + j] = cvmap2d.at<int8_t>(i, j);
        }
    }
    if (cvmap_vis_enable)
    {
        imshow("cvmap2d", cvmap2d);
        waitKey(2);
    }
}

void slam2d::getDeltaMove(double delta_move[3]) {
    delta_move[0] = state.t(0) - prev_state.t(0);
    delta_move[1] = state.t(1) - prev_state.t(1);
    delta_move[2] = state.theta - prev_state.theta;
    return;
}

void slam2d::updateState(Vector3d pose) {
    state.theta = pose[0];
    state.t(0) = pose[1];
    state.t(1) = pose[2];
}

MatrixSE2 slam2d::getLatestWheelOdom() {
    if (wheel_odom_poses_.size() == 0) {
        ROS_ERROR("No wheel odom poses available");
        return MatrixSE2::Identity();
    }
    return wheel_odom_poses_.back();
}

MatrixSE2 slam2d::getLatestOdom() {
    if (odom_poses_.size() == 0) {
        ROS_ERROR("No odom poses available");
        return MatrixSE2::Identity();
    }
    return odom_poses_.back();
}

void slam2d::addWheelOdom(MatrixSE2 pose) {
    wheel_odom_poses_.push_back(pose);
}

void slam2d::addOdom(MatrixSE2 pose) {
    odom_poses_.push_back(pose);
}
#endif
