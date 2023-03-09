#include "slam_demo/helper.h"

MatrixSE2 ang2Mat(double ang) {
    MatrixSE2 mat;
    mat(0, 0) = cos(ang);
    mat(0, 1) = -sin(ang);
    mat(1, 0) = sin(ang);
    mat(1, 1) = cos(ang);
    mat(0, 2) = 0;
    mat(1, 2) = 0;
    mat(2, 0) = 0;
    return mat;
}

/* convert 3d point cloud to 2d point cloud */
void point3d2point2d(const pcl::PointCloud<PointType3>::Ptr& cloud_in,
                     pcl::PointCloud<PointType>::Ptr& cloud_out,
                     double prune_dist) {
    if (prune_dist == 0) {
        /* Not pruning points */
        cloud_out->points.resize(cloud_in->points.size());
        for (unsigned int i = 0; i < cloud_in->points.size(); ++i) {
            cloud_out->points[i].x = cloud_in->points[i].x;
            cloud_out->points[i].y = cloud_in->points[i].y;
        }
    } else if (prune_dist > 0) {
        /* Pruning points */
        cloud_out->points.resize(cloud_in->points.size());
        unsigned int j = 0;
        for (unsigned int i = 0; i < cloud_in->points.size(); ++i) {
            if (cloud_in->points[i].x * cloud_in->points[i].x +
                    cloud_in->points[i].y * cloud_in->points[i].y <
                prune_dist * prune_dist) {
                cloud_out->points[j].x = cloud_in->points[i].x;
                cloud_out->points[j].y = cloud_in->points[i].y;
                j++;
            }
        }
        cloud_out->points.resize(j);
    } else {
        std::cout << "prune_dist should be non-negative" << std::endl;
    }
    return;
}

Eigen::Affine3d pose_stamped_to_eigen(
    const geometry_msgs::msg::PoseStamped& pose_msg) {
    Eigen::Affine3d eigen_pose = Eigen::Affine3d::Identity();

    // Extract position and orientation information from the PoseStamped message
    const auto& pos = pose_msg.pose.position;
    const auto& orient = pose_msg.pose.orientation;

    // Convert quaternion to rotation matrix using Eigen's Quaternion class
    Eigen::Quaterniond quat(orient.w, orient.x, orient.y, orient.z);
    eigen_pose.linear() = quat.toRotationMatrix();

    // Set the translation vector of the Eigen pose
    eigen_pose.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);

    return eigen_pose;
}

void convertVecToInfoMat(const std::vector<double>& vec, Eigen::MatrixXd& mat) {
    if (vec.size() == 6) {
        mat(0, 0) = vec[0];

        mat(1, 0) = vec[1];
        mat(0, 1) = vec[1];

        mat(0, 2) = vec[2];
        mat(2, 0) = vec[2];

        mat(1, 1) = vec[3];

        mat(1, 2) = vec[4];
        mat(2, 1) = vec[4];

        mat(3, 3) = vec[5];
    } else if (vec.size() == 3) {
        mat(0, 0) = vec[0];
        mat(1, 1) = vec[1];
        mat(2, 2) = vec[2];
    } else {
        std::cout << "Error: the size of the vector is not 3 or 6" << std::endl;
        exit(1);
    }
}