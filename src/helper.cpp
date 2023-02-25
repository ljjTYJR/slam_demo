#include "helper.h"

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
void point3d2point2d(const pcl::PointCloud<PointType3>::Ptr &cloud_in, pcl::PointCloud<PointType>::Ptr &cloud_out) {
    cloud_out->points.resize(cloud_in->points.size());
    for (unsigned int i = 0; i < cloud_in->points.size(); ++i) {
        cloud_out->points[i].x = cloud_in->points[i].x;
        cloud_out->points[i].y = cloud_in->points[i].y;
    }
    return;
}

Eigen::Affine3d pose_stamped_to_eigen(const geometry_msgs::msg::PoseStamped& pose_msg)
{
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