#ifndef __POSE_GRAPH_H
#define __POSE_GRAPH_H
#include "types.h"
#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/types_slam2d.h>


class PoseGraph {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PoseGraph();
    ~PoseGraph() {};

    /**
     * @brief add a pose node to the current graph, the node represents a pose in the world frame
     * @param the pose of the node
     * @return the pointer to the added node
    */
    g2o::VertexSE2* addSE2Node(const MatrixSE2& pose);

    /**
     * @brief add a pose edge to the current graph, the edge represents the relative pose between two nodes
     * @param the pointer to the from node
     * @param the pointer to the to node
     * @param the relative pose between the two nodes
     * @param the information matrix of the edge
     * @return the pointer to the added edge
    */
    g2o::EdgeSE2* addSE2Edge(const g2o::VertexSE2* from, const g2o::VertexSE2* to, const MatrixSE2& relative_pose, const Eigen::Matrix3d& info_matrix);

    /**
     * @brief optimize the current graph
    */
    void optimize();

    /**
     * @brief save the current graph to a file
     * @param the file name
    */
    void saveGraph(const std::string& file_name);

private:
    // define the used optimizer
    std::shared_ptr<g2o::SparseOptimizer> optimizer_;
};

#endif // __POSE_GRAPH_H