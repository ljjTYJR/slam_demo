#pragma once

// cunstomed
#include "types.h"
// Third party
#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/types_slam2d.h>

class PoseGraph {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    PoseGraph();
    ~PoseGraph() {};

    g2o::VertexSE2* addSE2Node(const MatrixSE2& pose, bool fixed);
    g2o::VertexSE2* addSE2Node(const MatrixSE2& pose);

    g2o::EdgeSE2* addSE2Edge(const g2o::VertexSE2* from, const g2o::VertexSE2* to, const MatrixSE2& relative_pose, const Eigen::Matrix3d& info_matrix);
    g2o::EdgeSE2* addSE2Edge(const int prev_id, const int cur_id, const MatrixSE2& relative_pose, const Eigen::Matrix3d& info_matrix);

    void optimize(unsigned int num_iterations = 50);
    void saveGraph(const std::string& file_name);

public:
    std::shared_ptr<g2o::SparseOptimizer> optimizer_;

private:

};
