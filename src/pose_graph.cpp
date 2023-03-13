#include "slam_demo/pose_graph.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "rclcpp/rclcpp.hpp"

PoseGraph::PoseGraph() {
    // define the used optimizer
    optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    optimizer_->setVerbose(false);

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto linearSolver = g2o::make_unique<LinearSolverType>();
    auto solver = g2o::make_unique<BlockSolverType>(std::move(linearSolver));
    g2o::OptimizationAlgorithmGaussNewton* algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver));

    optimizer_->setAlgorithm(algorithm);
}

g2o::VertexSE2* PoseGraph::addSE2Node(const MatrixSE2& pose) {
    // create a new node
    g2o::VertexSE2* node = new g2o::VertexSE2();
    node->setId(optimizer_->vertices().size());
    node->setEstimate(pose);
    if (optimizer_->vertices().size() == 0) /* set the first frame as the fixed one */
        node->setFixed(false);
    else
        node->setFixed(false);
    optimizer_->addVertex(node);

    return node;
}

g2o::VertexSE2* PoseGraph::addSE2Node(const MatrixSE2& pose, bool fixed) {
    // create a new node
    g2o::VertexSE2* node = new g2o::VertexSE2();
    node->setId(optimizer_->vertices().size());
    node->setEstimate(pose);
    if (fixed)
        node->setFixed(true);
    optimizer_->addVertex(node);

    return node;
}

g2o::EdgeSE2* PoseGraph::addSE2Edge(const g2o::VertexSE2* from,
                                    const g2o::VertexSE2* to,
                                    const MatrixSE2& relative_pose,
                                    const Eigen::Matrix3d& info_matrix) {
    // create a new edge
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = optimizer_->vertex(from->id());
    edge->vertices()[1] = optimizer_->vertex(to->id());
    edge->setMeasurement(relative_pose);
    edge->setInformation(info_matrix);
    optimizer_->addEdge(edge);

    return edge;
}

g2o::EdgeSE2* PoseGraph::addSE2Edge(const int prev_id,
                                    const int cur_id,
                                    const MatrixSE2& relative_pose,
                                    const Eigen::Matrix3d& info_matrix) {
    // create a new edge
    if (prev_id < 0 || cur_id < 0) {
        RCLCPP_WARN(rclcpp::get_logger("PoseGraph"),
                    "The idx of frames is less than 0, prev_id: %d, cur_id: %d",
                    prev_id,
                    cur_id);
        return nullptr;
    }
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = optimizer_->vertex(prev_id);
    edge->vertices()[1] = optimizer_->vertex(cur_id);
    edge->setMeasurement(relative_pose);
    edge->setInformation(info_matrix);
    optimizer_->addEdge(edge);

    return edge;
}

void PoseGraph::optimize(unsigned int num_iterations) {
    optimizer_->initializeOptimization();
    // set the number of iterations
    optimizer_->optimize(num_iterations);
}

void PoseGraph::saveGraph(const std::string& file_name) {
    RCLCPP_INFO(rclcpp::get_logger("PoseGraph"), "Saving graph to %s", file_name.c_str());
    optimizer_->save(file_name.c_str());
}