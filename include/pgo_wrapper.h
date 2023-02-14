/* The object to store the information of a graph */
#ifndef __PGO_WRAPPER_H__
#define __PGO_WRAPPER_H__

#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// g2o solver header files
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
using namespace g2o;

class pgo
{
public:
    /* pose of the robot */
    struct pose_node {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int id;
    SE2 pose;
    };
    typedef std::vector<pose_node, Eigen::aligned_allocator<pose_node> > poses_vector_t;

    /* odometry constraint */
    struct pose_edge {
    int from;
    int to;
    SE2 measurement;
    Eigen::Matrix3d information;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    typedef std::vector<pose_edge, Eigen::aligned_allocator<pose_edge> > pose_edge_t;

public:
    /* initialize the optimizer */
    pgo();
    ~pgo();

    const poses_vector_t& poses() const { return _pose;}
    const pose_edge_t& odom() const { return _odom;}

    const void pgoPushBackNode(pose_node node) { _pose.push_back(node); }
    const void pgoPushBackEdge(pose_edge edge) { _odom.push_back(edge); }

    /* save the g2o file */
    void pgoSave(const std::string& filename) { _optimizer.save(filename.c_str()); }
    /* clear the optimizer */
    void pgoClear() { _optimizer.clear(); }

    /* add vertex to the pose graph */
    void pgoAddVertex(int id, const SE2& pose, bool fixed = false);
    /* add edge to the pose graph */
    void pgoAddEdge(int from, int to, const SE2& measurement, const Eigen::Matrix3d& information);

    /* optimize the pose graph */
    void pgoOptimize(int iterations = 10);


private:
    poses_vector_t _pose;
    pose_edge_t _odom;
    SparseOptimizer _optimizer;
};


#endif /* define __PGO_WRAPPER_H__ */