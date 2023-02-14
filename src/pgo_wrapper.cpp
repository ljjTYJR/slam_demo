#include "pgo_wrapper.h"

/* initialize the optimizer */
pgo::pgo(/* args */)
{
    /* Need to optimize and clean */
    typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
    typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    auto linearSolver = make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(make_unique<SlamBlockSolver>(std::move(linearSolver)));
    _optimizer.setAlgorithm(solver);
    _optimizer.setVerbose(true);
    return;
}

pgo::~pgo()
{
    return;
}

/* add vertex to the pose graph
 * @id: the id of the vertex
 * @pose: the pose of the vertex
 */
void pgo::pgoAddVertex(int id, const SE2& pose, bool fixed)
{
    VertexSE2* v = new VertexSE2();
    v->setId(id);
    v->setEstimate(pose);
    v->setFixed(fixed);
    _optimizer.addVertex(v);
    return;
}

/* add edge to the pose graph
 * @from: the id of the from vertex
 * @to: the id of the to vertex
 */
void pgo::pgoAddEdge(int from, int to, const SE2& measurement, const Eigen::Matrix3d& information)
{
    EdgeSE2* e = new EdgeSE2();
    e->vertices()[0] = _optimizer.vertex(from);
    e->vertices()[1] = _optimizer.vertex(to);
    e->setMeasurement(measurement);
    e->setInformation(information);
    _optimizer.addEdge(e);
    return;
}

void pgo::pgoOptimize(int iterations)
{
    _optimizer.initializeOptimization();
    _optimizer.optimize(iterations);
    return;
}