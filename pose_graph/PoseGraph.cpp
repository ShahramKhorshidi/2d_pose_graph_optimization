#include <iostream>
#include <chrono>
#include <stdexcept>
#include "PoseGraph.h"
#include<eigen3/Eigen/SparseCholesky>

uint PoseGraph::nodesCount = 0;

uint PoseGraph::getNodesCount()
{
    return nodesCount;
}

PoseGraph::PoseGraph()
{
    init();
}

PoseGraph::PoseGraph(const Pose2D &priorPose)
{
    init();
    addPriorNode(priorPose);
}

PoseGraph::PoseGraph(const Pose2D &priorPose, const Eigen::Matrix3d& covariance)
{
    init();
    addPriorNode(priorPose, covariance);
}

PoseGraph::~PoseGraph()
{
    nodesCount--;
}

void PoseGraph::init()
{
    defaultNodeCovariance = Eigen::Matrix3d::Zero();
    defaultNodeCovariance(0, 0) = 0.15 * 0.15; // std_x = 0.15 m
    defaultNodeCovariance(1, 1) = 0.15 * 0.15; // std_y = 0.15 m
    defaultNodeCovariance(2, 2) = 0.2 * 0.2;   // std_theta = 0.2 rad

    /** @todo: Assign a separate covariance to each constraint. */
    Eigen::Matrix3d constraintCovariance = defaultNodeCovariance;
    omega = constraintCovariance.inverse();
    correctingPose = Pose2D(0, 0, 0);
    graphOptimized = false;
}

void PoseGraph::addPriorNode(const Pose2D& priorPose)
{
    addPriorNode(priorPose, defaultNodeCovariance);
}

void PoseGraph::addPriorNode(
        const Pose2D& priorPose,
        const Eigen::Matrix3d& covariance)
{
    if (graphNodes.empty())
    {
        currentNode.nodeId = nodesCount;
        currentNode.nodePose = priorPose;
        currentNode.initPose = priorPose;
        currentNode.setCovariance(covariance);
        currentNode.clearMarginalCovariance();
        graphNodes.push_back(currentNode);
        nodesCount++;
    }
}

void PoseGraph::addOdomNode(const Pose2D &odomPose)
{
    addOdomNode(odomPose, defaultNodeCovariance);
}

void PoseGraph::addOdomNode(
        const Pose2D &odomPose,
        const Eigen::Matrix3d& covariance)
{
    currentNode.nodeId = nodesCount;
    // Odometry poses are corrected by the error we get for the last pose, after optimization.
    currentNode.nodePose = odomPose;
    currentNode.initPose = currentNode.nodePose;
    currentNode.setCovariance(covariance);
    currentNode.clearMarginalCovariance();

    graphNodes.push_back(currentNode);
    nodesCount++;

    if (graphNodes.size() > 1)
    {
        uint i = graphNodes.size();
        currentEdge.addOdomConstraint(graphNodes[i-2], graphNodes[i-1]);
        graphConstraints.push_back(currentEdge);
    }
}

void PoseGraph::setNodeCovariance(uint nodeId, const Eigen::Matrix3d& covariance)
{
    graphNodes.at(nodeId).setCovariance(covariance);
}

Eigen::Matrix3d PoseGraph::getNodeCovariance(uint nodeId) const
{
    return graphNodes.at(nodeId).getCovariance();
}

Eigen::Matrix3d PoseGraph::getDefaultNodeCovariance() const
{
    return defaultNodeCovariance;
}

const Eigen::SparseMatrix<double>& PoseGraph::getInformationMatrix() const
{
    return informationMatrix;
}

Eigen::Matrix3d PoseGraph::getMarginalCovariance(uint nodeId) const
{
    const GraphNode& node = graphNodes.at(nodeId);
    if (!node.hasMarginalCovariance())
    {
        throw std::logic_error(
                "Marginal covariance is available only after the node has been optimized.");
    }
    return node.getMarginalCovariance();
}

void PoseGraph::connectLoopClosingNodes(uint nodeId1, uint nodeId2, const Pose2D &observation)
{
    GraphNode& nodei = graphNodes[nodeId1];
    GraphNode& nodej = graphNodes[nodeId2];
    currentEdge.addLoopClosingConstraint(nodei, nodej, observation);
    graphConstraints.push_back(currentEdge);
}

void PoseGraph::optimizeGraph(const GraphNode &node1, const GraphNode &node2)
{
    auto opt_start = std::chrono::high_resolution_clock::now();
    // Storing the last pose of the robot before the optimization.
    Pose2D beforeOptimization = graphNodes[nodesCount-1].nodePose;
    
    // Creating the linear sysyem.
    uint NoOfNodes = node2.nodeId - node1.nodeId + 1;
    uint dim = 3 * NoOfNodes;
    LinearSystem ls(dim);

    // Initial guess for ls.x, we set it as current graph nodes pose.
    for (uint i = 0; i < NoOfNodes; i++)
    {
        Pose2D pose = graphNodes[node1.nodeId+i].nodePose;
        ls.x(3*i) = pose.x;
        ls.x(3*i+1) = pose.y;
        ls.x(3*i+2) = pose.theta;
    }

    uint numOfIter = 0;
    while(!converged)
    {
        /** @brief Linearizing the least-squares problem. */
        buildLinearSystem(ls, node1.nodeId, node2.nodeId);

        /** @brief Sparse solver. */
        ls.HSparse = ls.H.sparseView();

        ls.solver.compute(ls.HSparse);
        if(ls.solver.info()!=Eigen::Success)
        {
          std::cout << "Decomposition failed at Iteration#: " << numOfIter << std::endl;
          return;
        }

        ls.dx = ls.solver.solve(ls.b);
        if(ls.solver.info()!=Eigen::Success)
        {
          std::cout << "Solving failed at Iteration#: " << numOfIter << std::endl;
          return;
        }
        // Update the parameters.
        ls.x += ls.dx;

        // Update the grpah poses and constraints, at each iteration.
        updateGraphPoses(ls.x, node1.nodeId, node2.nodeId);
        updateGraphConstraints();
        numOfIter++;

        if(ls.dx.norm() < ls.tolerance || numOfIter > ls.maxIter)
        {
            auto opt_stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(opt_stop - opt_start);
            std::cout << "\n==== Optimization Successful! ==== " << 
                         "\n --- Total time=" << duration.count() << "(ms)" <<
                         "\n --- No of Iterations=" << numOfIter << 
                         "\n --- Delta_X Norm=" << ls.dx.norm() << std::endl;
            // Storing the correcting pose for the last pose after optimization.
            Pose2D& afterOptimization = graphNodes[nodesCount-1].nodePose;
            correctingPose = afterOptimization.diff(beforeOptimization);
            converged = true;
        }
    }

    // Linearize once more at the final optimized poses before computing covariance.
    buildLinearSystem(ls, node1.nodeId, node2.nodeId);
    ls.HSparse = ls.H.sparseView();
    updateMarginalCovariances(ls, node1.nodeId, node2.nodeId);

    // Setting the condition false, for the next optimization loop.
    converged = false;

    // Updating the line map with the new pose configuration.
    graphOptimized = true;
}

void PoseGraph::updateGraphPoses(const Eigen::VectorXd &poseVector, uint nodeId1, uint nodeId2)
{
    uint NoOfNodes = nodeId2 - nodeId1 + 1;
    for (uint i = 0; i < NoOfNodes; i++)
    {
        double x = poseVector(3*i);
        double y = poseVector(3*i+1);
        double z = poseVector(3*i+2);
        Pose2D newPose = Pose2D(x, y, z);

        graphNodes[nodeId1+i].nodePose = newPose;
    }
}

void PoseGraph::updateGraphConstraints()
{
    for (auto& edge : graphConstraints)
    {
        uint i = edge.i;
        uint j = edge.j;
        edge.updateConstraint(graphNodes[i], graphNodes[j]);
    }
}

void PoseGraph::buildLinearSystem(LinearSystem& ls, uint nodeId1, uint nodeId2)
{
    ls.H = Eigen::MatrixXd::Zero(ls.dimension, ls.dimension);
    ls.b = Eigen::VectorXd::Zero(ls.dimension);

    for (auto& edge : graphConstraints)
    {
        // We only optimize constraints whose two nodes are inside the selected interval.
        if (edge.i >= nodeId1 && edge.j <= nodeId2)
        {
            edge.linearizeConstraint();
            uint i = edge.i-nodeId1;
            uint j = edge.j-nodeId1;
            ls.H.block<3, 3>(3*i, 3*i) += edge.getAij().transpose() * omega * edge.getAij();
            ls.H.block<3, 3>(3*i, 3*j) += edge.getAij().transpose() * omega * edge.getBij();
            ls.H.block<3, 3>(3*j, 3*i) += edge.getBij().transpose() * omega * edge.getAij();
            ls.H.block<3, 3>(3*j, 3*j) += edge.getBij().transpose() * omega * edge.getBij();

            ls.b.block<3, 1>(3*i, 0) += edge.getAij().transpose() * omega * edge.getErrorVector();
            ls.b.block<3, 1>(3*j, 0) += edge.getBij().transpose() * omega * edge.getErrorVector();
        }
    }

    // Keep the first node fixed and remove the global reference-frame ambiguity.
    ls.H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity();
}

void PoseGraph::updateMarginalCovariances(
        LinearSystem& ls,
        uint nodeId1,
        uint nodeId2)
{
    informationMatrix = ls.HSparse;
    for (uint nodeId = nodeId1; nodeId <= nodeId2; nodeId++)
    {
        graphNodes[nodeId].clearMarginalCovariance();
    }

    ls.solver.compute(informationMatrix);
    if (ls.solver.info() != Eigen::Success)
    {
        std::cout << "Decomposition failed while computing marginal covariances." << std::endl;
        return;
    }

    for (uint nodeId = nodeId1; nodeId <= nodeId2; nodeId++)
    {
        uint localNodeId = nodeId-nodeId1;
        Eigen::MatrixXd selector = Eigen::MatrixXd::Zero(ls.dimension, 3);
        selector.block<3, 3>(3*localNodeId, 0) = Eigen::Matrix3d::Identity();

        Eigen::MatrixXd covarianceColumns = ls.solver.solve(selector);
        if (ls.solver.info() != Eigen::Success)
        {
            std::cout << "Solving failed while computing marginal covariance for node "
                      << nodeId << "." << std::endl;
            return;
        }

        const Eigen::Matrix3d covariance =
                covarianceColumns.block<3, 3>(3*localNodeId, 0);
        const Eigen::Matrix3d symmetricCovariance =
                0.5 * (covariance + covariance.transpose());
        graphNodes[nodeId].setMarginalCovariance(symmetricCovariance);
    }
}

Pose2D PoseGraph::getCorrectingPose() const
{
    return correctingPose;
}
