#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include "Pose2D.h"
#include <eigen3/Eigen/Dense>

/**
 * @brief GraphNode struct to be used inside the PoseGraph class.
 * A node stores the 2D pose of the robot, its id and the covariance
 * representing the uncertainty of the pose.
 */

struct GraphNode
{
    uint nodeId;
    Pose2D nodePose;
    Pose2D initPose;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    void setCovariance(const Eigen::Matrix3d& nodeCovariance)
    {
        covariance = nodeCovariance;
    }

    Eigen::Matrix3d getCovariance() const
    {
        return covariance;
    }
};

#endif // GRAPHNODE_H
