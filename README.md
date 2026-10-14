# 2D Pose Graph Optimization

## Introduction

This repository contains a small C++ implementation of 2D pose graph optimization. Robot poses are represented as nodes, while odometry measurements and loop closures are represented as constraints between the nodes. The trajectory is corrected by minimizing the error of all constraints together.

The implementation follows the graph-based SLAM formulation described in [1, 2]. It uses Eigen for the linear algebra and Qt to show the recorded trajectory before and after a loop closure correction.

## Installation

### Dependencies

The project requires a C++17 compiler, CMake, Eigen3 and Qt5. On Ubuntu or Debian they can be installed with:

```
sudo apt update
sudo apt install build-essential cmake libeigen3-dev qtbase5-dev
```

Qt Creator is optional and can be used as an IDE, but it is not required to build or run the example.

### Source Code

The source code is hosted on GitHub: [ShahramKhorshidi/pose_graph_optimization](https://github.com/ShahramKhorshidi/pose_graph_optimization)

```
# Clone pose_graph_optimization
git clone https://github.com/ShahramKhorshidi/pose_graph_optimization.git
cd pose_graph_optimization
```

### Build

```
cmake -S . -B build
cmake --build build --parallel
```

### Quick Start

Run the example from the repository directory:

```
./build/example/RobotVisualization
```

The recorded trajectory and robot images are copied next to the executable during the CMake configuration. No path in `main.cpp` needs to be changed, and the executable can also be started from another working directory.

## Framework

Each node contains a 2D robot pose

$$
c_i = \begin{bmatrix} t_i \\ \theta_i \end{bmatrix}
    = \begin{bmatrix} x_i & y_i & \theta_i \end{bmatrix}^{T}.
$$

A constraint $z_{ij}$ describes the measured pose of node $j$ in the coordinate frame of node $i$. From the current node poses, its predicted value is

$$
h_{ij}(c_i,c_j) =
\begin{bmatrix}
R_i^{T}(t_j-t_i) \\
\theta_j-\theta_i
\end{bmatrix},
$$

where $R_i$ is the 2D rotation matrix of $\theta_i$. The constraint error and the total weighted error are

$$
e_{ij} = z_{ij}-h_{ij}(c_i,c_j), \qquad
F(c) = \sum_{(i,j)} e_{ij}^{T}\Omega_{ij}e_{ij}.
$$

The poses are stacked in one state vector $x$ and the error is linearized around its current value. With $J$ denoting the error Jacobian, one Gauss-Newton step is obtained from

$$
H\Delta x=-b, \qquad
H=\sum J^{T}\Omega J, \qquad
b=\sum J^{T}\Omega e.
$$

Only poses $i$ and $j$ occur in one constraint. Therefore, each edge adds only four $3\times3$ blocks to $H$: $H_{ii}$, $H_{ij}$, $H_{ji}$ and $H_{jj}$. This gives the pose graph its sparse block structure. The first pose is fixed to remove the global reference-frame ambiguity, and Eigen's sparse Cholesky solver is used to compute $\Delta x$. The poses and constraints are then updated until the increment is small.

## References

[1] G. Grisetti, R. Kümmerle, C. Stachniss and W. Burgard, "A Tutorial on Graph-Based SLAM," IEEE Intelligent Transportation Systems Magazine, 2010. [DOI: 10.1109/MITS.2010.939925](https://doi.org/10.1109/MITS.2010.939925)

[2] K. Konolige, G. Grisetti, R. Kümmerle, W. Burgard, B. Limketkai and R. Vincent, "Efficient Sparse Pose Adjustment for 2D Mapping," IEEE/RSJ International Conference on Intelligent Robots and Systems, 2010. [DOI: 10.1109/IROS.2010.5649043](https://doi.org/10.1109/IROS.2010.5649043)
