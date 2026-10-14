#include <iostream>
#include <fstream>
#include <filesystem>
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QElapsedTimer>
#include "Pose2D.h"
#include "PoseGraph.h"

struct Trajectory {
    float id, x, y, angle;
};

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    // Create a QGraphicsScene
    QGraphicsScene scene;
    scene.setSceneRect(-400, -1400, 1000, 1600); // Set scene size

    // The example data is copied next to the executable by CMake
    std::filesystem::path exampleDir = QCoreApplication::applicationDirPath().toStdString();
    std::filesystem::path trajectoryPath = exampleDir / "robot_trajectory.log";

    // Read trajectory data from a file
    std::ifstream inputFile(trajectoryPath);
    if (!inputFile.is_open()) {
        std::cerr << "Error opening trajectory file: " << trajectoryPath << std::endl;
        return -1;
    }

    std::vector<Trajectory> trajectory;
    float id, x, y, angle;
    while (inputFile >> id >> x >> y >> angle) {
        trajectory.push_back({id, x, y, angle});
    }

    // Load an image for the robot
    QPixmap robotImageRed((exampleDir / "robot_image_red.png").c_str());
    QPixmap robotImageGreen((exampleDir / "robot_image_green.png").c_str());

    // Create a QElapsedTimer to measure time
    QElapsedTimer timer;
    timer.start();

    // Create a QGraphicsView to visualize the scene
    QGraphicsView view(&scene);
    view.scale(-1, 1);
    view.show();

    // Instantiating a pose graph object
    PoseGraph poseGraph = PoseGraph();

    // Read the robot trajectory
    for (const Trajectory& robotTraj : trajectory) {
        // Adding consecutive nodes to the pose graph with odometry constraints
        Pose2D odomPose = Pose2D(robotTraj.x, robotTraj.y, robotTraj.angle);
        poseGraph.addOdomNode(odomPose);
        if (PoseGraph::getNodesCount() == 1){
            std::cout << "--- Add nodes from odometry to the graph.---" << std::endl;
        }
        // Loop closing constraint
        if (PoseGraph::getNodesCount() == 262){
            std::cout << "--- Loop closing between nodeId=9 and nodeId=261 ---" << std::endl;
            std::cout << "--- Loop closing constraint: [x, y, thta] = [-0.905603, -0.340513, 0.185957] ---" << std::endl;
            // Relative Transformation between two poses from loop closing detection.
            uint nodeIdi = 9;
            uint nodeIdj = 261;
            Pose2D observation = Pose2D(-0.905603, -0.340513, 0.185957);
            poseGraph.connectLoopClosingNodes(nodeIdi, nodeIdj, observation);

            // We only optimize the graph between two loop closing nodes.
            GraphNode& from = poseGraph.graphNodes[nodeIdi];
            GraphNode& to = poseGraph.graphNodes[nodeIdj];
            poseGraph.optimizeGraph(from, to);
        }

        // Draw trajectory on the scene
        // Display the robot image at the current pose with different colors befor and after graph optimization
        float scale = 22.0;
        if (PoseGraph::getNodesCount() < 263) {
            QGraphicsPixmapItem* robotItem = new QGraphicsPixmapItem(robotImageRed.scaledToWidth(20));  
            robotItem->setPos(odomPose.x * scale, odomPose.y * scale);
            robotItem->setRotation(odomPose.theta * 180 / M_PI);  // Convert radians to degrees
            scene.addItem(robotItem);
        } else {
            QGraphicsPixmapItem* robotItem = new QGraphicsPixmapItem(robotImageGreen.scaledToWidth(20));
            robotItem->setPos(odomPose.x * scale, odomPose.y * scale);
            robotItem->setRotation(odomPose.theta * 180 / M_PI);  // Convert radians to degrees
            scene.addItem(robotItem);            
        }

        // Add a delay for animation (adjust as needed)
        if (PoseGraph::getNodesCount() == 262)
        {
            while (timer.elapsed() < 100) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                // Display the robot trajectory after optimization
                for (uint i = 9; i < poseGraph.graphNodes.size(); i++) {
                    QGraphicsPixmapItem* robotItem = new QGraphicsPixmapItem(robotImageGreen.scaledToWidth(20));  // Adjust the size as needed
                    robotItem->setPos(poseGraph.graphNodes[i].nodePose.x * scale, poseGraph.graphNodes[i].nodePose.y * scale);
                    robotItem->setRotation(poseGraph.graphNodes[i].nodePose.theta * 180 / M_PI);  // Convert radians to degrees
                    scene.addItem(robotItem);
                }
            }
        } else {
            while (timer.elapsed() < 20) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            }
        }
        timer.restart();  // Restart the timer for the next iteration
    }

    // Start the Qt event loop
    return a.exec();
}
