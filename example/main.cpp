#include <iostream>
#include <fstream>
#include <filesystem>
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QElapsedTimer>
#include <QLabel>
#include <QGridLayout>
#include <QColor>
#include <QFont>
#include <QWidget>
#include "Pose2D.h"
#include "PoseGraph.h"

struct Trajectory {
    float id, x, y, angle;
};

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    // Animation timing in milliseconds. Increase the delay to make the robot move slower.
    const int trajectoryDelayMs = 50;
    const int loopClosingPauseMs = 4000;

    auto waitForAnimation = [](int delayMs) {
        QElapsedTimer timer;
        timer.start();
        while (timer.elapsed() < delayMs) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    };

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

    // Put the view and the legend in the same layout so the legend stays visible
    QWidget window;
    QGridLayout* windowLayout = new QGridLayout(&window);
    windowLayout->setContentsMargins(0, 0, 0, 0);

    // Create a QGraphicsView to visualize the scene
    QGraphicsView* view = new QGraphicsView(&scene, &window);
    view->scale(-1, 1);
    windowLayout->addWidget(view, 0, 0);

    QLabel* legend = new QLabel();
    legend->setText(
        "<b>Robot trajectory</b><br>"
        "<span style='color:#d7191c;'>&#9679;</span> Red: before loop closing<br>"
        "<span style='color:#1a9641;'>&#9679;</span> Green: after loop closing");
    legend->setStyleSheet(
        "QLabel { background-color: rgba(255, 255, 255, 220); "
        "border: 2px solid gray; padding: 12px; font-size: 24px; }");
    legend->adjustSize();
    legend->setAttribute(Qt::WA_TransparentForMouseEvents);
    windowLayout->addWidget(legend, 0, 0, Qt::AlignTop | Qt::AlignLeft);

    window.showMaximized();
    legend->raise();

    // Instantiating a pose graph object
    PoseGraph poseGraph = PoseGraph();

    float scale = 22.0;

    // Read the robot trajectory
    for (const Trajectory& robotTraj : trajectory) {
        // Adding consecutive nodes to the pose graph with odometry constraints
        Pose2D odomPose = Pose2D(robotTraj.x, robotTraj.y, robotTraj.angle);
        poseGraph.addOdomNode(odomPose);
        if (PoseGraph::getNodesCount() == 1){
            std::cout << "--- Add nodes from odometry to the graph.---" << std::endl;
        }

        // Draw the trajectory before and after loop closing with different colors
        QPixmap robotImage = PoseGraph::getNodesCount() < 263 ? robotImageRed : robotImageGreen;
        QGraphicsPixmapItem* robotItem = new QGraphicsPixmapItem(robotImage.scaledToWidth(20));
        robotItem->setPos(odomPose.x * scale, odomPose.y * scale);
        robotItem->setRotation(odomPose.theta * 180 / M_PI);  // Convert radians to degrees
        scene.addItem(robotItem);

        // Loop closing constraint
        if (PoseGraph::getNodesCount() == 262){
            std::cout << "--- Loop closing between nodeId=9 and nodeId=261 ---" << std::endl;
            std::cout << "--- Loop closing constraint: [x, y, theta] = [-0.905603, -0.340513, 0.185957] ---" << std::endl;

            // Show the loop closing event and pause before correcting the trajectory
            QGraphicsTextItem* loopClosingText = scene.addText(
                "Loop closing happens here (nodes 9 and 261)");
            loopClosingText->setDefaultTextColor(QColor(25, 70, 150));
            loopClosingText->setFont(QFont("Sans Serif", 16, QFont::Bold));
            loopClosingText->setFlag(QGraphicsItem::ItemIgnoresTransformations);
            loopClosingText->setPos(odomPose.x * scale + 20, odomPose.y * scale - 20);
            waitForAnimation(loopClosingPauseMs);

            // Relative Transformation between two poses from loop closing detection.
            uint nodeIdi = 9;
            uint nodeIdj = 261;
            Pose2D observation = Pose2D(-0.905603, -0.340513, 0.185957);
            poseGraph.connectLoopClosingNodes(nodeIdi, nodeIdj, observation);

            // We only optimize the graph between two loop closing nodes.
            GraphNode& from = poseGraph.graphNodes[nodeIdi];
            GraphNode& to = poseGraph.graphNodes[nodeIdj];
            poseGraph.optimizeGraph(from, to);

            // Display the optimized trajectory once in green
            for (uint i = nodeIdi; i <= nodeIdj; i++) {
                QGraphicsPixmapItem* optimizedRobotItem =
                    new QGraphicsPixmapItem(robotImageGreen.scaledToWidth(20));
                optimizedRobotItem->setPos(
                    poseGraph.graphNodes[i].nodePose.x * scale,
                    poseGraph.graphNodes[i].nodePose.y * scale);
                optimizedRobotItem->setRotation(
                    poseGraph.graphNodes[i].nodePose.theta * 180 / M_PI);
                scene.addItem(optimizedRobotItem);
            }
        }

        waitForAnimation(trajectoryDelayMs);
    }

    // Start the Qt event loop
    return a.exec();
}
