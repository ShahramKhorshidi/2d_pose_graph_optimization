cmake_minimum_required(VERSION 3.10)
project(RobotVisualization)

# Find Qt
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Add source files
set(SOURCES
    main.cpp
)

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Copy the example data next to the executable
configure_file(robot_trajectory.log robot_trajectory.log COPYONLY)
configure_file(robot_image_red.png robot_image_red.png COPYONLY)
configure_file(robot_image_green.png robot_image_green.png COPYONLY)

# Link the Qt5 libraries
target_link_libraries(${PROJECT_NAME} 
    UtilLibrary
    ProgramLibrary
    Qt5::Core Qt5::Widgets)
