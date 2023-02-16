cmake_minimum_required(VERSION 2.8.3)
project(publisher_pkg)

# add_dependencies(spawnLego_pkg ${catkin_EXPORTED_TARGETS})

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs control_msgs spawnLego_pkg ros_impedance_controller)
find_package(Eigen3 3.3 REQUIRED)


## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include/publisher_pkg/
  LIBRARIES publisher_pkg
)


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_executable(publisher_node src/publisher_node.cpp src/motion_planning.cpp src/kinematics.cpp)

#add_library(publisher_pkg src/kinematics.cpp src/motion_planning.cpp src/publisher_node.cpp)
target_link_libraries(publisher_node ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
#target_link_libraries(publisher_node src/publisher_node.cpp)

install(TARGETS publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
 DIRECTORY include/publisher_pkg/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
