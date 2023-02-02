cmake_minimum_required(VERSION 3.0.2)
project(spawnLego_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")


find_package(catkin REQUIRED rospy message_generation std_msgs geometry_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
   FILES
   legoDetection.msg
   legoGroup.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

catkin_package(
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)




#add_dependencies(spawnLego_pkg ${spawnLego_pkg_EXPORTED_TARGETS})
#  install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
