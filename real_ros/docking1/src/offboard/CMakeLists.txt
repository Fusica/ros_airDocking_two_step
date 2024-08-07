cmake_minimum_required(VERSION 3.0.2)
project(offboard)


find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS mavros roscpp std_msgs
#  DEPENDS system_lib
)
find_package(OpenCV REQUIRED)
find_package(Eigen3)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}_node src/offboard_node.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )