cmake_minimum_required(VERSION 3.0.2)
project(test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospack
  std_msgs
  geometry_msgs
  nodelet
  pluginlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
 LIBRARIES test_nodelet
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs rospack nodelet pluginlib
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_1 src/test_1.cpp)
target_link_libraries(test_1 ${catkin_LIBRARIES})

add_executable(test_2 src/test_2.cpp)
target_link_libraries(test_2 ${catkin_LIBRARIES})

add_executable(test_3 src/test_3.cpp)
target_link_libraries(test_3 ${catkin_LIBRARIES})

add_library(test_nodelet src/test_nodelet.cpp)
target_link_libraries(test_nodelet
  ${catkin_LIBRARIES}
)
install(TARGETS test_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)