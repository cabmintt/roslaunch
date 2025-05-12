cmake_minimum_required(VERSION 3.0)
project(roslaunch)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospack
  roslib
  nodelet
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PUGIXML REQUIRED pugixml)

find_package(Boost REQUIRED COMPONENTS)

find_package(yaml-cpp REQUIRED)
# find_package(xmlrpccpp REQUIRED)

find_library(XMLRPCPP_LIB
    NAMES xmlrpcpp
    PATHS /opt/ros/noetic/lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roslaunch_lib
  CATKIN_DEPENDS roscpp rospack roslib nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${XMLRPCPP_LIB}
  /opt/ros/noetic/include

)

add_library(roslaunch_lib
  src/roslaunch.cpp
)


target_include_directories(roslaunch_lib INTERFACE
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(main src/main.cpp)

target_link_libraries(main
  roslaunch_lib
  ${catkin_LIBRARIES}
  ${PUGIXML_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${XMLRPCPP_LIB}
)

