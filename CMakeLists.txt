cmake_minimum_required(VERSION 3.0.2)
project(semantic_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# libyaml-cpp (required)
find_package (yaml-cpp)
if (yaml-cpp_FOUND)
  message("YAML Libs: ${YAML_CPP_LIBRARIES}")
  message("YAML Headers: ${YAML_CPP_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "yaml-cpp NOT FOUND\n")
endif (yaml-cpp_FOUND)

# get boost (required)
find_package(Boost COMPONENTS program_options filesystem)
if(Boost_FOUND)
  message("Boost Libs: ${Boost_LIBRARIES}")
  message("Boost Headers: ${Boost_INCLUDE_DIRS}\n")
else()
  message(FATAL_ERROR "Boost NOT FOUND\n")
endif()

####################################
## TensorRT specific configuration ##
####################################
# this package libs and includes for TensorRT

# set(CMAKE_PREFIX_PATH "/home/lenovo/TensorRT-5.1.5.0/lib")
set(CMAKE_PREFIX_PATH "/home/lenovo/TensorRT-5.1.5.0")

option(TENSORRT_FOUND "TensorRT installed on system?" OFF)

# first try to find cuda and tensorrt-related stuff (nvinfer, onnx, etc)
find_package(TensorRT PATHS cmake)
if (TENSORRT_FOUND)
  message("TensorRT SUCCESS!\n")
  set(TENSORRT_FOUND ON)
else()
  message("TensorRT NOT FOUND\n")
  set(TENSORRT_FOUND OFF)
endif (TENSORRT_FOUND)

# complain if no backend is installed
if(NOT TENSORRT_FOUND)
  message(FATAL_ERROR
  "TensorRT must be installed
  TENSORRT_FOUND ${TENSORRT_FOUND}\n")
endif()


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    semantic_external
    semantic_lib
  CATKIN_DEPENDS 
    cv_bridge geometry_msgs image_transport nav_msgs roscpp rospy sensor_msgs std_msgs tf
  DEPENDS
    YAML_CPP
    PCL
  CFG_EXTRAS
    tensorrt-config.cmake
)

###########
## Build ##
###########

#########################
## Install cmake finds ##
#########################
install(DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

###########################
## Install header config ##
###########################
include_directories(
  ros
  include
  ${rangenet_external_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${TENSORRT_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  "/home/lenovo/TensorRT-5.1.5.0/include"
)


# turn config file into flags (sets TENSORRT_FOUND and TORCH_FOUND)
configure_file(include/external.hpp.in ${PROJECT_SOURCE_DIR}/include/external.hpp @ONLY)

# Mark cpp header files for installation
install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# make wrapper lib
add_library(semantic_external src/external.cpp)

## Declare a cpp library
# define the network library (compiling tf and trt conditionally)
set(semantic_lib_SOURCES src/net.cpp src/selector.cpp)

if(TENSORRT_FOUND)
  message("Building TensorRT")
  list(APPEND semantic_lib_SOURCES src/netTensorRT.cpp)
endif()

# make the library and link stuff to it
add_library(semantic_lib ${semantic_lib_SOURCES})
target_link_libraries(semantic_lib
    semantic_external
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${TENSORRT_LIBRARIES}
    ${Eigen3_LIBS}
    )


###################
## build ##
###################

add_executable(aloam_semantic_node ros/aloam_pose.cpp)
target_link_libraries(aloam_semantic_node  semantic_lib)