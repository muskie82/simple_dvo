cmake_minimum_required(VERSION 2.8.3)
project(simple_dvo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  std_msgs
  sensor_msgs
  image_geometry
  cv_bridge
  pcl_conversions
  pcl_ros
  cmake_modules
)

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR}  ${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(ThirdParty/Sophus)

find_package(PCL 1.7 REQUIRED COMPONENTS visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -O3 -ftree-vectorize -march=native")

catkin_package(
    DEPENDS sophus
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp include/image_alignment.h src/image_alignment.cpp include/util.h src/util.cpp include/dvo_class.h)


target_link_libraries(main
        ${catkin_LIBRARIES}
)
