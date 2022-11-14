cmake_minimum_required(VERSION 2.4.6)
project(active_eao_new)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()
#SET(ROS_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()
SET(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_STANDARD 17) # ok

set(CMAKE_CXX_FLAGS -DCV__ENABLE_C_API_CTORS) #OPENCV VERISON
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


set(OpenCV_DIR "/home/zhjd/thirdparty/opencv3.4.10/opencv-3.4.10/build/")
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


# find_package(Boost  REQUIRED COMPONENTS filesystem thread date_time iostreams serialization chrono system)
find_package(Boost REQUIRED COMPONENTS filesystem thread date_time iostreams serialization chrono system)
set(PCL_DIR "/home/zhjd/thirdparty/pcl-1.8.0/pcl-pcl-1.8.0/build/")
find_package( PCL 1.8 REQUIRED )

set(cv_bridge_DIR "/home/zhjd/thirdparty/vision_opencv-indigo/cv_bridge/cmake")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
  std_msgs

  # yolo
  darknet_ros_msgs
)  #for ros

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)   #for ros

include_directories(
${catkin_INCLUDE_DIRS}   #for ros
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/
${PROJECT_SOURCE_DIR}/include
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

# add_definitions( ${PCL_DEFINITIONS} )
# link_directories( ${PCL_LIBRARY_DIRS} )


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME}_lib SHARED

   src/System.cc
   src/Tracking.cc
   src/LocalMapping.cc
   src/LoopClosing.cc
   src/ORBextractor.cc
   src/ORBmatcher.cc
   src/FrameDrawer.cc
   src/Converter.cc
   src/MapPoint.cc
   src/KeyFrame.cc
   src/Map.cc
   src/MapDrawer.cc
   src/Optimizer.cc
   src/PnPsolver.cc
   src/Frame.cc
   src/KeyFrameDatabase.cc
   src/Sim3Solver.cc
   src/Initializer.cc
   src/Viewer.cc
   src/Object.cpp
   src/MapPublisher.cc

)

target_link_libraries(${PROJECT_NAME}_lib
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
   # /usr/lib/x86_64-linux-gnu/libboost_system.so
   /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1
   ${catkin_LIBRARIES}  #for ros
   ${PCL_LIBRARIES}

)



# Node for RGB-D camera
add_executable(active_rgbd ros_src/init_test.cpp)
target_link_libraries(active_rgbd   ${PROJECT_NAME}_lib )

