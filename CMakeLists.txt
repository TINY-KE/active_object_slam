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

set(CMAKE_CXX_STANDARD 14) # ok

set(CMAKE_CXX_FLAGS -DCV__ENABLE_C_API_CTORS) #OPENCV VERISON
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")  #-march=native
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 ") #-march=native
#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -fno-inline")
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
  tf

  # plane
  pcl_ros
  pcl_conversions

  # yolo
  darknet_ros_msgs
)  #for ros

find_package(Sophus REQUIRED)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#     INCLUDE_DIRS include
#     LIBRARIES connect_gazebo
#     CATKIN_DEPENDS roscpp std_msgs test_pkg gazebo_msgs
)   #for ros

include_directories(
${catkin_INCLUDE_DIRS}   #for ros
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/
${PROJECT_SOURCE_DIR}/include
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories( "/usr/local/include/sophus" )
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

   # line
   src/line_detect/line_lbd_allclass.cpp
   src/line_detect/libs/binary_descriptor.cpp
   src/line_detect/libs/binary_descriptor_matcher.cpp
   src/line_detect/libs/LSDDetector.cpp
   src/line_detect/libs/lsd.cpp
   src/line_detect/libs/bitarray.hpp
   src/line_detect/libs/bitops.hpp
   src/line_detect/libs/draw.cpp
   src/line_detect/libs/precomp.hpp
   src/line_detect/libs/types.hpp
   # cube
   src/detect_3d_cuboid/box_proposal_detail.cpp
   src/detect_3d_cuboid/object_3d_util.cpp
   src/detect_3d_cuboid/matrix_utils.cpp

   # plane
   src/MapPlane.cc
#    ../src/g2oAddition/EdgePlane.cc
#    ../src/g2oAddition/VertexPlane.cc

   # NBV
   src/NbvGenerator.cpp
   # NBV MAM
   src/camera.cc

   # read_local_objects
#   src/read_local_objects.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
#   ${Sophus_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
   # /usr/lib/x86_64-linux-gnu/libboost_system.so
#   /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1
   ${catkin_LIBRARIES}  #for ros
   ${PCL_LIBRARIES}

   # line
   ${PROJECT_SOURCE_DIR}/Thirdparty/EDLines/EDLinesLib.a
   ${PROJECT_SOURCE_DIR}/Thirdparty/EDTest/EDLib.a
   ${PROJECT_SOURCE_DIR}/Thirdparty/Line3Dpp/build/libline3Dpp.so


)



# Node for RGB-D camera
add_executable(active_rgbd ros_src/init_test.cpp)
target_link_libraries(active_rgbd   ${PROJECT_NAME}_lib )

#add_executable(rviz_dae ros_src/rviz_dae.cpp)
#target_link_libraries(rviz_dae   ${PROJECT_NAME}_lib )

add_executable(eval eval/eval.cpp)
target_link_libraries(eval   ${PROJECT_NAME}_lib   )

add_executable(extract_gazebo_groudth eval/extract_gazebo_groudth.cpp)
target_link_libraries(extract_gazebo_groudth
   ${PROJECT_NAME}_lib
   ${GAZEBO_LIBRARIES}
)

add_executable(rviz_gazebo_object eval/rviz_gazebo_object.cpp)
target_link_libraries(rviz_gazebo_object
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${EIGEN3_LIBS}
)

add_executable(rviz_nbv eval/rviz_nbv.cpp src/Converter.cc)
target_link_libraries(rviz_nbv
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}  #for ros
)