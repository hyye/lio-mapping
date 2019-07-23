cmake_minimum_required(VERSION 3.1)
project(lio)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lio
#  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_generation message_runtime nav_msgs pcl_conversions pcl_ros rosbag roscpp rospy sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lio.cpp
#

set(lio_EXT_LIBS
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
)

set(point_processor_EXT_LIBS
  ${catkin_LIBRARIES}
  ${lio_EXT_LIBS}
)

add_library(${PROJECT_NAME}-point-processor
  src/point_processor/PointMapping.cc
  src/point_processor/PointOdometry.cc
  src/point_processor/PointProcessor.cc
)
# target_link_libraries(${PROJECT_NAME}-point-processor
#   ${point_processor_EXT_LIBS}
# )

add_library(${PROJECT_NAME}-visualizer
  src/visualizer/Visualizer.cc
)
# target_link_libraries(${PROJECT_NAME}-visualizer
#   ${point_processor_EXT_LIBS}
# )

add_library(${PROJECT_NAME}-feature-manager
  src/feature_manager/FeatureManager.cc
)
# target_link_libraries(${PROJECT_NAME}-feature-manager
#   ${point_processor_EXT_LIBS}
# )

set(factor_EXT_LIBS
  ${PROJECT_NAME}-feature-manager
  ${catkin_LIBRARIES}
  ${lio_EXT_LIBS}
  ceres
)

add_library(${PROJECT_NAME}-factor
  src/factor/PointDistanceFactor.cc
  src/factor/PoseLocalParameterization.cc
  src/factor/GravityLocalParameterization.cc
  src/factor/PlaneProjectionFactor.cc
  src/factor/PlaneToPlaneFactor.cc
  src/factor/PivotPointPlaneFactor.cc
  src/factor/MarginalizationFactor.cc
  src/factor/PriorFactor.cc
)

add_dependencies(${PROJECT_NAME}-factor
  ${PROJECT_NAME}-feature-manager
)
 
# target_link_libraries(${PROJECT_NAME}-factor
#   ${PROJECT_NAME}-feature-manager
#   ${factor_EXT_LIBS}
# )

set(imu_processor_EXT_LIBS
  ${factor_EXT_LIBS}
)

add_library(${PROJECT_NAME}-imu-processor
  src/imu_processor/Estimator.cc
  src/imu_processor/ImuInitializer.cc
  src/imu_processor/MeasurementManager.cc
)

# TODO: separate these two libs
# target_link_libraries(${PROJECT_NAME}-imu-processor
#   ${imu_processor_EXT_LIBS}
#   ${PROJECT_NAME}-point-processor
#   ${PROJECT_NAME}-factor
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lio_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(${PROJECT_NAME}_processor_node src/processor_node.cc)
add_dependencies(${PROJECT_NAME}_processor_node ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}_processor_node
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_odometry_node src/odometry_node.cc)
add_dependencies(${PROJECT_NAME}_odometry_node ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}_odometry_node
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_mapping_node src/mapping_node.cc)
add_dependencies(${PROJECT_NAME}_mapping_node ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}_mapping_node
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_estimator_node src/estimator_node.cc)
add_dependencies(${PROJECT_NAME}_estimator_node
  ${PROJECT_NAME}-imu-processor
  ${PROJECT_NAME}-factor
  ${PROJECT_NAME}-feature-manager
  ${PROJECT_NAME}-point-processor
  ${PROJECT_NAME}-visualizer
)
target_link_libraries(${PROJECT_NAME}_estimator_node
  ${PROJECT_NAME}-imu-processor
  ${PROJECT_NAME}-factor
  ${PROJECT_NAME}-feature-manager
  ${PROJECT_NAME}-point-processor
  ${PROJECT_NAME}-visualizer
  ${imu_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_input_filters_node src/input_filters_node.cc)
target_link_libraries(${PROJECT_NAME}_input_filters_node
  ${imu_processor_EXT_LIBS}
)

#############
##   MAP   ##
#############
add_library(${PROJECT_NAME}-map-builder src/map_builder/MapBuilder.cc)
# add_dependencies(${PROJECT_NAME}-map-builder
#   ${PROJECT_NAME}-point-processor
# )
# target_link_libraries(${PROJECT_NAME}-map-builder
#   ${PROJECT_NAME}-point-processor
# )

add_executable(${PROJECT_NAME}_map_builder_node src/map_builder_node.cc)
add_dependencies(${PROJECT_NAME}_map_builder_node
  ${PROJECT_NAME}-map-builder
)
target_link_libraries(${PROJECT_NAME}_map_builder_node
  ${PROJECT_NAME}-map-builder
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_save_bag_to_pcd src/save_bag_to_pcd.cc)
add_dependencies(${PROJECT_NAME}_save_bag_to_pcd
  ${PROJECT_NAME}-map-builder
)
target_link_libraries(${PROJECT_NAME}_save_bag_to_pcd
  ${point_processor_EXT_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lio.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# if (test)

catkin_add_gtest(${PROJECT_NAME}-test-point-processor test/test_point_processor/test_point_processor.cc)
add_dependencies(${PROJECT_NAME}-test-point-processor ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}-test-point-processor
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

catkin_add_gtest(${PROJECT_NAME}-test-point-odometry test/test_point_processor/test_point_odometry.cc)
add_dependencies(${PROJECT_NAME}-test-point-odometry ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}-test-point-odometry
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

catkin_add_gtest(${PROJECT_NAME}-test-point-mapping test/test_point_processor/test_point_mapping.cc)
add_dependencies(${PROJECT_NAME}-test-point-mapping ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}-test-point-mapping
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

catkin_add_gtest(${PROJECT_NAME}-test-rotations test/test_rotations.cc)
add_dependencies(${PROJECT_NAME}-test-rotations ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}-test-rotations
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

catkin_add_gtest(${PROJECT_NAME}-test-measurement-manager test/test_imu_processor/test_measurement_manager.cc)
add_dependencies(${PROJECT_NAME}-test-measurement-manager
  ${PROJECT_NAME}-imu-processor
  ${PROJECT_NAME}-feature-manager
  ${PROJECT_NAME}-visualizer
)
target_link_libraries(${PROJECT_NAME}-test-measurement-manager
  ${PROJECT_NAME}-imu-processor
  ${PROJECT_NAME}-feature-manager
  ${PROJECT_NAME}-visualizer
  ${imu_processor_EXT_LIBS}
)

catkin_add_gtest(${PROJECT_NAME}-test-circular-buffer test/test_imu_processor/test_circular_buffer.cc)
add_dependencies(${PROJECT_NAME}-test-circular-buffer ${PROJECT_NAME}-imu-processor)
target_link_libraries(${PROJECT_NAME}-test-circular-buffer
  ${PROJECT_NAME}-imu-processor
  ${imu_processor_EXT_LIBS}
)


catkin_add_gtest(${PROJECT_NAME}-test-visualizer test/test_visualizaer.cc)
add_dependencies(${PROJECT_NAME}-test-visualizer ${PROJECT_NAME}-visualizer)
target_link_libraries(${PROJECT_NAME}-test-visualizer
  ${PROJECT_NAME}-visualizer
)

catkin_add_gtest(${PROJECT_NAME}-test-imu-factor test/test_imu_processor/test_imu_factor.cc)
add_dependencies(${PROJECT_NAME}-test-imu-factor
#  ${PROJECT_NAME}-imu-processor
#  ${PROJECT_NAME}-feature-manager
#  ${PROJECT_NAME}-visualizer
${PROJECT_NAME}-factor
)
target_link_libraries(${PROJECT_NAME}-test-imu-factor
#  ${PROJECT_NAME}-imu-processor
#  ${imu_processor_EXT_LIBS}
#  ${PROJECT_NAME}-feature-manager
#  ${PROJECT_NAME}-visualizer
${PROJECT_NAME}-factor
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
