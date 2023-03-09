cmake_minimum_required(VERSION 3.8)
project(slam_demo)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies: ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# 3-rd party packages
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(libpointmatcher 1.1.0 REQUIRED)

list(APPEND G2O_LIBS
  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
  g2o_core g2o_solver_csparse g2o_solver_structure_only
  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_solver_dense g2o_stuff
  g2o_types_sclam2d g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include/
  src/
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${libpointmatcher_INCLUDE_DIRS}
)

# add the source files
set(SOURCES
  src/odometer.cpp
  src/slam.cpp
  src/pose_graph.cpp
  src/scan_context.cpp
  src/visualization.cpp
  src/helper.cpp
)

add_executable(slam_node slam_node.cpp ${SOURCES})
ament_target_dependencies(
  slam_node
  rclcpp
  std_msgs
  laser_geometry
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  pcl_msgs
  tf2_ros
  tf2_eigen
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(
  slam_node
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${G2O_LIBS}
  ${libpointmatcher_LIBRARIES}
)

add_executable(sensor_offset_node src/sensor_offset_node.cpp)
ament_target_dependencies(
  sensor_offset_node
  rclcpp
  geometry_msgs
  tf2_ros
)

add_executable(opt_topic_pub_node code_snippet/opt_topic_pub.cpp)
ament_target_dependencies(
  opt_topic_pub_node
  rclcpp
  std_msgs
  tf2_ros
)

install(
  TARGETS
  slam_node
  sensor_offset_node
  opt_topic_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
  bag
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
