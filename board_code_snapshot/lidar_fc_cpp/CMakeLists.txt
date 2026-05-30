cmake_minimum_required(VERSION 3.8)
project(lidar_fc_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc objdetect)

set(COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  livox_ros_driver2
)

add_executable(mid360_xy_cpp src/mid360_xy_node.cpp)
ament_target_dependencies(mid360_xy_cpp ${COMMON_DEPENDENCIES})

add_executable(relative_pose_cpp src/relative_pose_node.cpp)
ament_target_dependencies(relative_pose_cpp ${COMMON_DEPENDENCIES})

add_executable(fc_bridge_cpp src/fc_bridge_node.cpp)
ament_target_dependencies(fc_bridge_cpp ${COMMON_DEPENDENCIES})

add_executable(lio_debug_cpp src/lio_debug_node.cpp)
ament_target_dependencies(lio_debug_cpp ${COMMON_DEPENDENCIES})

add_executable(debug_recorder_cpp src/debug_recorder_node.cpp)
ament_target_dependencies(debug_recorder_cpp ${COMMON_DEPENDENCIES})

add_executable(qr_detector_cpp src/qr_detector_node.cpp)
ament_target_dependencies(qr_detector_cpp
  rclcpp
  std_msgs
  sensor_msgs
)
target_include_directories(qr_detector_cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(qr_detector_cpp ${OpenCV_LIBS})

install(TARGETS
  mid360_xy_cpp
  relative_pose_cpp
  fc_bridge_cpp
  lio_debug_cpp
  debug_recorder_cpp
  qr_detector_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
