cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)

project(detection
  DESCRIPTION "detection module"
  VERSION 0.0.1
  LANGUAGES CXX 
)

# Enforce compiler flags for certain compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(a2rl_bs_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(Ceres REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PCL_INCLUDE_DIRS}
${pcl_conversions_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${rclcpp_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
# ${CERES_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(radar_fusion_node src/radar_fusion_node.cpp)
ament_target_dependencies(radar_fusion_node rclcpp rclcpp_components tf2 tf2_ros tf2_geometry_msgs pcl_conversions PCL sensor_msgs geometry_msgs)
target_link_libraries(radar_fusion_node ${PCL_LIBRARIES})

add_executable(detection_node src/detection_node.cpp src/detection.cpp src/fusion_function.cpp src/tracker.cpp)
ament_target_dependencies(detection_node rclcpp rclcpp_components tf2 tf2_ros tf2_geometry_msgs pcl_conversions PCL sensor_msgs geometry_msgs OpenCV a2rl_bs_msgs)
target_link_libraries(detection_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(radar_moving_node src/radar_moving_filter_node.cpp.cpp src/fusion_function.cpp)
ament_target_dependencies(radar_moving_node rclcpp rclcpp_components tf2 tf2_ros tf2_geometry_msgs pcl_conversions PCL sensor_msgs geometry_msgs a2rl_bs_msgs)
target_link_libraries(radar_moving_node ${PCL_LIBRARIES})

install(TARGETS radar_fusion_node detection_node radar_moving_node
  ARCHIVE
  LIBRARY
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
