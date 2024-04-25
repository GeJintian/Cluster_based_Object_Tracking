cmake_minimum_required(VERSION 3.5)
project(detection)
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DTARGET_ROS2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(a2rl_bs_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
ENDIF()

add_library(${PROJECT_NAME} SHARED)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    "rclcpp"
    "std_msgs"
    "a2rl_bs_msgs"
    "tf2" 
    "tf2_ros" 
    "tf2_geometry_msgs" 
    "pcl_conversions" 
    "PCL" 
    "sensor_msgs" 
    "geometry_msgs"
    "OpenCV"
)

target_sources(${PROJECT_NAME}
  PRIVATE
    #"src/detection.cpp"
    "src/frenet.cpp"
    #"src/voxel_grid_omp.cpp"
    "src/tracker.cpp"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)


set(INCLUDE_DIRS include 
${ament_cmake_INCLUDE_DIRS} 
${rclcpp_INCLUDE_DIRS}
${pcl_ros_INCLUDE_DIRS} 
${pcl_conversions_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${rclcpp_components_INCLUDE_DIRS}
)
# Include
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
${ament_cmake_LIBRARY_DIRS} 
${rclcpp_LIBRARY_DIRS}
${pcl_ros_LIBRARY_DIRS} 
${pcl_conversions_LIBRARY_DIRS}
${std_msgs_LIBRARY_DIRS}
${sensor_msgs_LIBRARY_DIRS}
${rclcpp_components_LIBRARY_DIRS})
#  Linking
link_directories(${LIBRARY_DIRS})

set(LIBS 
${ament_cmake_LIBRARIES} 
${rclcpp_LIBRARIES} 
${pcl_ros_LIBRARIES} 
${pcl_conversions_LIBRARIES} 
${std_msgs_LIBRARIES} 
${sensor_msgs_LIBRARIES}
${rclcpp_components_LIBRARIES})


add_executable(detection_node src/detection_node.cpp)
target_compile_options(detection_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(detection_node ${LIBS} ${PCL_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(merge_node src/merge_node.cpp)
target_link_libraries(merge_node ${LIBS} ${PCL_LIBRARIES} ${PROJECT_NAME})

install(TARGETS
${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

# ==== Install ====
install(TARGETS detection_node merge_node
ARCHIVE
LIBRARY
RUNTIME
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_package()
