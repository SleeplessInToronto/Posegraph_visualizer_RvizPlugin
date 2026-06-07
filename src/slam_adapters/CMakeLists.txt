cmake_minimum_required(VERSION 3.8)
project(slam_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common)

add_executable(lidar_ring_preprocessor src/lidar_ring_preprocessor.cpp)
target_include_directories(lidar_ring_preprocessor PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_ring_preprocessor ${PCL_LIBRARIES})
ament_target_dependencies(lidar_ring_preprocessor rclcpp sensor_msgs pcl_conversions)

install(TARGETS lidar_ring_preprocessor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
