cmake_minimum_required(VERSION 3.8)
project(factor_graph_rviz_plugin)

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(factor_graph_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(${PROJECT_NAME} SHARED
  src/factor_graph_display.cpp
  include/factor_graph_rviz_plugin/factor_graph_display.hpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  factor_graph_msgs
  pluginlib
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rviz_common
  rviz_rendering
  factor_graph_msgs
  pluginlib
)

ament_package()
