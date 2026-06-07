cmake_minimum_required(VERSION 3.8)
project(factor_graph_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FactorGraphNode.msg"
  "msg/FactorGraphEdge.msg"
  "msg/FactorGraph.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)

ament_package()
