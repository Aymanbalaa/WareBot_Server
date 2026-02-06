cmake_minimum_required(VERSION 3.20)
project(vizanti_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbridge_suite REQUIRED)

# Service definitions (formerly vizanti_msgs)
set(srv_files
  "srv/GetNodeParameters.srv"
  "srv/SetNodeParameter.srv"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/RecordRosbag.srv"
  "srv/ManageNode.srv"
  "srv/ListPackages.srv"
  "srv/ListExecutables.srv"
  "srv/ListLifecycles.srv"
  "srv/GetLastPose.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
)

# C++ TF consolidator node (formerly vizanti_cpp)
add_executable(tf_consolidator src/tf_consolidator.cpp)
ament_target_dependencies(tf_consolidator rclcpp std_msgs tf2_msgs)

install(TARGETS
  tf_consolidator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  public
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/__init__.py
  scripts/server.py
  scripts/service_handler.py
  scripts/rqt_reconfigure_param_api.py
  scripts/pose_persistence.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
