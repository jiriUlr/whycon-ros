cmake_minimum_required(VERSION 3.28.3)

project(whycode_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
)

set(srv_files
  "srv/SelectMarker.srv"
  "srv/SetCalibMethod.srv"
  "srv/SetCalibPath.srv"
  "srv/SetCoords.srv"
  "srv/SetDrawing.srv"
  "srv/GetGuiSettings.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()