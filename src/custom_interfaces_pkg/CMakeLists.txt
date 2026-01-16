cmake_minimum_required(VERSION 3.8)
project(custom_interfaces_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dipendenze base
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generazione delle interfacce
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConeDetection.msg"
  "msg/ImageAcquisition.msg"
  "msg/VisualInformation.msg"
  "msg/PixelPoint.msg"
  "msg/Waypoint.msg"
  "srv/DetectCones.srv"
  "srv/ComputeWaypoint.srv"
  "srv/PointTransformer.srv"
  DEPENDENCIES sensor_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
