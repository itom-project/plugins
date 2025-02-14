cmake_minimum_required(VERSION 3.8)
project(itom_ros2_test)
# Set up the CMake policy CMP0148
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# Turn off development warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build a custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Exchange1DArray.srv"
  "srv/Exchange2DArray.srv"
  "srv/ExchangeNDArray.srv"
  "srv/ExchangeObject.srv"
  "srv/ExchangePointCloud.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
