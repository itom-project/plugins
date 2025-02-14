cmake_minimum_required(VERSION 3.8)
project(image_exchange_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(itom_ros2_test REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 添加客户端节点
add_executable(image_exchange_client src/image_exchange_client.cpp)
ament_target_dependencies(image_exchange_client rclcpp sensor_msgs itom_ros2_test OpenCV cv_bridge)




# 安装目标文件
install(TARGETS
  image_exchange_client
  DESTINATION lib/${PROJECT_NAME}
)

# 安装数据文件夹
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_package()
