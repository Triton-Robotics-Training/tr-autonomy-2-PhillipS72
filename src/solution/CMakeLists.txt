cmake_minimum_required(VERSION 3.5)
project(solution)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(image_angle_controller image_angle_controller.cpp)

ament_target_dependencies(image_angle_controller
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  image_angle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
