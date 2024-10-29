cmake_minimum_required(VERSION 3.8)
project(jazzy_camera_issues)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cloud_sub src/cloud_sub.cpp)
target_include_directories(cloud_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(cloud_sub
  rclcpp
  sensor_msgs
)
target_compile_features(cloud_sub PUBLIC c_std_99 cxx_std_17)

add_executable(image_sub src/image_sub.cpp)
target_include_directories(image_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(image_sub
  rclcpp
  sensor_msgs
)
target_compile_features(image_sub PUBLIC c_std_99 cxx_std_17)

install(TARGETS cloud_sub image_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
