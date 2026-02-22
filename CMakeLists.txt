cmake_minimum_required(VERSION 3.8)
project(vsp_to_qsp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(velocity_to_attitude_node
  src/velocity_to_attitude_node.cpp
)
target_compile_features(velocity_to_attitude_node PRIVATE cxx_std_17)
target_include_directories(velocity_to_attitude_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(velocity_to_attitude_node
  rclcpp
  px4_msgs
)

install(TARGETS velocity_to_attitude_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
