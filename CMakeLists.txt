cmake_minimum_required(VERSION 3.8)
project(vsp_to_qsp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(VSP_TO_QSP_PX4_MSGS_PREFIX "" CACHE PATH
  "Optional install prefix for px4_msgs (prefix containing share/px4_msgs/cmake)")

find_package(px4_msgs QUIET)
if(NOT px4_msgs_FOUND)
  set(_px4_msgs_candidate_prefixes "")
  if(NOT "${VSP_TO_QSP_PX4_MSGS_PREFIX}" STREQUAL "")
    list(APPEND _px4_msgs_candidate_prefixes "${VSP_TO_QSP_PX4_MSGS_PREFIX}")
  endif()
  if(DEFINED ENV{PX4_MSGS_PREFIX} AND NOT "$ENV{PX4_MSGS_PREFIX}" STREQUAL "")
    list(APPEND _px4_msgs_candidate_prefixes "$ENV{PX4_MSGS_PREFIX}")
  endif()
  get_filename_component(_vsp_to_qsp_src_dir "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
  get_filename_component(_ros2_ws_src_dir "${_vsp_to_qsp_src_dir}" DIRECTORY)
  get_filename_component(_ros2_ws_dir "${_ros2_ws_src_dir}" DIRECTORY)
  get_filename_component(_ros2_root_dir "${_ros2_ws_dir}" DIRECTORY)
  list(APPEND _px4_msgs_candidate_prefixes "${_ros2_root_dir}/px4_ros/install")
  if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    list(APPEND _px4_msgs_candidate_prefixes "$ENV{HOME}/Documents/A4VAI-SITL/ROS2/px4_ros/install")
  endif()
  list(REMOVE_DUPLICATES _px4_msgs_candidate_prefixes)

  set(_px4_msgs_checked_prefixes "")
  set(_px4_msgs_discovered_prefix "")
  set(_px4_msgs_discovered_layout "")
  foreach(_px4_msgs_prefix IN LISTS _px4_msgs_candidate_prefixes)
    list(APPEND _px4_msgs_checked_prefixes "${_px4_msgs_prefix}")
    if(EXISTS "${_px4_msgs_prefix}/share/px4_msgs/cmake/px4_msgsConfig.cmake")
      set(_px4_msgs_discovered_prefix "${_px4_msgs_prefix}")
      set(_px4_msgs_discovered_layout "merged")
      break()
    endif()
    if(EXISTS "${_px4_msgs_prefix}/px4_msgs/share/px4_msgs/cmake/px4_msgsConfig.cmake")
      set(_px4_msgs_discovered_prefix "${_px4_msgs_prefix}/px4_msgs")
      set(_px4_msgs_discovered_layout "isolated")
      break()
    endif()
  endforeach()

  if(NOT "${_px4_msgs_discovered_prefix}" STREQUAL "")
    set(px4_msgs_DIR "${_px4_msgs_discovered_prefix}/share/px4_msgs/cmake")
    set(CMAKE_PREFIX_PATH "${_px4_msgs_discovered_prefix};${CMAKE_PREFIX_PATH}")
    message(STATUS
      "vsp_to_qsp: px4_msgs fallback prefix (${_px4_msgs_discovered_layout}): "
      "${_px4_msgs_discovered_prefix}")
    find_package(px4_msgs REQUIRED CONFIG)
  else()
    set(_px4_msgs_checked_msg "")
    foreach(_checked_prefix IN LISTS _px4_msgs_checked_prefixes)
      string(APPEND _px4_msgs_checked_msg "\n  - ${_checked_prefix}")
    endforeach()
    if("${_px4_msgs_checked_msg}" STREQUAL "")
      set(_px4_msgs_checked_msg "\n  - (no candidate prefixes)")
    endif()

    message(FATAL_ERROR
      "vsp_to_qsp: px4_msgs not found.\n"
      "Checked prefixes:${_px4_msgs_checked_msg}\n"
      "Build px4_ros first (recommended): ./scripts/run.sh ros2 build px4_ros\n"
      "Or provide prefix via:\n"
      "  -DVSP_TO_QSP_PX4_MSGS_PREFIX=/path/to/install\n"
      "  PX4_MSGS_PREFIX=/path/to/install"
    )
  endif()
else()
  message(STATUS "vsp_to_qsp: px4_msgs found via default search path: ${px4_msgs_DIR}")
endif()

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
