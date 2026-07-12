cmake_minimum_required(VERSION 3.8)
project(spot_micro_motion_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(i2c_pwm_board_msgs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=maybe-uninitialized")

add_subdirectory(libs/spot_micro_kinematics_cpp)

include_directories(
  include
  include/spot_micro_motion_cmd
  src/smfsm
  src/rate_limited_first_order_filter
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(smfsm
  src/smfsm/spot_micro_state.cpp
  src/smfsm/spot_micro_idle.cpp
  src/smfsm/spot_micro_stand.cpp
  src/smfsm/spot_micro_transition_stand.cpp
  src/smfsm/spot_micro_transition_idle.cpp
  src/smfsm/spot_micro_walk.cpp
)

target_link_libraries(smfsm
  spot_micro_kinematics
)

ament_target_dependencies(smfsm
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  i2c_pwm_board_msgs
)

add_executable(${PROJECT_NAME}_node
  src/spot_micro_motion_cmd.cpp
  src/spot_micro_motion_cmd_node.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  smfsm
  spot_micro_kinematics
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  i2c_pwm_board_msgs
)

install(TARGETS
  ${PROJECT_NAME}_node
  smfsm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()