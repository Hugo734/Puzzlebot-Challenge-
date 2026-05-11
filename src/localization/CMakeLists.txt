cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(DEPS
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- ekf_localization ---
add_executable(ekf_localization src/ekf_localization_node.cpp)
target_include_directories(ekf_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ekf_localization ${DEPS})
target_link_libraries(ekf_localization Eigen3::Eigen)

# --- kinematic_simulator ---
add_executable(kinematic_simulator src/kinematic_simulator_node.cpp)
ament_target_dependencies(kinematic_simulator rclcpp std_msgs geometry_msgs)

# --- velocity_bridge ---
add_executable(velocity_bridge src/velocity_bridge_node.cpp)
ament_target_dependencies(velocity_bridge rclcpp std_msgs)

# --- icp_node ---
add_executable(icp_node src/icp_node.cpp)
target_include_directories(icp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(icp_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(icp_node Eigen3::Eigen)
target_compile_options(icp_node PRIVATE -O3)

# Install executables
install(TARGETS
  ekf_localization
  kinematic_simulator
  velocity_bridge
  icp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers, config, launch
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
