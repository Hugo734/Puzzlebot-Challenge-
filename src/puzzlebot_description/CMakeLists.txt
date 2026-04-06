cmake_minimum_required(VERSION 3.8)
project(puzzlebot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes rviz worlds models config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/circular_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
