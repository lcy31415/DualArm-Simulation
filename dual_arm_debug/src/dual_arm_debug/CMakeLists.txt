cmake_minimum_required(VERSION 3.8)
project(dual_arm_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/dual_arm_joint_publisher_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
