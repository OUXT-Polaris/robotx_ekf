cmake_minimum_required(VERSION 3.5)
project(robotx_ekf)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_auto_find_build_dependencies()

ament_auto_add_executable(robotx_ekf_node
  src/ekf_node.cpp)

ament_auto_add_library(ekf_component SHARED
 src/ekf_component.cpp)

rclcpp_components_register_nodes(ekf_component "robotx_ekf::EKFComponent")

target_link_libraries(robotx_ekf_node ekf_component Eigen3::Eigen)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
