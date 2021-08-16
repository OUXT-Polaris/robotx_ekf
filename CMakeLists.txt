cmake_minimum_required(VERSION 3.5)
project(robotx_ekf)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(ekf_component SHARED src/ekf_component.cpp)

target_compile_definitions(ekf_component PRIVATE "ROBOTX_EKF__EKF_COMPONENT_BUILDING_DLL")
ament_target_dependencies(ekf_component rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs)
rclcpp_components_register_nodes(ekf_component "robotx_ekf::EKFComponent")

add_executable(ekf_node src/ekf_node.cpp)

target_link_libraries(ekf_node ekf_component Eigen3::Eigen)

ament_target_dependencies(ekf_node rclcpp rclcpp_components)

install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
