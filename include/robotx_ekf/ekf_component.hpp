// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOTX_EKF__EKF_COMPONENT_HPP_
#define ROBOTX_EKF__EKF_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define ROBOTX_EKF__EKF_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define ROBOTX_EKF__EKF_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define ROBOTX_EKF__EKF_COMPONENT_EXPORT __declspec(dllexport)
        #define ROBOTX_EKF__EKF_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef ROBOTX_EKF__EKF_COMPONENT_BUILDING_DLL
        #define ROBOTX_EKF__EKF_COMPONENT_PUBLIC \
  ROBOTX_EKF__EKF_COMPONENT_EXPORT
    #else
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC \
  ROBOTX_EKF__EKF_COMPONENT_IMPORT
    #endif
        #define ROBOTX_EKF__EKF_COMPONENT_PUBLIC_TYPE \
  ROBOTX_EKF__EKF_COMPONENT_PUBLIC
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_LOCAL
#else
    #define ROBOTX_EKF__EKF_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define ROBOTX_EKF__EKF_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define ROBOTX_EKF__EKF_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define ROBOTX_EKF__EKF_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define ROBOTX_EKF__EKF_COMPONENT_PUBLIC
        #define ROBOTX_EKF__EKF_COMPONENT_LOCAL
    #endif
    #define ROBOTX_EKF__EKF_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/PoseWithCovarianceStamped>
#include <geometry_msgs/msg/PoseStamped>
#include <sensor_msgs/msg/Imu>
#include <Eigen/Dense>

{
class EKFComponent : public rclcpp::Node
{
public:
  ROBOTX_EKF__EKF_COMPONENT_PUBLIC
  explicit EKFComponent(const rclcpp::NodeOptions & options);

private:
  bool init();
  void GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void update();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr GPSsubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Posepublisher_;
  size_t count_;
  double dt;
  bool initialized = false;
  Eigen::Vector10d x(10);
  Eigen::Vector10d y(10);
  Eigen::Vector6d u(6);
  Eigen::Matrix10d P(10,10);
};
}  // namespace robotx_ekf

#endif  // ROBOTX_EKF__EKF_COMPONENT_HPP_