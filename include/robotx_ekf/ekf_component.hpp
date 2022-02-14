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
#define ROBOTX_EKF__EKF_COMPONENT_EXPORT __attribute__((dllexport))
#define ROBOTX_EKF__EKF_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define ROBOTX_EKF__EKF_COMPONENT_EXPORT __declspec(dllexport)
#define ROBOTX_EKF__EKF_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOTX_EKF__EKF_COMPONENT_BUILDING_DLL
#define ROBOTX_EKF__EKF_COMPONENT_PUBLIC ROBOTX_EKF__EKF_COMPONENT_EXPORT
#else
#define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC ROBOTX_EKF__EKF_COMPONENT_IMPORT
#endif
#define ROBOTX_EKF__EKF_COMPONENT_PUBLIC_TYPE ROBOTX_EKF__EKF_COMPONENT_PUBLIC
#define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_LOCAL
#else
#define ROBOTX_EKF__EKF_COMPONENT_EXPORT __attribute__((visibility("default")))
#define ROBOTX_EKF__EKF_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ROBOTX_EKF__EKF_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define ROBOTX_EKF__EKF_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOTX_EKF__EKF_COMPONENT_PUBLIC
#define ROBOTX_EKF__EKF_COMPONENT_LOCAL
#endif
#define ROBOTX_EKF__EKF_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace robotx_ekf
{
class EKFComponent : public rclcpp::Node
{
public:
  ROBOTX_EKF__EKF_COMPONENT_PUBLIC
  explicit EKFComponent(const rclcpp::NodeOptions & options);

  double dt = 0.0025;
  bool initialized = false;
  Eigen::MatrixXd P;
  Eigen::VectorXd X;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd yy;
  Eigen::VectorXd u;
  Eigen::MatrixXd I;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd Cy;
  Eigen::MatrixXd M;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd L;
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
  Eigen::MatrixXd Ky;
  Eigen::MatrixXd Sy;
  Eigen::VectorXd cov;

  rclcpp::Time odomtimestamp;
  rclcpp::Time gpstimestamp;
  rclcpp::Time imutimestamp;

private:
  bool receive_odom_;
  void GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void Odomtopic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void modelfunc();
  void jacobi();
  bool init();
  void update();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr GPSsubscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odomsubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
};
}  // namespace robotx_ekf

#endif  // ROBOTX_EKF__EKF_COMPONENT_HPP_
