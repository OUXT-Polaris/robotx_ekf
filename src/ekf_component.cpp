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

#include <Eigen/Dense>
#include <robotx_ekf/ekf_component.hpp>

namespace robotx_ekf
{
EKFComponent::EKFComponent(const rclcpp::NodeOptions & options) : Node("robotx_ekf", options)
{
  A = Eigen::MatrixXd::Zero(10, 10);
  B = Eigen::MatrixXd::Zero(10, 6);
  C = Eigen::MatrixXd::Zero(10, 10);
  M = Eigen::MatrixXd::Zero(6, 6);
  Q = Eigen::MatrixXd::Zero(10, 10);
  K = Eigen::MatrixXd::Zero(10, 10);
  P = Eigen::MatrixXd::Zero(10, 10);
  I = Eigen::MatrixXd::Identity(10, 10);

  x = Eigen::VectorXd::Zero(10);
  y = Eigen::VectorXd::Zero(10);
  u = Eigen::VectorXd::Zero(6);
  x_hat = Eigen::VectorXd::Zero(6);

  GPSsubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&EKFComponent::GPStopic_callback, this, std::placeholders::_1));

  IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1));

  Posepublisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
}

void EKFComponent::GPStopic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  gpstimestamp = msg->header.stamp;
  y(0) = msg->pose.pose.position.x;
  y(1) = msg->pose.pose.position.y;
  y(2) = msg->pose.pose.position.z;
  std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
  std::cout << "Receive GPS DATA" << std::endl;
  std::cout << "=============================" << std::endl;
  std::cout << "ts: " << std::endl;
  std::cout << "x: " << y(0) << std::endl;
  std::cout << "y: " << y(1) << std::endl;
  std::cout << "z: " << y(2) << std::endl;

  init();
  update();
}

void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imutimestamp = msg->header.stamp;
  u(0) = msg->linear_acceleration.x;
  u(1) = msg->linear_acceleration.y;
  u(2) = msg->linear_acceleration.z;
  u(3) = msg->angular_velocity.x;
  u(4) = msg->angular_velocity.y;
  u(5) = msg->angular_velocity.z;
}

bool EKFComponent::init()
{
  if (!initialized) {
    x << y(1), y(2), y(3), 0, 0, 0, 1, 0, 0, 0;
    P << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1;
  }
  return initialized = true;
}

void EKFComponent::update()
{
  if (!initialized) {
    std::cout << "NOT Initialized" << std::endl;
  }

  A << 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, (2 * x(6) * u(0) - 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt,
    (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt,
    (-2 * x(8) * u(0) + 2 * x(7) * u(1) + 2 * x(6) * u(2)) * dt,
    (-2 * x(9) * u(0) - 2 * x(6) * u(1) + 2 * x(7) * u(2)) * dt, 0, 0, 0, 0, 1, 0,
    (2 * x(9) * u(0) + 2 * x(6) * u(1) - 2 * x(7) * u(2)) * dt,
    (2 * x(8) * u(0) - 2 * x(7) * u(1) - 2 * x(6) * u(2)) * dt,
    (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt,
    (2 * x(6) * u(0) - 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt, 0, 0, 0, 0, 0, 1,
    (-2 * x(9) * u(0) + 2 * x(7) * u(1) + 2 * x(6) * u(2)) * dt,
    (2 * x(9) * u(0) + 2 * x(6) * u(1) - 2 * x(7) * u(2)) * dt,
    (-2 * x(6) * u(0) + 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt,
    (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt, 0, 0, 0, 0, 0, 0, 1, -dt * u(3),
    -dt * u(4), -dt * u(5), 0, 0, 0, 0, 0, 0, dt * u(3), 1, dt * u(5), -dt * u(4), 0, 0, 0, 0, 0, 0,
    dt * u(4), -dt * u(5), 1, dt * u(3), 0, 0, 0, 0, 0, 0, dt * u(5), dt * u(4), -dt * u(3), 1;
  B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    (x(6) * x(6) + x(7) * x(7) - x(8) * x(8) - x(9) * x(9)) * dt,
    2 * (x(7) * x(8) - x(6) * x(9)) * dt, 2 * (x(7) * x(9) + x(6) * x(8)) * dt, 0, 0, 0,
    2 * (x(7) * x(8) - x(6) * x(9)) * dt,
    (x(6) * x(6) - x(7) * x(7) + x(8) * x(8) - x(9) * x(9)) * dt,
    2 * (x(8) * x(9) + x(6) * x(7)) * dt, 0, 0, 0, 2 * (x(7) * x(9) + x(6) * x(8)) * dt,
    2 * (x(8) * x(9) - x(6) * x(7)) * dt,
    (x(6) * x(6) + x(7) * x(7) - x(8) * x(8) - x(9) * x(9)) * dt, 0, 0, 0, 0, 0, 0, -x(7) * dt,
    x(8) * dt, -x(9) * dt, 0, 0, 0, x(6) * dt, -x(9) * dt, x(8) * dt, 0, 0, 0, x(9) * dt, x(6) * dt,
    -x(7) * dt, 0, 0, 0, -x(8) * dt, x(7) * dt, x(6) * dt;
  C << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0;
  M << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1;
  Q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1;

  // 予測ステップ
  x_hat = A * x + B * u;
  P = A * P * A.transpose() + B * M * B.transpose();
  K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();
  // フィルタリングステップ
  x_hat += K * (y - C * x_hat);
  P = (I - K * C) * P;
  x = x_hat;

  geometry_msgs::msg::PoseWithCovarianceStamped pose_ekf{};
  pose_ekf.header.stamp = imutimestamp;
  pose_ekf.header.frame_id = "/map";
  pose_ekf.pose.pose.position.x = x(0);
  pose_ekf.pose.pose.position.y = x(1);
  pose_ekf.pose.pose.position.z = x(2);
  pose_ekf.pose.covariance = {
    P(0, 0), P(0, 1), P(0, 2), P(0, 7), P(0, 8), P(0, 9), P(1, 0), P(1, 1), P(1, 2),
    P(1, 7), P(1, 8), P(1, 9), P(2, 0), P(2, 1), P(2, 2), P(2, 7), P(2, 8), P(2, 9),
    P(7, 0), P(7, 1), P(7, 2), P(7, 7), P(7, 8), P(7, 9), P(8, 0), P(8, 1), P(8, 2),
    P(8, 7), P(8, 8), P(8, 9), P(9, 0), P(9, 1), P(9, 2), P(9, 7), P(9, 8), P(9, 9)};
  std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
  std::cout << "Publish Pose DATA" << std::endl;
  std::cout << "=============================" << std::endl;
  std::cout << "ts: " << std::endl;
  std::cout << "x: " << x(0) << std::endl;
  std::cout << "y: " << x(1) << std::endl;
  std::cout << "z: " << x(2) << std::endl;

  Posepublisher_->publish(pose_ekf);
}
}  // namespace robotx_ekf
