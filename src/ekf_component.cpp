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
  declare_parameter("receive_odom", false);
  get_parameter("receive_odom", receive_odom_);

  A = Eigen::MatrixXd::Zero(10, 10);
  B = Eigen::MatrixXd::Zero(10, 6);
  C = Eigen::MatrixXd::Zero(10, 10);
  M = Eigen::MatrixXd::Zero(6, 6);
  Q = Eigen::MatrixXd::Zero(10, 10);
  K = Eigen::MatrixXd::Zero(10, 10);
  S = Eigen::MatrixXd::Zero(10, 10);
  P = Eigen::MatrixXd::Zero(10, 10);
  I = Eigen::MatrixXd::Identity(10, 10);
  x = Eigen::VectorXd::Zero(10);
  y = Eigen::VectorXd::Zero(10);
  u = Eigen::VectorXd::Zero(6);
  cov = Eigen::VectorXd::Zero(36);

  if (receive_odom_) {
    Odomsubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 100, std::bind(&EKFComponent::Odomtopic_callback, this, std::placeholders::_1));
    std::cout << "[INFO]: we use topic /odom for observation "<< std::endl;
  } else if (!receive_odom_) {
    GPSsubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/gps_pose", 100, std::bind(&EKFComponent::GPStopic_callback, this, std::placeholders::_1));
    std::cout << "[INFO]: we use topic /gps_pose for observation"<< std::endl;
  } else {
    std::cout << "[ERROR]: plz, check parameter receive_odom_ "<< std::endl;
  }

  IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 100, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1));

  Posepublisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 100);
}

void EKFComponent::GPStopic_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  init();
  gpstimestamp = msg->header.stamp;
  y(0) = msg->pose.pose.position.x;
  y(1) = msg->pose.pose.position.y;
  y(2) = msg->pose.pose.position.z;
  for (int i; i < 36; i++) {
    cov(i) = msg->pose.covariance[i];
  }
  update();
}

void EKFComponent::Odomtopic_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  odomtimestamp = msg->header.stamp;
  y(0) = msg->pose.pose.position.x;
  y(1) = msg->pose.pose.position.y;
  y(2) = msg->pose.pose.position.z;
  for (int i = 0; i < 36; i++) {
    cov(i) = msg->pose.covariance[i];
  }
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
    x << y(0), y(1), y(2), 0, 0, 0, 0, 0, 0, 0;
    P << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1;
  }
  return initialized = true;
}

void EKFComponent::modelfunc()
{
  double xx, xy, xz, vx, vy, vz, w0, w1, w2, w3;
  xx = x(0);
  xy = x(1);
  xz = x(2);
  vx = x(3);
  vy = x(4);
  vz = x(5);
  w0 = x(6);
  w1 = x(7);
  w2 = x(8);
  w3 = x(9);

  x(0) = xx + vx * dt;
  x(1) = xy + vy * dt;
  x(2) = xz + vz * dt;

  x(3) = vx + ((w0 * w0 + w1 * w1 - w2 * w2 - w3 * w3) * u(0) + (2 * w1 * w2 - 2 * w0 * w3) * u(1) +
               (2 * w1 * w3 - 2 * w0 * w2) * u(2)) *
                dt;
  x(4) = vx + ((2 * w1 * w2 + 2 * w0 * w3) * u(0) + (w0 * w0 + w2 * w2 - w1 * w1 - w3 * w3) * u(1) +
               (2 * w2 * w3 - 2 * w0 * w1) * u(2)) *
                dt;
  x(5) = vx + ((2 * w1 * w3 - 2 * w0 * w2) * u(0) + (2 * w2 * w3 + 2 * w0 * w1) * u(1) +
               (w0 * w0 + w3 * w3 - w1 * w1 - w2 * w2) * u(2) - 9.81) *
                dt;

  x(6) = (-u(3) * w1 - u(4) * w2 - u(5) * w3) * dt + w0;
  x(7) = (u(3) * w0 + u(5) * w2 - u(4) * w3) * dt + w1;
  x(8) = (u(4) * w0 - u(5) * w1 + u(3) * w2) * dt + w2;
  x(9) = (u(5) * w0 + u(4) * w1 - u(3) * w2) * dt + w3;
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

  // if you have GPS covariance, you can use here.
  /*
  Q << cov(0), cov(1), cov(2), 0, 0, 0, 0, cov(3), cov(4), cov(5), cov(6), cov(7), cov(8), 0, 0, 0,
    0, cov(9), cov(10), cov(11), cov(12), cov(13), cov(14), 0, 0, 0, 0, cov(15), cov(16), cov(17),
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, cov(18), cov(19), cov(20), 0, 0, 0, 0, cov(21), cov(22), cov(23),
    cov(24), cov(25), cov(26), 0, 0, 0, 0, cov(27), cov(28), cov(29), cov(30), cov(31), cov(32), 0,
    0, 0, 0, cov(33), cov(34), cov(35);
  */

  Q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1;

  // 予測ステップ

  modelfunc();
  //x = A*x + B*u;
  P = A * P * A.transpose() + B * M * B.transpose();
  S = C * P * C.transpose() + Q;
  K = P * C.transpose() * S.inverse();
  // フィルタリングステップ
  x += K * (y - C * x);
  P = (I - K * C) * P;

  geometry_msgs::msg::PoseWithCovarianceStamped pose_ekf;
  if (receive_odom_) {
    pose_ekf.header.stamp = odomtimestamp;
  }
  if (!receive_odom_) {
    pose_ekf.header.stamp = gpstimestamp;
  }
  pose_ekf.header.frame_id = "/map";
  pose_ekf.pose.pose.position.x = x(0);
  pose_ekf.pose.pose.position.y = x(1);
  pose_ekf.pose.pose.position.z = x(2);

  pose_ekf.pose.covariance = {
    P(0, 0), P(0, 1), P(0, 2), P(0, 7), P(0, 8), P(0, 9), P(1, 0), P(1, 1), P(1, 2),
    P(1, 7), P(1, 8), P(1, 9), P(2, 0), P(2, 1), P(2, 2), P(2, 7), P(2, 8), P(2, 9),
    P(7, 0), P(7, 1), P(7, 2), P(7, 7), P(7, 8), P(7, 9), P(8, 0), P(8, 1), P(8, 2),
    P(8, 7), P(8, 8), P(8, 9), P(9, 0), P(9, 1), P(9, 2), P(9, 7), P(9, 8), P(9, 9)};

  Posepublisher_->publish(pose_ekf);
}
}  // namespace robotx_ekf
