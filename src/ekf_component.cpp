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

#include "robotx_ekf/ekf_component.hpp"

#include <chrono>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace robotx_ekf
{
EKFComponent::EKFComponent(const rclcpp::NodeOptions & options)
: Node("robotx_ekf_node", options), broadcaster_(this)
{
  // パラメータの宣言と取得

  this->declare_parameters();

  // 行列の初期化
  this->initialize_matrices();

  // トピックのサブスクリプションとパブリッシャの設定
  this->setup_subscribers_and_publishers();

  // タイマーの設定
  timer_ = this->create_wall_timer(10ms, std::bind(&EKFComponent::timer_callback, this));
}

void EKFComponent::declare_parameters()
{
  declare_parameter("receive_odom", false);
  get_parameter("receive_odom", receive_odom_);
  declare_parameter("broadcast_transform", true);
  get_parameter("broadcast_transform", broadcast_transform_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("map_frame_id", "map");
  get_parameter("map_frame_id", map_frame_id_);

  declare_parameter("topic_covariance", false);
  get_parameter("topic_covariance", topic_covariance_);

  declare_parameter("position_covariance_x", 0.1);
  get_parameter("position_covariance_x", covariance.position_covariance.x);
  declare_parameter("position_covariance_y", 0.1);
  get_parameter("position_covariance_y", covariance.position_covariance.y);
  declare_parameter("position_covariance_z", 0.1);
  get_parameter("position_covariance_z", covariance.position_covariance.z);

  declare_parameter("orientation_covariance_x", 0.01);
  get_parameter("orientation_covariance_x", covariance.orientation_covariance.x);
  declare_parameter("orientation_covariance_y", 0.01);
  get_parameter("orientation_covariance_y", covariance.orientation_covariance.y);
  declare_parameter("orientation_covariance_z", 0.01);
  get_parameter("orientation_covariance_z", covariance.orientation_covariance.z);
  declare_parameter("orientation_covariance_w", 0.01);
  get_parameter("orientation_covariance_w", covariance.orientation_covariance.w);
}

void EKFComponent::initialize_matrices()
{
  state_ = Eigen::VectorXd::Zero(10);
  acceleration_ = Eigen::VectorXd::Zero(3);
  gyro_ = Eigen::VectorXd::Zero(3);
  position_from_gnss_ = Eigen::VectorXd::Zero(3);

  A_ = Eigen::MatrixXd::Zero(10, 10);
  B_ = Eigen::MatrixXd::Zero(10, 6);
  C_ = Eigen::MatrixXd::Zero(6, 10);
  M_ = Eigen::MatrixXd::Zero(6, 6);
  Q_ = Eigen::MatrixXd::Zero(6, 6);
  K_ = Eigen::MatrixXd::Zero(10, 6);
  S_ = Eigen::MatrixXd::Zero(6, 6);
  P_ = Eigen::MatrixXd::Zero(10, 10);
  E_ = Eigen::MatrixXd::Zero(3, 3);
  I = Eigen::MatrixXd::Identity(10, 10);
  cov = Eigen::VectorXd::Zero(36);
  G = Eigen::VectorXd::Zero(3);
  G << 0, 0, -g;

  is_initialized_ = false;
}

void EKFComponent::setup_subscribers_and_publishers()
{
  // オドメトリまたはGPSのサブスクリプション
  if (receive_odom_) {
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&EKFComponent::Odomtopic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Using topic /odom for observation");
  } else {
    if (topic_covariance_) {
      gps_pose_with_covariance_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "gps_pose", 10,
          std::bind(&EKFComponent::GPStopic_covariance_callback, this, std::placeholders::_1));
    } else {
      gps_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "gps_pose", 10, std::bind(&EKFComponent::GPStopic_callback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(this->get_logger(), "Using topic gps_pose for observation");
  }

  // IMUデータのサブスクリプション
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1));

  // EKFの推定結果をパブリッシュ
  if (topic_covariance_) {
    ekf_pose_with_covariance_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/current_pose", 10);
  } else {
    ekf_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
  }
}

// put position from GNSS sensor
void EKFComponent::GPStopic_covariance_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // GNSSデータからタイムスタンプを取得
  gpstimestamp = msg->header.stamp;

  // 位置情報を更新
  position_from_gnss_(0) = msg->pose.pose.position.x;
  position_from_gnss_(1) = msg->pose.pose.position.y;
  position_from_gnss_(2) = msg->pose.pose.position.z;

  // 共分散行列を更新
  for (int i = 0; i < 36; i++) {
    cov(i) = msg->pose.covariance[i];
  }

  // Initialize
  if (!is_initialized_) {
    state_(0) = position_from_gnss_(0);
    state_(1) = position_from_gnss_(1);
    state_(2) = position_from_gnss_(2);

    // if you can get orientation from gnss
    // state_(6) = msg->pose.pose.orientation.w; // qx
    // state_(7) = msg->pose.pose.orientation.state; // qy
    // state_(8) = msg->pose.pose.orientation.y; // qz
    // state_(9) = msg->pose.pose.orientation.z; // qw

    state_(6) = 1.0;  // qx
    state_(7) = 0.0;  // qy
    state_(8) = 0.0;  // qz
    state_(9) = 0.0;  // qw

    const double P_x = 0.01;
    P_ << P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x;

    is_initialized_ = true;  // 初期化が完了したことを示す
  }
}

// put position from GNSS sensor
void EKFComponent::GPStopic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // GNSSデータからタイムスタンプを取得
  gpstimestamp = msg->header.stamp;

  // 位置情報を更新
  position_from_gnss_(0) = msg->pose.position.x;
  position_from_gnss_(1) = msg->pose.position.y;
  position_from_gnss_(2) = msg->pose.position.z;

  // Initialize
  if (!is_initialized_) {
    state_(0) = position_from_gnss_(0);
    state_(1) = position_from_gnss_(1);
    state_(2) = position_from_gnss_(2);

    // if you can get orientation from gnss
    // state_(6) = msg->pose.orientation.w; // qx
    // state_(7) = msg->pose.orientation.state; // qy
    // state_(8) = msg->pose.orientation.y; // qz
    // state_(9) = msg->pose.orientation.z; // qw

    state_(6) = 1.0;  // qx
    state_(7) = 0.0;  // qy
    state_(8) = 0.0;  // qz
    state_(9) = 0.0;  // qw

    const double P_x = 0.01;
    P_ << P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x;

    is_initialized_ = true;  // 初期化が完了したことを示す
  }
}

void EKFComponent::Odomtopic_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  odomtimestamp = msg->header.stamp;
  position_from_gnss_(0) = msg->pose.pose.position.x;
  position_from_gnss_(1) = msg->pose.pose.position.y;
  position_from_gnss_(2) = msg->pose.pose.position.z;
  for (int i = 0; i < 36; i++) {
    cov(i) = msg->pose.covariance[i];
  }
}

// put acceleration and gyro
void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // IMUデータを取得する
  imutimestamp = msg->header.stamp;

  // 加速度
  acceleration_(0) = msg->linear_acceleration.x;
  acceleration_(1) = msg->linear_acceleration.y;
  acceleration_(2) = msg->linear_acceleration.z;

  // 角速度
  gyro_(0) = msg->angular_velocity.x;
  gyro_(1) = msg->angular_velocity.y;
  gyro_(2) = msg->angular_velocity.z;
}

void EKFComponent::UpdateByStateEq(
  const Eigen::VectorXd & acceleration, const Eigen::VectorXd & gyro,
  Eigen::VectorXd & current_state)
{
  double xx, xy, xz, vx, vy, vz, q0, q1, q2, q3;
  xx = current_state(0);
  xy = current_state(1);
  xz = current_state(2);
  vx = current_state(3);
  vy = current_state(4);
  vz = current_state(5);
  q0 = current_state(6);
  q1 = current_state(7);
  q2 = current_state(8);
  q3 = current_state(9);

  // quaternion
  const double norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  Eigen::VectorXd u(6);
  u << acceleration, gyro;

  current_state(0) = xx + vx * dt;
  current_state(1) = xy + vy * dt;
  current_state(2) = xz + vz * dt;

  current_state(3) =
    vx + ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * u(0) + (2 * q1 * q2 - 2 * q0 * q3) * u(1) +
          (2 * q1 * q3 - 2 * q0 * q2) * u(2)) *
           dt;
  current_state(4) =
    vy + ((2 * q1 * q2 + 2 * q0 * q3) * u(0) + (q0 * q0 + q2 * q2 - q1 * q1 - q3 * q3) * u(1) +
          (2 * q2 * q3 - 2 * q0 * q1) * u(2)) *
           dt;
  current_state(5) = vz + ((2 * q1 * q3 - 2 * q0 * q2) * u(0) + (2 * q2 * q3 + 2 * q0 * q1) * u(1) +
                           (q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2) * u(2) + g) *
                            dt;

  current_state(6) = 0.5 * (-u(3) * q1 - u(4) * q2 - u(5) * q3) * dt + q0;
  current_state(7) = 0.5 * (u(3) * q0 + u(5) * q2 - u(4) * q3) * dt + q1;
  current_state(8) = 0.5 * (u(4) * q0 - u(5) * q1 + u(3) * q2) * dt + q2;
  current_state(9) = 0.5 * (u(5) * q0 + u(4) * q1 - u(3) * q2) * dt + q3;
}

void EKFComponent::CalculateMatrices(
  const Eigen::VectorXd & current_state, const Eigen::VectorXd & acceleration,
  const Eigen::VectorXd & gyro, Eigen::MatrixXd & A, Eigen::MatrixXd & B, Eigen::MatrixXd & C,
  Eigen::MatrixXd & M, Eigen::MatrixXd & Q, Eigen::MatrixXd & E, Eigen::MatrixXd & P,
  Eigen::MatrixXd & S, Eigen::MatrixXd & K)
{
  Eigen::VectorXd x(10);
  x << current_state;

  Eigen::VectorXd u(6);
  u << acceleration, gyro;

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
    0, 0, 0, 0, 0, 2 * (-x(8) * (-g)), 2 * (x(9) * (-g)), 2 * (-x(6) * (-g)), 2 * (x(7) * (-g)), 0,
    0, 0, 0, 0, 0, 2 * (x(7) * (-g)), 2 * (x(6) * (-g)), 2 * (-x(9) * (-g)), 2 * (x(8) * (-g)), 0,
    0, 0, 0, 0, 0, 2 * x(6) * (-g), 2 * -x(7) * (-g), 2 * -x(8) * (-g), 2 * x(9) * (-g);

  const double M_x = 50;
  M << M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, 10 * M_x, 0, 0, 0, 0, 0,
    0, 10 * M_x, 0, 0, 0, 0, 0, 0, 10 * M_x;

  const double Q_x = 3;
  Q << Q_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0,
    M_x, 0, 0, 0, 0, 0, 0, M_x;

  // if you have GPS covariance, you can use here.
  if (topic_covariance_) {
    Q << cov(0), cov(1), cov(2), 0, 0, 0, 0, cov(3), cov(4), cov(5), cov(6), cov(7), cov(8), 0, 0,
      0, 0, cov(9), cov(10), cov(11), cov(12), cov(13), cov(14), 0, 0, 0, 0, cov(15), cov(16),
      cov(17), 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, cov(18), cov(19), cov(20), 0, 0, 0, 0, cov(21), cov(22),
      cov(23), cov(24), cov(25), cov(26), 0, 0, 0, 0, cov(27), cov(28), cov(29), cov(30), cov(31),
      cov(32), 0, 0, 0, 0, cov(33), cov(34), cov(35);
  }

  // frame_base -> Inertial_base
  E << (x(6) * x(6) + x(7) * x(7) - x(8) * x(8) - x(9) * x(9)), 2 * (x(7) * x(8) - x(6) * x(9)),
    2 * (x(7) * x(9) + x(6) * x(8)), 2 * (x(7) * x(8) + x(6) * x(9)),
    (x(6) * x(6) - x(7) * x(7) + x(8) * x(8) - x(9) * x(9)), 2 * (x(8) * x(9) - x(6) * x(7)),
    2 * (x(7) * x(9) - x(6) * x(8)), 2 * (x(8) * x(9) + x(6) * x(7)),
    (x(6) * x(6) - x(7) * x(7) - x(8) * x(8) + x(9) * x(9));

  P = A * P * A.transpose() + B * M * B.transpose();

  S = C * P * C.transpose() + Q;

  K = P * C.transpose() * S.inverse();
}

void EKFComponent::UpdateByObservation(
  const Eigen::VectorXd & position_from_gnss, const Eigen::VectorXd & acceleration,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & E, const Eigen::MatrixXd & K,
  Eigen::VectorXd & current_state, Eigen::MatrixXd & P)
{
  Eigen::VectorXd zz(3);
  Eigen::VectorXd z(6);
  Eigen::VectorXd y(6);

  zz = E.transpose() * G;
  z << current_state(0), current_state(1), current_state(2), zz(0), zz(1), zz(2);

  y(0) = position_from_gnss(0);
  y(1) = position_from_gnss(1);
  y(2) = position_from_gnss(2);
  y(3) = acceleration(0);
  y(4) = acceleration(1);
  y(5) = acceleration(2);

  current_state = current_state + K * (y - z);

  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(10, 10);
  P = (I - K * C) * P;
}

void EKFComponent::publish_topic_covariance(
  const Eigen::VectorXd & current_state, const Eigen::MatrixXd & Pin)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_ekf;

  Eigen::VectorXd state(10);
  state << current_state;

  Eigen::MatrixXd P(10, 10);
  P << Pin;

  if (receive_odom_) {
    pose_ekf.header.stamp = std::max(odomtimestamp, imutimestamp);
  }
  if (!receive_odom_) {
    pose_ekf.header.stamp = std::max(gpstimestamp, imutimestamp);
  }

  pose_ekf.header.frame_id = map_frame_id_;
  pose_ekf.pose.pose.position.x = state(0);
  pose_ekf.pose.pose.position.y = state(1);
  pose_ekf.pose.pose.position.z = state(2);

  pose_ekf.pose.pose.orientation.w = state(3) / std::sqrt(
                                                  state(3) * state(3) + state(4) * state(4) +
                                                  state(5) * state(5) + state(6) * state(6));
  pose_ekf.pose.pose.orientation.x = state(4) / std::sqrt(
                                                  state(3) * state(3) + state(4) * state(4) +
                                                  state(5) * state(5) + state(6) * state(6));
  pose_ekf.pose.pose.orientation.y = state(5) / std::sqrt(
                                                  state(3) * state(3) + state(4) * state(4) +
                                                  state(5) * state(5) + state(6) * state(6));
  pose_ekf.pose.pose.orientation.z = state(6) / std::sqrt(
                                                  state(3) * state(3) + state(4) * state(4) +
                                                  state(5) * state(5) + state(6) * state(6));

  pose_ekf.pose.covariance = {
    P(0, 0), P(0, 1), P(0, 2), P(0, 7), P(0, 8), P(0, 9), P(1, 0), P(1, 1), P(1, 2),
    P(1, 7), P(1, 8), P(1, 9), P(2, 0), P(2, 1), P(2, 2), P(2, 7), P(2, 8), P(2, 9),
    P(7, 0), P(7, 1), P(7, 2), P(7, 7), P(7, 8), P(7, 9), P(8, 0), P(8, 1), P(8, 2),
    P(8, 7), P(8, 8), P(8, 9), P(9, 0), P(9, 1), P(9, 2), P(9, 7), P(9, 8), P(9, 9)};

  ekf_pose_with_covariance_publisher_->publish(pose_ekf);

  if (broadcast_transform_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = map_frame_id_;
    transform_stamped.header.stamp = pose_ekf.header.stamp;
    transform_stamped.child_frame_id = robot_frame_id_;
    transform_stamped.transform.translation.x = pose_ekf.pose.pose.position.x;
    transform_stamped.transform.translation.y = pose_ekf.pose.pose.position.y;
    transform_stamped.transform.translation.z = pose_ekf.pose.pose.position.z;
    transform_stamped.transform.rotation = pose_ekf.pose.pose.orientation;
    broadcaster_.sendTransform(transform_stamped);
  }
}

void EKFComponent::publish_topic(const Eigen::VectorXd & current_state)
{
  geometry_msgs::msg::PoseStamped pose_ekf;

  Eigen::VectorXd state(10);
  state << current_state;

  if (receive_odom_) {
    pose_ekf.header.stamp = std::max(odomtimestamp, imutimestamp);
  }
  if (!receive_odom_) {
    pose_ekf.header.stamp = std::max(gpstimestamp, imutimestamp);
  }

  pose_ekf.header.frame_id = map_frame_id_;
  pose_ekf.pose.position.x = state(0);
  pose_ekf.pose.position.y = state(1);
  pose_ekf.pose.position.z = state(2);

  pose_ekf.pose.orientation.w = state(3) / std::sqrt(
                                             state(3) * state(3) + state(4) * state(4) +
                                             state(5) * state(5) + state(6) * state(6));
  pose_ekf.pose.orientation.x = state(4) / std::sqrt(
                                             state(3) * state(3) + state(4) * state(4) +
                                             state(5) * state(5) + state(6) * state(6));
  pose_ekf.pose.orientation.y = state(5) / std::sqrt(
                                             state(3) * state(3) + state(4) * state(4) +
                                             state(5) * state(5) + state(6) * state(6));
  pose_ekf.pose.orientation.z = state(6) / std::sqrt(
                                             state(3) * state(3) + state(4) * state(4) +
                                             state(5) * state(5) + state(6) * state(6));

  ekf_pose_publisher_->publish(pose_ekf);

  if (broadcast_transform_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = map_frame_id_;
    transform_stamped.header.stamp = pose_ekf.header.stamp;
    transform_stamped.child_frame_id = robot_frame_id_;
    transform_stamped.transform.translation.x = pose_ekf.pose.position.x;
    transform_stamped.transform.translation.y = pose_ekf.pose.position.y;
    transform_stamped.transform.translation.z = pose_ekf.pose.position.z;
    transform_stamped.transform.rotation = pose_ekf.pose.orientation;
    broadcaster_.sendTransform(transform_stamped);
  }
}

void EKFComponent::CalcPositionByEKF(
  const Eigen::VectorXd & acceleration, const Eigen::VectorXd & gyro,
  const Eigen::VectorXd & position_from_gnss)
{
  if (!is_initialized_) {
    return;  // 初期化が完了するまで更新をスキップ
  }

  Eigen::VectorXd current_state(9);
  current_state = state_;

  // set past value to Matrix
  Eigen::MatrixXd A = A_;
  Eigen::MatrixXd B = B_;
  Eigen::MatrixXd C = C_;
  Eigen::MatrixXd M = M_;
  Eigen::MatrixXd Q = Q_;
  Eigen::MatrixXd E = E_;
  Eigen::MatrixXd P = P_;
  Eigen::MatrixXd S = S_;
  Eigen::MatrixXd K = K_;

  UpdateByStateEq(acceleration, gyro, current_state);

  CalculateMatrices(current_state, acceleration, gyro, A, B, C, M, Q, E, P, S, K);

  UpdateByObservation(position_from_gnss, acceleration, C, E, K, current_state, P);

  A_ = A;
  B_ = B;
  C_ = C;
  M_ = M;
  Q_ = Q;
  E_ = E;
  P_ = P;
  S_ = S;
  K_ = K;
  state_ = current_state;

  if (topic_covariance_) {
    publish_topic_covariance(current_state, P);
  } else {
    publish_topic(current_state);
  }
}

void EKFComponent::timer_callback()
{
  // 加速度、ジャイロ、位置の例（実際にはセンサーからのデータを使用）
  Eigen::VectorXd acceleration = acceleration_;  // 実際の加速度データを取得
  Eigen::VectorXd gyro = gyro_;                  // 実際のジャイロデータを取得
  Eigen::VectorXd position_from_gnss = position_from_gnss_;  // 実際の位置データを取得

  // EKF計算の呼び出し
  CalcPositionByEKF(acceleration, gyro, position_from_gnss);
}

}  // namespace robotx_ekf
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_ekf::EKFComponent)
