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
  declare_parameter("receive_odom", false);
  get_parameter("receive_odom", receive_odom_);
  declare_parameter("broadcast_map_transform", true);
  get_parameter("broadcast_transform", broadcast_transform_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("map_frame_id", "map");
  get_parameter("map_frame_id", map_frame_id_);

  A = Eigen::MatrixXd::Zero(10, 10);
  B = Eigen::MatrixXd::Zero(10, 6);
  C = Eigen::MatrixXd::Zero(6, 10);

  M = Eigen::MatrixXd::Zero(6, 6);
  Q = Eigen::MatrixXd::Zero(6, 6);

  K = Eigen::MatrixXd::Zero(10, 6);
  S = Eigen::MatrixXd::Zero(6, 6);
  P = Eigen::MatrixXd::Zero(10, 10);
  E = Eigen::MatrixXd::Zero(3, 3);
  I = Eigen::MatrixXd::Identity(10, 10);

  x = Eigen::VectorXd::Zero(10);

  u = Eigen::VectorXd::Zero(6);
  cov = Eigen::VectorXd::Zero(36);

  y = Eigen::VectorXd::Zero(6);
  z = Eigen::VectorXd::Zero(6);
  a = Eigen::VectorXd::Zero(3);
  am = Eigen::VectorXd::Zero(3);

  G = Eigen::VectorXd::Zero(3);

  if (receive_odom_) {
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&EKFComponent::Odomtopic_callback, this, std::placeholders::_1));
    std::cout << "[INFO]: we use topic /odom for observation " << std::endl;

  } else if (!receive_odom_) {
    gps_pose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/gps_pose", 10, std::bind(&EKFComponent::GPStopic_callback, this, std::placeholders::_1));
    std::cout << "[INFO]: we use topic /gps_pose for observation" << std::endl;
  } else {
    std::cout << "[ERROR]: plz, check parameter receive_odom_" << std::endl;
  }

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1));

  ekf_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

  timer_ = this->create_wall_timer(10ms, std::bind(&EKFComponent::update, this));
}

void EKFComponent::GPStopic_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::optional optional_y = msg->pose.pose.position.x;
  if (optional_y.has_value()) {
    gpstimestamp = msg->header.stamp;
    y(0) = msg->pose.pose.position.x;
    y(1) = msg->pose.pose.position.y;
    y(2) = msg->pose.pose.position.z;
    for (int i; i < 36; i++) {
      cov(i) = msg->pose.covariance[i];
    }
  }
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
}

void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::optional optional_imu = msg->linear_acceleration.x;
  if (optional_imu.has_value()) {
    imutimestamp = msg->header.stamp;
    u(0) = msg->linear_acceleration.x;
    u(1) = msg->linear_acceleration.y;
    u(2) = msg->linear_acceleration.z;
    u(3) = msg->angular_velocity.x;
    u(4) = msg->angular_velocity.y;
    u(5) = msg->angular_velocity.z;

    am << u(0), u(1), u(2);
  }
}

void EKFComponent::LPF() { a = a + k * (am - a); }

void EKFComponent::prefilter()
{
  Eigen::VectorXd a_dr(3);
  a_dr = E * a - G;
  a = a - E.transpose() * a_dr;
  double norm_am = std::sqrt(u(0) * u(0) + u(1) * u(1) + u(2) * u(2));
  if (norm_am < g + eps) {
    a_dr << 0, 0, 0;
  }
  a = a + E.transpose() * a_dr;
}

bool EKFComponent::init()
{
  std::optional optional_init = y(0);
  // optional_init.has_value()
  if (y(0) != 0 && u(0) != 0) {
    x << y(0), y(1), y(2), 0, 0, 0, 1, 0, 0, 0;
    // x << 6000000, -280000, -1, 0, 0, 0, 1, 0, 0, 0;
    double P_x = 1;
    P << P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, P_x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, P_x;

    G << 0, 0, -g;
    initialized = true;
  } else {
    initialized = false;
  }

  return initialized;
}

void EKFComponent::modelfunc()
{
  double xx, xy, xz, vx, vy, vz, q0, q1, q2, q3;
  xx = x(0);
  xy = x(1);
  xz = x(2);
  vx = x(3);
  vy = x(4);
  vz = x(5);
  q0 = x(6);
  q1 = x(7);
  q2 = x(8);
  q3 = x(9);

  x(0) = xx + vx * dt;
  x(1) = xy + vy * dt;
  x(2) = xz + vz * dt;

  x(3) = vx + ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * u(0) + (2 * q1 * q2 - 2 * q0 * q3) * u(1) +
               (2 * q1 * q3 - 2 * q0 * q2) * u(2)) *
                dt;
  x(4) = vy + ((2 * q1 * q2 + 2 * q0 * q3) * u(0) + (q0 * q0 + q2 * q2 - q1 * q1 - q3 * q3) * u(1) +
               (2 * q2 * q3 - 2 * q0 * q1) * u(2)) *
                dt;
  x(5) = vz + ((2 * q1 * q3 - 2 * q0 * q2) * u(0) + (2 * q2 * q3 + 2 * q0 * q1) * u(1) +
               (q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2) * u(2) + g) *
                dt;

  x(6) = 0.5 * (-u(3) * q1 - u(4) * q2 - u(5) * q3) * dt + q0;
  x(7) = 0.5 * (u(3) * q0 + u(5) * q2 - u(4) * q3) * dt + q1;
  x(8) = 0.5 * (u(4) * q0 - u(5) * q1 + u(3) * q2) * dt + q2;
  x(9) = 0.5 * (u(5) * q0 + u(4) * q1 - u(3) * q2) * dt + q3;
}

void EKFComponent::observation()
{
  Eigen::VectorXd zz(3);
  zz = E.transpose() * G;
  z << x(0), x(1), x(2), zz(0), zz(1), zz(2);

  y(3) = a(0);
  y(4) = a(1);
  y(5) = a(2);
}

void EKFComponent::jacobi()
{
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

  double M_x = 50;
  M << M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, 10 * M_x, 0, 0, 0, 0, 0,
    0, 10 * M_x, 0, 0, 0, 0, 0, 0, 10 * M_x;

  // if you have GPS covariance, you can use here.

  // Q << cov(0), cov(1), cov(2), 0, 0, 0, 0, cov(3), cov(4), cov(5), cov(6), cov(7), cov(8), 0, 0, 0,
  //   0, cov(9), cov(10), cov(11), cov(12), cov(13), cov(14), 0, 0, 0, 0, cov(15), cov(16), cov(17),
  //   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
  //   0, 0, 0, 0, 1, 0, 0, 0, cov(18), cov(19), cov(20), 0, 0, 0, 0, cov(21), cov(22), cov(23),
  //   cov(24), cov(25), cov(26), 0, 0, 0, 0, cov(27), cov(28), cov(29), cov(30), cov(31), cov(32), 0,
  //   0, 0, 0, cov(33), cov(34), cov(35);

  double Q_x = 3;
  Q << Q_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0,
    M_x, 0, 0, 0, 0, 0, 0, M_x;

  // frame_base -> Inertial_base
  E << (x(6) * x(6) + x(7) * x(7) - x(8) * x(8) - x(9) * x(9)), 2 * (x(7) * x(8) - x(6) * x(9)),
    2 * (x(7) * x(9) + x(6) * x(8)), 2 * (x(7) * x(8) + x(6) * x(9)),
    (x(6) * x(6) - x(7) * x(7) + x(8) * x(8) - x(9) * x(9)), 2 * (x(8) * x(9) - x(6) * x(7)),
    2 * (x(7) * x(9) - x(6) * x(8)), 2 * (x(8) * x(9) + x(6) * x(7)),
    (x(6) * x(6) - x(7) * x(7) - x(8) * x(8) + x(9) * x(9));
}

void EKFComponent::update()
{
  //std::cout << "a" << std::endl;
  if (!initialized) {
    init();
  } else {
    //std::cout << "b" << std::endl;
    // prefilter
    LPF();
    //std::cout << "c" << std::endl;
    // prefilter();
    //std::cout << "d" << std::endl;
    // 予測ステップ

    modelfunc();
    //std::cout << "e" << std::endl;
    jacobi();
    //std::cout << "f" << std::endl;
    // filtering step 1
    P = A * P * A.transpose() + B * M * B.transpose();
    //std::cout << "g" << std::endl;
    S = C * P * C.transpose() + Q;
    //std::cout << "h" << std::endl;
    K = P * C.transpose() * S.inverse();
    //std::cout << "i" << std::endl;
    observation();
    //std::cout << "j" << std::endl;
    x = x + K * (y - z);
    //std::cout << "k" << std::endl;
    P = (I - K * C) * P;
    //std::cout << "l" << std::endl;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_ekf;
    if (receive_odom_) {
      pose_ekf.header.stamp = odomtimestamp;
    }
    if (!receive_odom_) {
      pose_ekf.header.stamp = gpstimestamp;
    }
    pose_ekf.header.frame_id = map_frame_id_;
    pose_ekf.pose.pose.position.x = x(0);
    pose_ekf.pose.pose.position.y = x(1);
    pose_ekf.pose.pose.position.z = x(2);

    pose_ekf.pose.pose.orientation.w = x(3);
    pose_ekf.pose.pose.orientation.x = x(4);
    pose_ekf.pose.pose.orientation.y = x(5);
    pose_ekf.pose.pose.orientation.z = x(6);

    //std::cout << "m" << std::endl;
    pose_ekf.pose.covariance = {
      P(0, 0), P(0, 1), P(0, 2), P(0, 7), P(0, 8), P(0, 9), P(1, 0), P(1, 1), P(1, 2),
      P(1, 7), P(1, 8), P(1, 9), P(2, 0), P(2, 1), P(2, 2), P(2, 7), P(2, 8), P(2, 9),
      P(7, 0), P(7, 1), P(7, 2), P(7, 7), P(7, 8), P(7, 9), P(8, 0), P(8, 1), P(8, 2),
      P(8, 7), P(8, 8), P(8, 9), P(9, 0), P(9, 1), P(9, 2), P(9, 7), P(9, 8), P(9, 9)};

    ekf_pose_publisher_->publish(pose_ekf);
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
}
}  // namespace robotx_ekf
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_ekf::EKFComponent)
