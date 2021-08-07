// Copyright (c) 2020 OUXT Polaris
//
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you mu(1) not use this file except in compliance with the License.
// You mu(1) obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <robotx_ekf/ekf_component.hpp>
#include <memory>
#include <Eigen/Dense>

namespace robotx_ekf
{
  EKFComponent::EKFComponent(const rclcpp::NodeOptions &options)
      : Node("robotx_ekf", options)
  {
    GPSsubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "GPS_Topic", 10, std::bind(&EKFComponent::GPStopic_callback, this, std::placeholders::_1, std::placeholders::_2));

    IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "IMU_Topic", 10, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1, std::placeholders::_2));

    Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Pub_Pose", 10);
  }
  void EKFComponent::GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, Eigen::vector10d &y(10)) const
  {
    y(0) = msg->pose.pose.position.x;
    y(1) = msg->pose.pose.position.y;
    y(2) = msg->pose.pose.position.z;
    return;
  }

  void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg, Eigen::vector6d &u(6)) const
  {
    u(0) = msg->linear_acceleration.x;
    u(1) = msg->linear_acceleration.y;
    u(2) = msg->linear_acceleration.z;
    u(3) = msg->angular_velocity.x;
    u(4) = msg->angular_velocity.y;
    u(5) = msg->angular_velocity.z;
    return;
  }

  bool EKFComponent::init()
  {
    x << y(1), y(2), y(3), 0, 0, 0, 1, 0, 0, 0;
    P << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    dt = 0.01;
    return initialized = true;
  }

  void EKFComponent::update()
  {
    Eigen::vector10d x_hat(10);
    Eigen::MatrixXd A, B, C, Q, M, Q, K;

    do
    {
      EKFComponent::GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
      EKFComponent::init();
    } while (!initialized);

    EKFComponent::GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu msg);
    A << 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 
        0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 
        0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 
        0, 0, 0, 1, 0, 0, (2 * x(6) * u(0) - 2 * x(9) * u(1) - 2 * x(8) * u(2)) * dt, (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt, 
        (-2 * x(8) * u(0) + 2 * x(7) * u(1) - 2 * x(6) * u(2)) * dt, (-2 * x(9) * u(0) - 2 * x(6) * u(1) + 2 * x(7) * u(2)) * dt, 
        0, 0, 0, 0, 1, 0, (-2 * x(9) * u(0) + 2 * x(6) * u(1) + 2 * x(7) * u(2)) * dt, (2 * x(8) * u(0) - 2 * x(7) * u(1) + 2 * x(6) * u(2)) * dt, 
        (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt, (-2 * x(6) * u(0) - 2 * x(9) * u(1) + 2 * x(8) * u(2)) * dt, 
        0, 0, 0, 0, 0, 1, (2 * x(9) * u(0) - 2 * x(7) * u(1) + 2 * x(6) * u(2)) * dt, (2 * x(9) * u(0) - 2 * x(6) * u(1) - 2 * x(7) * u(2)) * dt, 
        (2 * x(6) * u(0) + 2 * x(9) * u(1) - 2 * x(8) * u(2)) * dt, (2 * x(7) * u(0) + 2 * x(8) * u(1) + 2 * x(9) * u(2)) * dt, 
        0, 0, 0, 0, 0, 0, 1, -dt * 0.5 * u(3), -dt * 0.5 * u(4), -dt * 0.5 * u(5), 
        0, 0, 0, 0, 0, 0, dt * 0.5 * u(3), 1, dt * -0.5 * u(5), -dt * 0.5 * u(4), 
        0, 0, 0, 0, 0, 0, dt * 0.5 * u(4), -dt * 0.5 * u(5), 1, dt * 0.5 * u(3), 
        0, 0, 0, 0, 0, 0, dt * 0.5 * u(5), dt * 0.5 * u(4), -dt * 0.5 * u(3), 1;
    B << 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 
        (x(6) ^ 2 + x(7) ^ 2 - x(8) ^ 2 - x(9) ^ 2) * dt, 2(x(7) * x(8) - x(6) * x(9)) * dt, 2(x(7) * x(9) + x(6) * x(8)) * dt, 0, 0, 0, 
        2(x(7) * x(8) - x(6) * x(9)) * dt, (x(6) ^ 2 - x(7) ^ 2 + x(8) ^ 2 - x(9) ^ 2) * dt, 2(x(8) * x(9) + x(6) * x(7)) * dt, 0, 0, 0, 
        2(x(7) * x(9) + x(6) * x(8)) * dt, 2(x(8) * x(9) - x(6) * x(7)) * dt, (x(6) ^ 2 + x(7) ^ 2 - x(8) ^ 2 - x(9) ^ 2) * dt, 0, 0, 0, 
        0, 0, 0, -x(7), x(8), -x(9), 
        0, 0, 0, 1 + x(6), -x(9), x(8), 
        0, 0, 0, x(9), 1 + x(6), -x(7), 
        0, 0, 0, -x(8), x(7), 1 + x(6);
    C << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
    M << 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0;
    Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    // 予測ステップ
    x_hat = A * x + B * u;
    P = A * P * A.transpose() + B * M * B.transpose();
    K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();
    // フィルタリングステップ
    x_hat += K * (y - C * x_hat);
    P = (I - K * C) * P;
    x = x_hat;

    auto pose_ekf = geometry_msgs::msg::PoseStamped();
    pose_ekf.pose.position.x = x(0);
    pose_ekf.pose.position.y = x(1);
    pose_ekf.pose.position.z = x(2);
    pose_ekf.pose.orientation.w = x(6);
    pose_ekf.pose.orientation.x = x(7);
    pose_ekf.pose.orientation.y = x(8);
    pose_ekf.pose.orientation.z = x(9);

    Posepublisher_->publish(pose_ekf);
    return;
  }
} // namespace robotx_ekf