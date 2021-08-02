// Copyright (c) 2020 OUXT Polaris
//
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




#include <robotx_ekf/ekf_component.hpp>
#include <memory>
#include <vector>

namespace robotx_ekf
{
EKFComponent::EKFComponent(const rclcpp::NodeOptions & options)
: Node("robotx_ekf", options)
{
  
  GPSsubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "GPS_Topic", 10, std::bind(&EKFComponent::GPStopic_callback, this, std::placeholders::_1));

  IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "IMU_Topic", 10, std::bind(&EKFComponent::IMUtopic_callback, this, std::placeholders::_1));

  Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Pub_Pose", 10);

  timer_ = this->create_wall_timer(
    500ms, std::bind(&EKFComponent::timer_callback, this));
}
void EKFComponent::GPStopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
    {
      y(0) = msg -> pose.pose.position.x;
      y(1) = msg -> pose.pose.position.y;
      y(2) = msg -> pose.pose.position.z;
        
    }

void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu msg) const
{
  u(0) = msg-> angular_velocity.x;
  u(1) = msg-> angular_velocity.y;
  u(2) = msg-> angular_velocity.z;

  u(3) = msg-> linear_acceleration.x;
  u(4) = msg-> linear_acceleration.y;
  u(5) = msg-> linear_acceleration.z;

}

void EKFComponent::timer_callback()
{
  auto message = geometry_msgs::msg::PoseStamped();
  message.pose.position.x = x(0);
  message.pose.position.y = x(1);
  message.pose.position.y = x(2);

  Posepublisher_->publish(message);
}
}  // namespace robotx_ekf
