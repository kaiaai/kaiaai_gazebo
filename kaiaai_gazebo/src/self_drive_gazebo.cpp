// Copyright 2023 REMAKE.AI
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

#include "kaiaai_gazebo/self_drive_gazebo.hpp"
#include <memory>
#include <signal.h>

using namespace std::chrono_literals;

static volatile bool keepRunning = true;

KaiaSelfDrive::KaiaSelfDrive()
: Node("self_drive_gazebo_node")
{
  this->declare_parameter("velocity.topic_name_pub", "cmd_vel");
  this->declare_parameter("velocity.linear", 0.3);
  this->declare_parameter("velocity.angular", 1.5);

  this->declare_parameter("laser_scan.topic_name_sub", "scan");

  this->declare_parameter("odometry.topic_name_sub", "odom");

  this->declare_parameter("check.angle", 30);
  this->declare_parameter("check.distance", 0.5);

  obstacle_angle_left_ = 180;
  obstacle_angle_right_ = 180;
  robot_pose_ = 0.0;

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    this->get_parameter("velocity.topic_name_pub").as_string(), 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->get_parameter("laser_scan.topic_name_sub").as_string(), rclcpp::SensorDataQoS(), \
    std::bind(&KaiaSelfDrive::scan_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->get_parameter("odometry.topic_name_sub").as_string(), 10,
    std::bind(&KaiaSelfDrive::odom_callback, this, std::placeholders::_1));

  update_timer_ = this->create_wall_timer(10ms, std::bind(&KaiaSelfDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Robot is now moving");
}

KaiaSelfDrive::~KaiaSelfDrive()
{
  RCLCPP_INFO(this->get_logger(),
    "Robot has been stopped");
}

void KaiaSelfDrive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void KaiaSelfDrive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double check_distance = this->get_parameter("check.distance").as_double();

  for (obstacle_angle_left_ = 0; obstacle_angle_left_ < 180; obstacle_angle_left_++) {
    if (!std::isinf(msg->ranges.at(obstacle_angle_left_))) {
      double distance = msg->ranges.at(obstacle_angle_left_);
      if (distance < check_distance)
        break;
    }
  }

  for (obstacle_angle_right_ = 359; obstacle_angle_right_ >= 180; obstacle_angle_right_--) {
    if (!std::isinf(msg->ranges.at(obstacle_angle_right_))) {
      double distance = msg->ranges.at(obstacle_angle_right_);
      if (distance < check_distance)
        break;
    }
  }
  obstacle_angle_right_ = 360 - obstacle_angle_right_;

}

void KaiaSelfDrive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;

  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

void KaiaSelfDrive::update_callback()
{
  if (!keepRunning) {
    update_cmd_vel(0.0, 0.0);
    rclcpp::shutdown();
    return;
  }

  int check_angle = this->get_parameter("check.angle").as_int();
  int gap_angle = obstacle_angle_right_ + obstacle_angle_left_;
  uint8_t kaiaai_state = KAIAAI_RIGHT_TURN;

  if (obstacle_angle_right_ > check_angle && obstacle_angle_left_ > check_angle)
    kaiaai_state = KAIAAI_DRIVE_FORWARD;
  else if (obstacle_angle_right_ > check_angle && gap_angle > 2*check_angle)
    kaiaai_state = KAIAAI_RIGHT_TURN;
  else if (obstacle_angle_left_ > check_angle && gap_angle > 2*check_angle)
    kaiaai_state = KAIAAI_LEFT_TURN;
  else if (obstacle_angle_left_ > check_angle && gap_angle > 2*check_angle)
    kaiaai_state = KAIAAI_RIGHT_TURN;

  switch (kaiaai_state) {
    case KAIAAI_DRIVE_FORWARD:
      update_cmd_vel(this->get_parameter("velocity.linear").as_double(), 0.0);
      break;

    case KAIAAI_RIGHT_TURN:
      update_cmd_vel(0.0, -1 * this->get_parameter("velocity.angular").as_double());
      break;

    case KAIAAI_LEFT_TURN:
      update_cmd_vel(0.0, this->get_parameter("velocity.angular").as_double());
      break;
  }
}

void intHandler(int) {
  keepRunning = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, intHandler);
  rclcpp::spin(std::make_shared<KaiaSelfDrive>());
  rclcpp::shutdown();

  return 0;
}
