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

#include "kaia_gazebo/self_drive_gazebo.hpp"
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

  this->declare_parameter("check.side_scan_angle", 30);
  this->declare_parameter("check.forward_distance", 0.7);
  this->declare_parameter("check.side_distance", 0.6);
  this->declare_parameter("check.turn_away_angle", 30);

  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    this->get_parameter("velocity.topic_name_pub").as_string(), 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->get_parameter("laser_scan.topic_name_sub").as_string(), rclcpp::SensorDataQoS(), \
    std::bind(&KaiaSelfDrive::scan_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->get_parameter("odometry.topic_name_sub").as_string(), 10,
    std::bind(&KaiaSelfDrive::odom_callback, this, std::placeholders::_1));

  update_timer_ = this->create_wall_timer(10ms, std::bind(&KaiaSelfDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Self-drive Gazebo simulation node has been initialized");
}

KaiaSelfDrive::~KaiaSelfDrive()
{
  RCLCPP_INFO(this->get_logger(),
    "Self-drive Gazebo simulation node has been terminated. Robot has been stopped.");
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
  int side_scan_angle = (uint16_t)this->get_parameter("check.side_scan_angle").as_int();
  uint16_t scan_angle[3] = {0, (uint16_t)side_scan_angle, (uint16_t)(360-side_scan_angle)};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
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
  static uint8_t kaia_state_num = GET_KAIA_DIRECTION;

  if (!keepRunning) {
    update_cmd_vel(0.0, 0.0);
    rclcpp::shutdown();
  }

  switch (kaia_state_num) {
    case GET_KAIA_DIRECTION:

      if (scan_data_[CENTER] > this->get_parameter("check.forward_distance").as_double()) {
        double check_side_dist = this->get_parameter("check.side_distance").as_double();

        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          kaia_state_num = KAIA_RIGHT_TURN;
        } else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          kaia_state_num = KAIA_LEFT_TURN;
        } else {
          kaia_state_num = KAIA_DRIVE_FORWARD;
        }
      } else {
        prev_robot_pose_ = robot_pose_;
        kaia_state_num = KAIA_RIGHT_TURN;
      }
      break;

    case KAIA_DRIVE_FORWARD:
      update_cmd_vel(this->get_parameter("velocity.linear").as_double(), 0.0);
      kaia_state_num = GET_KAIA_DIRECTION;
      break;

    case KAIA_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >=
        this->get_parameter("check.turn_away_angle").as_int() * DEG2RAD) {
        kaia_state_num = GET_KAIA_DIRECTION;
      } else {
        update_cmd_vel(0.0, -1 * this->get_parameter("velocity.angular").as_double());
      }
      break;

    case KAIA_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >=
        this->get_parameter("check.turn_away_angle").as_int() * DEG2RAD) {
        kaia_state_num = GET_KAIA_DIRECTION;
      } else {
        update_cmd_vel(0.0, this->get_parameter("velocity.angular").as_double());
      }
      break;

    default:
      kaia_state_num = GET_KAIA_DIRECTION;
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
