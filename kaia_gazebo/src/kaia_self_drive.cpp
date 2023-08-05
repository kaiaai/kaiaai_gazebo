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

//https://github.com/aarsht7/teleop_cpp_ros2/blob/main/src/teleop_cpp_ros2.cpp

// ACKNOWLEDGEMENT: This code is based on ROBOTIS Turtlebot3 project

#include "kaia_gazebo/kaia_self_drive.hpp"
#include <memory>
#include <unistd.h>
#include <termios.h>

using namespace std::chrono_literals;

int getch() {
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

KaiaSelfDrive::KaiaSelfDrive()
: Node("kaia_self_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  robot_paused_ = false;
  cmd_vel_last_linear_ = 0.0;
  cmd_vel_last_angular_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), \
    std::bind(&KaiaSelfDrive::scan_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&KaiaSelfDrive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&KaiaSelfDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Kaia bot simulation node has been initialized");
  RCLCPP_INFO(this->get_logger(), "Press SPACE KEY to pause/unpause the bot, CTRL-C to exit");
}

KaiaSelfDrive::~KaiaSelfDrive()
{
  RCLCPP_INFO(this->get_logger(), "Kaia bot simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
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
  uint16_t scan_angle[3] = {0, 30, 330};

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
  cmd_vel_last_linear_ = linear;
  cmd_vel_last_angular_ = angular;

  publish_cmd_vel(linear, angular);
}

void KaiaSelfDrive::publish_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void KaiaSelfDrive::update_callback()
{
  static uint8_t kaia_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.7;
  double check_side_dist = 0.6;

  char key = getch();

  if (key == ' ') {
    robot_paused_ = not robot_paused_;

    if (robot_paused_) {
      publish_cmd_vel(0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "Robot paused");
    } else {
      publish_cmd_vel(cmd_vel_last_linear_, cmd_vel_last_angular_);
      RCLCPP_INFO(this->get_logger(), "Robot un-paused");
    }
  }

  if (robot_paused_)
    return;

  switch (kaia_state_num) {
    case GET_KAIA_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist) {
        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          kaia_state_num = KAIA_RIGHT_TURN;
        } else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          kaia_state_num = KAIA_LEFT_TURN;
        } else {
          kaia_state_num = KAIA_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist) {
        prev_robot_pose_ = robot_pose_;
        kaia_state_num = KAIA_RIGHT_TURN;
      }
      break;

    case KAIA_DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      kaia_state_num = GET_KAIA_DIRECTION;
      break;

    case KAIA_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        kaia_state_num = GET_KAIA_DIRECTION;
      } else {
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    case KAIA_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        kaia_state_num = GET_KAIA_DIRECTION;
      } else {
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      }
      break;

    default:
      kaia_state_num = GET_KAIA_DIRECTION;
      break;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KaiaSelfDrive>());
  rclcpp::shutdown();

  return 0;
}
