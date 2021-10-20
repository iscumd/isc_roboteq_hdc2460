#ifndef ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
#define ISC_ROBOTEQ__ISC_ROBOTEQ_

#include <memory>

#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

using std::string;

namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
public:
  explicit Roboteq(rclcpp::NodeOptions options);

private:
  // class atributes
  double leftspeed;
  double rightspeed;
  float speedMultipler;
  std::string port;
  serial::Serial my_serial;
  bool roboteqIsConnected;

  // class methods
  void driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
  void disconnect();
  void connect();
  unsigned char constrainSpeed(double speed);
  bool send_command(string command);

   // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Speed;

};
}  // namespace Roboteq

#endif  //ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
