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
  bool roboteqIsConnected;
  double Max_Current;
  unsigned long Baudrate;
  std::string USB_Port;
  unsigned long baud;
  int ChunkSize;

  // class methods
  void driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
  void enumerate_port();
  void disconnect();
  void connect();
  unsigned char constrainSpeed(double speed);
  bool send_Command(string command);
  void move();
  bool current_watchdog();

   // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Speed;

};
}  // namespace Roboteq

#endif  //ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
