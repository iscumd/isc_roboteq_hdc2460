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
#include "std_msgs/msg/int16.hpp"

using std::string;

namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
public:
  explicit Roboteq(rclcpp::NodeOptions options);
  ~Roboteq();
  
private:
  // class atributes
  double left_speed;
  double right_speed;
  double speed_multipler;
  bool roboteq_is_connected;
  double max_current;
  unsigned long baud_rate;
  std::string usb_port;
  unsigned long baud;
  int chunk_size;
  bool flip_inputs = false;
  rclcpp::TimerBase::SharedPtr encoder_timer;

  // class methods
  void driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
  void enumerate_port();
  void connect();
  int constrainSpeed(double speed);
  bool send_Command(string command);
  void move();
  void set_current();
  void get_encoder_count();
  void recieve(std::string result);

   // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed;

};
}  // namespace Roboteq

#endif  //ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
