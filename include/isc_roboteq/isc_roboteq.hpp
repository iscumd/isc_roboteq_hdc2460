#ifndef ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
#define ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_

#include <math.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <isc_roboteq/utils.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"

namespace Roboteq {
class Roboteq : public rclcpp::Node {
 public:
  explicit Roboteq(rclcpp::NodeOptions options);
  ~Roboteq();

 private:
  // class atributes
  float left_speed{};
  float right_speed{};
  float min_speed{};
  float max_speed{};
  float speed_multipler{};
  bool roboteq_is_connected{};
  unsigned long baud_rate{};
  std::string usb_port{};
  int chunk_size{};

  // class methods

  /**
   * @brief takes in the cmd velocity and converts it to left/right wheel speeds
   * @param msg geomtry_msgs::msg::Twist::SharedPtr
   */
  void driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief regexs all usb ports to find the roboteq
   */
  void enumerate_port();

  /**
   * @brief set up all serial/serial listener protocols and connect to the ports
   */
  void connect();

  /**
   * @brief sends the command to the roboteq
   * @param command string
   * @return true if the roboteq accepted, else false
   */
  bool send_Command(std::string command);

  /**
   * @brief Main Function that formats the commands, constrain the speed and
   * sends it to move
   */
  void move();

  /**
   * @brief dynamic param update callback. updates params every 1000ms
   */
  void update_params();

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed;

  // wall timer
  rclcpp::TimerBase::SharedPtr param_update_timer;
};
}  // namespace Roboteq

#endif  // ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
