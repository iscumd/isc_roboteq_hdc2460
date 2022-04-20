#ifndef ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
#define ISC_ROBOTEQ__ISC_ROBOTEQ_

#include <memory>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <isc_roboteq/utils.hpp>

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
  float left_speed{};
  float right_speed{};
  float min_speed{};
  float max_speed{};
  float speed_multipler{};
  float gear_reduction{};
  bool roboteq_is_connected{};
  unsigned long baud_rate{};
  int sign{};
  std::string usb_port{};
  int chunk_size{};
  bool flip_inputs = false;
  bool left_encoder_value_recieved = false;
  bool has_encoders{};
  std::string echo_back;

  // class methods
  /**
  * @brief takes in the cmd velocity and converts it to left/right wheel speeds
  * @param msg geomtry_msgs::msg::Twist::SharedPtr
  */
  void driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
  * @brief polling callback function to get wheel encoder ticks 
  */
  void encoderCallBack();

  /**
  * @brief default handler for serial utils that handles what the roboteq
  * echos back
  * @param result std::string
  */
  void recieve(std::string result);

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
  void send_Command(string command);

  /**
  * @brief Main Function that formats the commands, constrain the speed and sends it to move
  */
  void move();

  float clamp_speed(float speed);

  /**
  * @brief dynamic param update callback. updates params every 1000ms
  */
  void update_params();

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed;

  // publishers
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr right_encoder_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr left_encoder_count_pub_;

  // encoder timer
  rclcpp::TimerBase::SharedPtr timer;

  // wall timer
  rclcpp::TimerBase::SharedPtr param_update_timer;
};
}  // namespace Roboteq

#endif  //ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
