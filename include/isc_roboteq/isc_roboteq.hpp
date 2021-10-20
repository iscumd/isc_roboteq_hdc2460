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
#include <geometry_msgs/Twist.h>
#include "serial/serial.h"
namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
public:
  explicit Roboteq(rclcpp::NodeOptions options);

private:
   void driveModeCallback(const geometry_msgs::Twist::ConstPtr msg);
   unsigned char constrainSpeed(double speed);

};
}  // namespace Roboteq

#endif  //ISC_ROBOTEQ__ISC_ROBOTEQ_HPP_
