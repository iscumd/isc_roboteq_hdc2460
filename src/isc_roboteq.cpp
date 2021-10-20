#include <memory>
#include <functional>

#include "isc_roboteq/isc_roboteq.hpp"
#include "std_msgs/msg/string.hpp"

namespace Roboteq
{
Roboteq::Roboteq(rclcpp::NodeOptions options)
: Node("roboteq", options)
{
  rightspeed = 0;
  leftspeed = 0;
  roboteqIsConnected = false;

  Speed = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&Roboteq::driveCallBack, this, std::placeholders::_1));
}
// called every time it recives a Twist message
void Roboteq::driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speedMultipler = 127;
  rightspeed = (msg->linear.x - msg->angular.z) * speedMultipler;
  leftspeed = (msg->linear.x - msg->angular.z) * speedMultipler;

  RCLCPP_INFO(this->get_logger(), "Roboteq: '%s%lf%s%lf/n'"," Right Wheel = ",rightspeed, " Left Wheel = ", leftspeed);
}
}

int main()
{

  return 0;
}
