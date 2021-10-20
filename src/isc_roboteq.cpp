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

// open serial port
// setup USB protocol parameters
void Roboteq::connect()
{
	if(roboteqIsConnected){
		RCLCPP_WARN(this->get_logger(), "%s/n","Roboteq already connected");
		return;
	}
	if(port.empty()){
		RCLCPP_ERROR(this->get_logger(), "%s/n","Invalid or Empty Serial Port name");
		return;
	}
  
  port = "/dev/ttyUSB0";
  unsigned long baud = 9600;
  my_serial.setPort(port);
  my_serial.setBaudrate(baud);
  my_serial.setParity(serial::parity_even);
  my_serial.setStopbits(serial::stopbits_one);
  my_serial.setBytesize(serial::sevenbits);
  serial::Timeout time = serial::Timeout::simpleTimeout(10);
  my_serial.setTimeout(time);

  my_serial.open();

}

// put a speed govenor on how fast the motors can turn
unsigned char Roboteq::constrainSpeed(double speed)
{
  unsigned char temp_speed = fabs(speed);
	if(temp_speed > 127){
		temp_speed = 127;
	}
	return temp_speed;
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<Roboteq::Roboteq>(options);
  exec.add_node(lp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
