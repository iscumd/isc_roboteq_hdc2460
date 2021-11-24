#include <memory>
#include <functional>
#include "isc_roboteq/isc_roboteq.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

using std::string;
using std::stringstream;
using serial::Serial;
using serial::utils::SerialListener;
using serial::utils::BufferedFilterPtr;

string port;
serial::Serial serialPort;
serial::utils::SerialListener serialListener;
serial::utils::BufferedFilterPtr echoFilter;

namespace Roboteq
{
Roboteq::Roboteq(rclcpp::NodeOptions options)
: Node("roboteq", options)
{
  rightspeed = 0;
  leftspeed = 0;
  roboteqIsConnected = false;
  connect();
  Speed = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&Roboteq::driveCallBack, this, std::placeholders::_1));
  move();
}

// Disconnect the controller from serial 
 void Roboteq::disconnect()
 {
	if(serialListener.isListening()){
		serialListener.stopListening();
		RCLCPP_INFO(this->get_logger(), "%s","Listener closed");
  }
  port = "";
  roboteqIsConnected = false;
	RCLCPP_INFO(this->get_logger(), "%s","Port Closed");
}

// open serial port
// setup USB protocol parameters
void Roboteq::connect()
{

  port = "/dev/ttyUSB0";
  baud = 9600;
  
	if(roboteqIsConnected){
		RCLCPP_WARN(this->get_logger(), "%s","Roboteq already connected:)");
		return;
	}
	if(port.empty()){
		RCLCPP_ERROR(this->get_logger(), "%s","Invalid or Empty Serial Port name:(");
		return;
	}
	
  serialPort.setPort(port);
  serialPort.setBaudrate(baud);
  serialPort.open();
  serialListener.setChunkSize(64);
  serialListener.startListening(serialPort);
  roboteqIsConnected = true;
  
  if(serialPort.isOpen()){
  	RCLCPP_INFO(this->get_logger(), "%s","Roboteq Port Opened:)");
  }
  else{
    RCLCPP_WARN(this->get_logger(), "%s","Roboteq Port not Opened:(");
	}
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

  RCLCPP_INFO(this->get_logger(), "Roboteq: %s%lf%s%lf"," Right Wheel = ",rightspeed, " Left Wheel = ", leftspeed);
}

inline string stringFormat(const string &fmt, ...) {
  int size = 100;
  string str;
  va_list ap;
  while (1) {
	str.resize(size);
	va_start(ap, fmt);
	int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
	va_end(ap);
	if (n > -1 && n < size) {
	  str.resize(n);
	  return str;
	}
	if (n > -1) {
	  size = n + 1;
	} else {
	  size *= 2;
	}
  }
  return str;
}

inline bool isPlusOrMinus(const string &token) {
	if (token.find_first_of("+-") != string::npos) {
		return true;
	}
	return false;
}

// send command to controller over serial
bool Roboteq::send_Command(std::string command)
{
  BufferedFilterPtr echoFilter = serialListener.createBufferedFilter(SerialListener::exactly(command));
	serialPort.write(command+"\r");
	
	if (echoFilter->wait(50).empty()) {
		RCLCPP_ERROR(this->get_logger(), "%s","Failed to receive an echo from Roboteq:(");
		return false;
	}
	
	BufferedFilterPtr plusMinusFilter = serialListener.createBufferedFilter(isPlusOrMinus);
	std::string result = plusMinusFilter->wait(100);
	if(result != "+"){
		if(result == "-"){
			RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq rejected the command:(");
			return false;
		}
	}
	return true;
}
void Roboteq::move(){

	if(!roboteqIsConnected){
		RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq needs to connected first:(");
		return;
	}

	if(rightspeed < 0){
		send_Command(stringFormat("!a%.2X", constrainSpeed(rightspeed)));
	}
	else{
		send_Command(stringFormat("!A%.2X", constrainSpeed(rightspeed)));
	}

	if(leftspeed < 0){
		send_Command(stringFormat("!b%.2X", constrainSpeed(leftspeed)));
	}
	else{
		send_Command(stringFormat("!B%.2X", constrainSpeed(leftspeed)));
	}
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