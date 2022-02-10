#include <memory>
#include <functional>
#include <vector>
#include <regex>
#include "std_msgs/msg/string.hpp"
#include "isc_roboteq/isc_roboteq.hpp"
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

using std::string;
using std::stringstream;
using serial::Serial;
using serial::utils::SerialListener;
using serial::utils::BufferedFilterPtr;

serial::Serial serialPort;
serial::utils::SerialListener serialListener;

namespace Roboteq
{
Roboteq::Roboteq(rclcpp::NodeOptions options)
: Node("roboteq", options)
{
  max_current = this->declare_parameter("max_current", 40.0);
  baud_rate = this->declare_parameter("baud_rate", 9600);
  chunk_size = this->declare_parameter("chunk_size", 64);
  
  right_speed = 0;
  left_speed = 0;
  roboteq_is_connected = false;
  connect();
   
  speed = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1,
    std::bind(&Roboteq::driveCallBack, this, std::placeholders::_1));
}

/*
find the port the roboteq is connected to
by regexing all open ports. Once found, assign 
the roboteqs /dev path to the usb_port attribute
*/
void Roboteq::enumerate_port()
{
  std::regex manufacture("(Prolific)(.*)");
  std::vector<serial::PortInfo> devices = serial::list_ports();
  std::vector<serial::PortInfo>::iterator ports = devices.begin();
  while(ports != devices.end())
  {
    serial::PortInfo found_port = *ports++;
    if(std::regex_match(found_port.description.c_str(), manufacture))
    {
      usb_port = found_port.port.c_str();
    }
  }
}

/* 
open serial port
open serial listener
setup USB protocol parameters
*/
void Roboteq::connect()
{
  enumerate_port();
  if(roboteq_is_connected){
    RCLCPP_WARN(this->get_logger(), "%s","Roboteq already connected:)");
    return;
  }
  if(usb_port.empty()){
    RCLCPP_ERROR(this->get_logger(), "%s","Invalid or Empty Serial Port name:(");
    return;
  }
    
  RCLCPP_INFO(this->get_logger(), "%s%lf","The Set Max Current is ", max_current);
  RCLCPP_INFO(this->get_logger(), "%s%s","The Set Port to open is ", usb_port.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%lu","The Set Baudrate is ", baud_rate);  
  RCLCPP_INFO(this->get_logger(), "%s%d","The Set Chunksize is ", chunk_size);

  serial::Timeout to = serial::Timeout::simpleTimeout(10);
  serialPort.setPort(usb_port);
  serialPort.setBaudrate(baud_rate);
  serialPort.setParity(serial::parity_even);
  serialPort.setStopbits(serial::stopbits_one);
  serialPort.setBytesize(serial::sevenbits);
  serialPort.setTimeout(to);
  serialPort.open();
  
  serialListener.setChunkSize(chunk_size);
  serialListener.setDefaultHandler(std::bind(&Roboteq::recieve, this, std::placeholders::_1));
  serialListener.startListening(serialPort);
  
  roboteq_is_connected = true;
  
  if(serialPort.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "%s","Roboteq Port Opened:)");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "%s","Roboteq Port not Opened:(");
  }
}

// put a speed govenor on how fast the motors can turn
unsigned char Roboteq::constrainSpeed(double speed)
{
  unsigned char temp_speed = fabs(speed);
    if(temp_speed > 127)
    {
        temp_speed = 127;
    }
  return temp_speed;
}

/* 
called every time it recives a Twist message
calculates the left and right motors wheel speeds
and then multiplies it by the speed multipler 
*/
void Roboteq::driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_multipler = 60.0;
  left_speed = (msg->linear.x - msg->angular.z) * speed_multipler;
  right_speed = (msg->linear.x + msg->angular.z) * speed_multipler;
  move();
  RCLCPP_INFO(this->get_logger(), "Roboteq: %s%lf%s%lf"," Left Wheel = ", left_speed, " Right Wheel = ", right_speed);
}

// format command string before it is sent to controller
inline string stringFormat(const string &fmt, ...) 
{
  int size = 100;
  string str;
  va_list ap;
  while (1) 
  {
    str.resize(size);
    va_start(ap, fmt);
    int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
    va_end(ap);
    if (n > -1 && n < size) 
    {
      str.resize(n);
      return str;
    }
    if (n > -1) 
    {
      size = n + 1;
    } else {
      size *= 2;
    }
  }
  return str;
}

/*
utility function to check the response from controller
+ equals accepted command 
- equals failed command
*/
inline bool isPlusOrMinus(const string &token) 
{
  if (token.find_first_of("+-") != string::npos) 
  {
    return true;
  }
  return false;
}

 
// send command to controller over serial
void Roboteq::send_Command(std::string command)
{
  serialPort.write(command+"\r");
}

/*
call back function that is called when roboteq 
echos back to the node
*/
void Roboteq::recieve(std::string result)
{
  if (result.empty()) 
  { 
    RCLCPP_ERROR(this->get_logger(), "%s","Failed to receive an echo from Roboteq:(");
  }

  if(result != "+")
  {
    if(result == "-")
    {
      RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq rejected the command:(");
    }
    RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq neither rejected or accepted the command:(");
  }
}

/* 
move the robot by sending wheel speeds to send command function
constrain wheel speeds before sent to controller
TODO implement current watch dog
*/
void Roboteq::move()
{
  if(!roboteq_is_connected){
    RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq needs to connected first:(");
    return;
  }
  if(right_speed < 0){
    send_Command(stringFormat("!a%.2X", constrainSpeed(right_speed)));
  }
  else{
    send_Command(stringFormat("!A%.2X", constrainSpeed(right_speed)));
  }
  if(left_speed < 0){
    send_Command(stringFormat("!b%.2X", constrainSpeed(left_speed)));
  }
  else{
    send_Command(stringFormat("!B%.2X", constrainSpeed(left_speed)));
  }
}

// Disconnect the controller from serial 
Roboteq::~Roboteq()
 {
  if(serialListener.isListening()){
    serialListener.stopListening();
    RCLCPP_INFO(this->get_logger(), "%s","Listener closed:)");
  }

  roboteq_is_connected = false;
  RCLCPP_INFO(this->get_logger(), "%s","Port Closed:)");
}
} // end of namespace Roboteq

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
