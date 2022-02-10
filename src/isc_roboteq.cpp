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
serial::utils::BufferedFilterPtr echoFilter;

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
    
  encoder_timer = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&Roboteq::get_encoder_count, this));
}

/*
find the port the roboteq is connected to
by regexing all open ports. Once found, assign 
the roboteqs /dev path to the usb_port attribute
*/
void Roboteq::enumerate_port()
{
  std::regex manufacture("(Prolific | Roboteq)(.*)");
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
int Roboteq::constrainSpeed(double speed)
{
	if(abs(speed) > 1000){
		if(speed > 0){
			speed = 1000;
	}
	else{
		speed = -1000;
	}		
	}
	return speed;
}

/* 
called every time it recives a Twist message
calculates the left and right motors wheel speeds
and then multiplies it by the speed multipler 
*/
void Roboteq::driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_multipler = 450.0;
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

/* 
send command to controller over serial
and listen to Roboteq for response
*/
bool Roboteq::send_Command(std::string command)
{
  BufferedFilterPtr echoFilter = serialListener.createBufferedFilter(SerialListener::exactly(command));
  serialPort.write(command+"\r");
  if (echoFilter->wait(50).empty()) 
  { 
    RCLCPP_ERROR(this->get_logger(), "%s","Failed to receive an echo from Roboteq:(");
    return false;
  }

  BufferedFilterPtr plusMinusFilter = serialListener.createBufferedFilter(isPlusOrMinus);
  std::string result = plusMinusFilter->wait(100);

  if(result != "+")
  {
    if(result == "-")
    {
      RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq rejected the command:(");
      return false;
    }
    RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq neither rejected or accepted the command:(");
    return false;
  }
  return true;
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
	send_Command(stringFormat("!G 1 %d", (flip_inputs ? -1 : 1) * constrainSpeed(right_speed)));
	send_Command(stringFormat("!G 2 %d", (flip_inputs ? -1 : 1) * constrainSpeed(left_speed)));
}

/* 
ask the roboteq to return the encoder count
for each channel 
*/
void Roboteq::get_encoder_count()
{
	serialPort.write("?CR 1");
	serialPort.write("?CR 2");
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
