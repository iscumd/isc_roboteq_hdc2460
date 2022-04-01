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
  baud_rate = this->declare_parameter("baud_rate", 9600);
  chunk_size = this->declare_parameter("chunk_size", 64);
  min_speed = this->declare_parameter("min_speed", -127.0);
  max_speed = this->declare_parameter("max_speed", 127.0);
  speed_multipler = this->declare_parameter("speed_multipler", 60.0);

  right_speed = 0;
  left_speed = 0;
  roboteq_is_connected = false;
  connect();
   
  speed = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1,
    std::bind(&Roboteq::driveCallBack, this, std::placeholders::_1));

  using namespace std::chrono_literals;
  param_update_timer = this->create_wall_timer(
    1000ms, std::bind(&Roboteq::update_params, this));
}

void Roboteq::update_params()
{
  this->get_parameter("baud_rate", baud_rate);
  this->get_parameter("chunk_size", chunk_size);
  this->get_parameter("min_speed", min_speed);
  this->get_parameter("max_speed", max_speed);
  this->get_parameter("speed_multipler", speed_multipler);
}

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

void Roboteq::driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  left_speed = (msg->linear.x - msg->angular.z) * speed_multipler;
  right_speed = (msg->linear.x + msg->angular.z) * speed_multipler;
  move();
  RCLCPP_INFO(this->get_logger(), "Roboteq: %s%lf%s%lf"," Left Wheel = ", left_speed, " Right Wheel = ", right_speed);
}

bool Roboteq::send_Command(std::string command)
{
  BufferedFilterPtr echoFilter = serialListener.createBufferedFilter(SerialListener::exactly(command));
  serialPort.write(command+"\r");
  if (echoFilter->wait(50).empty()) // wait 50ms for a response
  { 
    RCLCPP_ERROR(this->get_logger(), "%s","Failed to receive an echo from Roboteq:(");
    return false;
  }
  return true;
}

void Roboteq::move()
{
  if(!roboteq_is_connected){
    RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq needs to connected first:(");
    return;
  }

  // clamp speed, format, and send it of to the roboteq
  if(right_speed < 0){
    send_Command(stringFormat("!a%.2X", std::clamp(right_speed, min_speed, max_speed)));
  }
  else{
    send_Command(stringFormat("!A%.2X", std::clamp(right_speed, min_speed, max_speed)));
  }
  if(left_speed < 0){
    send_Command(stringFormat("!b%.2X", std::clamp(left_speed, min_speed, max_speed)));
  }
  else{
    send_Command(stringFormat("!B%.2X", std::clamp(left_speed, min_speed, max_speed)));
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
