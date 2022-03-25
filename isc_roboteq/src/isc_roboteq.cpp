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
  min_speed = this->declare_parameter("min_speed", 0);
  max_speed = this->declare_parameter("max_speed", 1000);
  speed_multipler = this->declare_parameter("speed_multipler", 450);
  has_encoders = this->declare_parameter("has_encoders", true);
  gear_reduction = this->declare_parameter("gear_reduction", 1.0);

  right_speed = 0;
  left_speed = 0;
  roboteq_is_connected = false;
  connect();
   
  speed = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1,
    std::bind(&Roboteq::driveCallBack, this, std::placeholders::_1));

  encoder_count_pub_ = this->create_publisher<roboteq_msgs::msg::EncoderCounts>(
        "/robot/encoder_counts", 10
  );

  using namespace std::chrono_literals;
  param_update_timer = this->create_wall_timer(
    1000ms, std::bind(&Roboteq::update_params, this));

  timer = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&Roboteq::encoderCallBack, this));
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
  std::regex manufacture("(Roboteq)(.*)");
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

void Roboteq::driveCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  left_speed = (msg->linear.x - msg->angular.z) * speed_multipler;
  right_speed = (msg->linear.x + msg->angular.z) * speed_multipler;
  move();
  RCLCPP_INFO(this->get_logger(), "Roboteq: %s%lf%s%lf"," Left Wheel = ", left_speed, " Right Wheel = ", right_speed);
}

void Roboteq::recieve(std::string result)
{
  roboteq_msgs::msg::EncoderCounts counts;
  int count;
  if (result.empty()) 
  { 
    RCLCPP_ERROR(this->get_logger(), "%s","Failed to receive an echo from Roboteq:(");
  }

  //If encoders are present, check the recieved message to see if its an encoder message
	if (has_encoders && result.substr(0, 3) == "CR=") {
    //Encoder values come in one at a time left first then right so keep track of which one youve
    //Aquired using the variable
		if (!left_encoder_value_recieved) {
			counts.left_encoder = std::stoi(result.substr(3))/gear_reduction;
			left_encoder_value_recieved = true;
		}
		else {
			counts.right_encoder = std::stoi(result.substr(3))/gear_reduction;
			left_encoder_value_recieved = false;

      //Publish after recieving both encoder values
      encoder_count_pub_->publish(counts);
		}
	}

}

void Roboteq::send_Command(std::string command)
{
serialPort.write(command+"\r");
}

void Roboteq::encoderCallBack()
{
  if(has_encoders){
    send_Command("?CR 1");
    send_Command("?CR 2");
  }
}

void Roboteq::move()
{
  if(!roboteq_is_connected){
    RCLCPP_ERROR(this->get_logger(), "%s","The Roboteq needs to connected first:(");
    return;
  }
	send_Command(stringFormat("!G 1 %d", (flip_inputs ? -1 : 1) * std::clamp(right_speed, min_speed, max_speed)));
	send_Command(stringFormat("!G 2 %d", (flip_inputs ? -1 : 1) * std::clamp(left_speed, min_speed, max_speed)));
}
 
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
