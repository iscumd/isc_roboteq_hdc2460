#include <cmath>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "encoder_odom/ekf_odom_pub.hpp"

EncoderOdom::EncoderOdom(rclcpp::NodeOptions options): Node("encoder_odom", options)
{
    wheel_radius = this->declare_parameter("wheel_radius", 0.33); //in meters
    wheel_seperation = this->declare_parameter("wheel_seperation", 0.17);

    //Sync the two encoder count values together. This should come in as wheel rpm
    rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_sensor_data;
    left_encoder_count_sub_.subscribe(this, "/robot/left_encoder_counts", rmw_qos_profile);
    right_encoder_count_sub_.subscribe(this, "/robot/right_encoder_counts", rmw_qos_profile);
    sync.reset(new Sync(MySyncPolicy(10), left_encoder_count_sub_, right_encoder_count_sub_));
    sync->registerCallback(std::bind(&EncoderOdom::encoder_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/robot/encoder_odom", 10
    );
    currentTime = this->get_clock()->now();
    lastTime = this->get_clock()->now();
}

 
void EncoderOdom::encoder_callback(const std_msgs::msg::Int16::ConstSharedPtr &left_encoder, const std_msgs::msg::Int16::ConstSharedPtr &right_encoder){
  //Calculate how far weve moved based on the encoder values given
  //We need to get the distance that the robot has traveled between the last
  //Encoder message and this encoder message. To do this the original program used
  //A variable that would store the last message value recieved and use that to
  //calculate the difference in distance. I could use a similar function so that
  //I could keep track of it.
  
  currentTime = this->get_clock()->now();
  //Generate wheel velocities using RPM
  left_speed = wheel_radius * left_encoder->data * 0.10472;
  right_speed = wheel_radius * right_encoder->data * 0.10472;

  //Compute velocites
  delta_time = (currentTime - lastTime).seconds();
  velocity_x = (right_speed + left_speed) / 2;
  velocity_y = 0.0;
  velocity_th = (right_speed - left_speed) / wheel_seperation;

  //Compute change in x, y, theta position
  delta_x = (velocity_x * std::cos(theta)) * delta_time;
  delta_y = (velocity_x * std::sin(theta)) * delta_time;
  delta_theta = velocity_th * delta_time;

  //Add changes in position to overall position
  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  //Update last time for next iteration
  lastTime = currentTime;

  publish_quat();
  
}
 
void EncoderOdom::publish_quat() {

  //Fill odom msg based on data from the callback and publish
  q.setRPY(0,0,theta);
  tf2::convert(q, q_msg);
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.x = x;
  //If we are moving up something has gone horribly wrong
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = q_msg;

  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x = velocity_x;
  odom.twist.twist.linear.y = velocity_y;
  odom.twist.twist.angular.z = velocity_th;
  
  odom_data_pub_->publish(odom);

}
 
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto encoder_odom_node = std::make_shared<EncoderOdom>(options);
  exec.add_node(encoder_odom_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;

}