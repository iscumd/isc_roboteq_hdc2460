#include <cmath>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_broadcaster.h>
#include "encoder_odom/ekf_odom_pub.hpp"

EncoderOdom::EncoderOdom(rclcpp::NodeOptions options): Node("encoder_odom", options)
{
    wheel_radius = this->declare_parameter("wheel_radius", 0.165); //in meters
    wheel_seperation = this->declare_parameter("wheel_seperation", 0.47); //in meters
    gear_ratio = this->declare_parameter("gear_ratio", 18.0); // gear_ratio:1
    publish_odom_tf = this->declare_parameter("publish_odom_tf", false); // Whether or not to publish odom -> base_link transform

    //Sync the two encoder count values together. This should come in as wheel rpm
    rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_sensor_data;
    left_encoder_count_sub_.subscribe(this, "/robot/left_encoder_counts", rmw_qos_profile);
    right_encoder_count_sub_.subscribe(this, "/robot/right_encoder_counts", rmw_qos_profile);
    sync.reset(new Sync(MySyncPolicy(10), left_encoder_count_sub_, right_encoder_count_sub_));
    sync->registerCallback(std::bind(&EncoderOdom::encoder_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/robot/encoder_odom", 10
    );
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    currentTime = this->get_clock()->now();
    lastTime = this->get_clock()->now();
}

 
void EncoderOdom::encoder_callback(const std_msgs::msg::Int16::ConstSharedPtr &left_encoder, const std_msgs::msg::Int16::ConstSharedPtr &right_encoder){
  //Calculate how far weve moved based on the rpm values given adjusted for gear ratios  
  currentTime = this->get_clock()->now();

  //Generate wheel speed using RPM
  left_speed = wheel_radius * (left_encoder->data/gear_ratio) * 0.10472;
  right_speed = wheel_radius * (right_encoder->data/gear_ratio) * 0.10472;

  //Compute velocites
  delta_time = (currentTime - lastTime).seconds();
  velocity_x = (right_speed + left_speed) / 2;
  velocity_y = 0.0;
  velocity_th = (right_speed - left_speed) / wheel_seperation;

  //Compute change in x, y, theta position
  delta_x = std::cos(theta) * (velocity_x * delta_time);
  delta_y = std::sin(theta) * (velocity_x * delta_time);
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
  message_time = this->get_clock()->now();

  if(publish_odom_tf){
    //Send tf translation
    trans.header.stamp = message_time;
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_footprint";

    //Set x and y translations, wheel encoders wont tell us if were moving up
    //so z translation is set to zero
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;
    trans.transform.translation.z = 0.0;

    trans.transform.rotation = q_msg;

    tf_broadcaster_->sendTransform(trans);
  }

  //Publish odom message
  odom.header.stamp = message_time;
  odom.header.frame_id = "odom";

  //Set x and y position, wheel encoders wont tell us if were moving up
  //so z translation is set to zero
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
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