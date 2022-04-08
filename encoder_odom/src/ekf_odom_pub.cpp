/*
 * Automatic Addison
 * Date: May 20, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
#include <cmath>
#include "encoder_odom/ekf_odom_pub.hpp"

EncoderOdom::EncoderOdom(rclcpp::NodeOptions options): Node("encoder_odom", options)
{
    ticks_per_revolution = this->declare_parameter("ticks_per_revolution", 620.0);
    wheel_radius = this->declare_parameter("wheel_radius", 0.33); //in meters
    wheel_base = this->declare_parameter("wheel_base", 0.17); //in meters

    //Determine ticks per meter based on ticks per rev and wheel radius
    ticks_per_meter = (1.0/((2*M_PI)*wheel_radius)) * ticks_per_revolution;

    // Set the data fields of the odometry message
    quatOdom.header.frame_id = "odom";
    quatOdom.pose.pose.position.z = 0;
    quatOdom.pose.pose.orientation.x = 0;
    quatOdom.pose.pose.orientation.y = 0;
    quatOdom.twist.twist.linear.x = 0;
    quatOdom.twist.twist.linear.y = 0;
    quatOdom.twist.twist.linear.z = 0;
    quatOdom.twist.twist.angular.x = 0;
    quatOdom.twist.twist.angular.y = 0;
    quatOdom.twist.twist.angular.z = 0;
    quatOdomOld.pose.pose.position.x = initialX;
    quatOdomOld.pose.pose.position.y = initialY;
    qOld.setRPY(0, 0, initialTheta);


    rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_sensor_data;
    left_encoder_count_sub_.subscribe(this, "/robot/left_encoder_counts", rmw_qos_profile);
    right_encoder_count_sub_.subscribe(this, "/robot/right_encoder_counts", rmw_qos_profile);
    sync.reset(new Sync(MySyncPolicy(10), left_encoder_count_sub_, right_encoder_count_sub_));
    sync->registerCallback(std::bind(&EncoderOdom::encoder_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    odom_data_pub_quat_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/robot/encoder_odom", 10
    );
}

 
void EncoderOdom::encoder_callback(const std_msgs::msg::Int16::ConstSharedPtr &left_encoder, const std_msgs::msg::Int16::ConstSharedPtr &right_encoder){
  //Calculate how far weve moved based on the encoder values given
  //We need to get the distance that the robot has traveled between the last
  //Encoder message and this encoder message. To do this the original program used
  //A variable that would store the last message value recieved and use that to
  //calculate the difference in distance. I could use a similar function so that
  //I could keep track of it.
  
  //Get the difference between the last encoder values and the current values
  if(left_encoder->data == lastCountL){
    //If we are continuously moving in a straight line then distance traveled is the same
    left_ticks = left_encoder->data;
  }
  else if(right_encoder->data == lastCountR){
    //If we are continuously moving in a straight line then distance traveled is the same
    right_ticks = right_encoder->data;
  }
  else{
    left_ticks = left_encoder->data - lastCountL;
    right_ticks = right_encoder->data - lastCountR;
  }

  //Convert that difference to meters traveled
  distanceLeft = left_ticks/ticks_per_meter;
  distanceRight = right_ticks/ticks_per_meter;

  //Store the current value into the last counts to be used again later
  lastCountL = left_encoder->data;
  lastCountR = right_encoder->data;
  
  //Update and publish odom
  publish_quat();
}
 
// Publish a nav_msgs::Odometry message in quaternion format
void EncoderOdom::publish_quat() {
 
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
   
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = std::asin((distanceRight-distanceLeft)/wheel_base);
 
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + qOld.z();
     
  if (avgAngle > M_PI) {
    avgAngle -= 2*M_PI;
  }
  else if (avgAngle < -M_PI) {
    avgAngle += 2*M_PI;
  }
 
  // Calculate the new pose (x, y, and theta)
  quatOdom.pose.pose.position.x = quatOdomOld.pose.pose.position.x + std::cos(avgAngle)*cycleDistance;
  quatOdom.pose.pose.position.y = quatOdomOld.pose.pose.position.y + std::sin(avgAngle)*cycleDistance;
  q.setRPY(0, 0, cycleAngle + qOld.z());
 
  // Prevent lockup from a single bad cycle
  if (std::isnan(quatOdom.pose.pose.position.x) || std::isnan(quatOdom.pose.pose.position.y)
     || std::isnan(quatOdom.pose.pose.position.z)) {
    quatOdom.pose.pose.position.x = quatOdomOld.pose.pose.position.x;
    quatOdom.pose.pose.position.y = quatOdomOld.pose.pose.position.y;
    q.setRPY(0, 0, qOld.z());
  }
 
  // Make sure theta stays in the correct range
  if (q.z() > M_PI) {
    q.setRPY(0, 0, q.z() - 2*M_PI);
  }
  else if (q.z() < -M_PI) {
    q.setRPY(0, 0, q.z() + 2*M_PI);
  }

  // Compute the velocity
  quatOdom.header.stamp = this->get_clock()->now();
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.twist.twist.linear.x = cycleDistance/(quatOdom.header.stamp.nanosec - quatOdomOld.header.stamp.nanosec);
  quatOdom.twist.twist.angular.z = cycleAngle/(quatOdom.header.stamp.nanosec - quatOdomOld.header.stamp.nanosec);

   //Set orientation
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();

  //Fill covariance matrix
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
    }
    else if (i == 21 || i == 28 || i== 35) {
      quatOdom.pose.covariance[i] += 0.1;
    }
    else {
      quatOdom.pose.covariance[i] = 0;
    }
  }

  // Save the pose data for the next cycle
  quatOdomOld.pose.pose.position.x = quatOdom.pose.pose.position.x;
  quatOdomOld.pose.pose.position.y = quatOdom.pose.pose.position.y;
  qOld.setRPY(0, 0, q.z());
  quatOdomOld.header.stamp = quatOdom.header.stamp;

  odom_data_pub_quat_->publish(quatOdom);
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