#ifndef ENCODER_ODOM__EKF_ODOM_PUB_HPP_
#define ENCODER_ODOM__EKF_ODOM_PUB_

#include <memory>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

class EncoderOdom: public rclcpp::Node
{
public:
 explicit EncoderOdom(rclcpp::NodeOptions options);
private:

//Initial pose
const double initialX{0.0};
const double initialY{0.0};
const double initialTheta{0.00000000001};

// Robot physical constants
double ticks_per_revolution{}; // How many ticks per each revolution of the wheel
double wheel_radius{}; // Wheel radius in meters
double wheel_base{}; // Center of left tire to center of right tire
double ticks_per_meter{}; // Based on ticks_per_revolution and wheel size

// Distance both wheels have traveled
double distanceLeft{0};
double distanceRight{0};

int lastCountR{0};
int lastCountL{0};
int left_ticks{0};
int right_ticks{0};

// Flag to see if initial pose has been received
bool initialPoseRecieved{false};

//Odom msgs
nav_msgs::msg::Odometry quatOdom{};
nav_msgs::msg::Odometry quatOdomOld{};
tf2::Quaternion q{};
tf2::Quaternion qOld{};

void publish_quat();

void encoder_callback(const std_msgs::msg::Int16::ConstSharedPtr &left_encoder, const std_msgs::msg::Int16::ConstSharedPtr &right_encoder);

//Subscribers and message filters
message_filters::Subscriber<std_msgs::msg::Int16> left_encoder_count_sub_;
message_filters::Subscriber<std_msgs::msg::Int16> right_encoder_count_sub_;
typedef sync_policies::ApproximateTime<std_msgs::msg::Int16, std_msgs::msg::Int16> MySyncPolicy;
typedef Synchronizer<MySyncPolicy> Sync;
std::shared_ptr<Sync> sync;

//Publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat_;


};


#endif  //ENCODER_ODOM__EKF_ODOM_PUB_
