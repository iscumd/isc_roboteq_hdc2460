#pragma once

#include <memory>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

class EncoderOdom: public rclcpp::Node
{
public:
 explicit EncoderOdom(rclcpp::NodeOptions options);
private:

// Robot physical constants
double wheel_radius{}; // Wheel radius in meters
double wheel_seperation{}; // Center of left tire to center of right tire

rclcpp::Time currentTime{};
rclcpp::Time lastTime{};

//Positions
double x{0.0};
double y{0.0};
double theta{0.0};

//Velocities in the different directions
double velocity_x{};
double velocity_y{0.0};
double velocity_th{};


//Wheel velocities
double left_speed{};
double right_speed{};

//Deltas
double delta_time{};
double delta_x{};
double delta_y{};
double delta_theta{};


//Odom msgs
nav_msgs::msg::Odometry odom{};
geometry_msgs::msg::Quaternion q_msg{};
tf2::Quaternion q{};

void publish_quat();

void encoder_callback(const std_msgs::msg::Int16::ConstSharedPtr &left_encoder, const std_msgs::msg::Int16::ConstSharedPtr &right_encoder);

//Subscribers and message filters
message_filters::Subscriber<std_msgs::msg::Int16> left_encoder_count_sub_;
message_filters::Subscriber<std_msgs::msg::Int16> right_encoder_count_sub_;
typedef sync_policies::ApproximateTime<std_msgs::msg::Int16, std_msgs::msg::Int16> MySyncPolicy;
typedef Synchronizer<MySyncPolicy> Sync;
std::shared_ptr<Sync> sync;

//Publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;


};
