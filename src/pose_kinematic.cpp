// Copyright 2019 Zhihao Zhang License MIT

#include "pose_kinematic.hpp"

#include "student_helper.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <utility>

#include "math.h"
 
#define sind(x) (sin(x * 3.1415/ 180))
#define cosd(x) (cos(x * 3.1415/ 180))

namespace assignment2
{
PoseKinematic::PoseKinematic(
    std::string const & zid, std::chrono::milliseconds const refresh_period)
    : rclcpp::Node(helper::pose_node_name(zid))
{
	//Initialise Initial Parameters 
	this->velocity_ = std::make_unique<geometry_msgs::msg::TwistStamped>(); 	
	this->pose_ = std::make_unique<geometry_msgs::msg::PoseStamped>(); 

	this->pose_->pose.position.x = 0; 
	this->pose_->pose.position.y = 0; 
	this->pose_->pose.orientation.z = 0; 
	this->pose_->header.stamp = rclcpp::Node::now();
	this->pose_->header.frame_id = zid; 

	this->velocity_->twist.linear.x = 0; 
	this->velocity_->twist.angular.z = 0; 

	//Subscribe to Velocity 
	auto velocity_callback_wrapper = std::bind(&PoseKinematic::velocity_callback, this, std::placeholders::_1);
	this->velocity_input_ = create_subscription<geometry_msgs::msg::TwistStamped>(std::string("/" + zid + "/velocity"), 10, velocity_callback_wrapper);

	//Create Pose publisher
	this->pose_output_ = create_publisher<geometry_msgs::msg::PoseStamped>(std::string("/" + zid + "/pose"), 10);

	//Set up wall timer
	auto pose_callback_wrapper = std::bind(&PoseKinematic::pose_callback, this);
	this->timer_ = create_wall_timer(std::chrono::milliseconds{refresh_period}, pose_callback_wrapper);
}

auto PoseKinematic::velocity_callback(
    geometry_msgs::msg::TwistStamped::UniquePtr input_message) -> void
{
	velocity_->twist.linear.x = input_message->twist.linear.x; 
	velocity_->twist.angular.z = input_message->twist.angular.z; 
}

auto PoseKinematic::pose_callback() -> void
{
	//Calculate dt and update time stamp 
	double dt = (rclcpp::Node::now() - pose_->header.stamp).seconds();
	pose_->header.stamp = rclcpp::Node::now();

	//v(t+dt) = v(t) + dt * v(t)
	pose_->pose.position.x += dt * velocity_->twist.linear.x * cos(dt * velocity_->twist.angular.z); 
	pose_->pose.position.y += dt * velocity_->twist.linear.x * sin(dt * velocity_->twist.angular.z); 
	pose_->pose.orientation.z += dt * velocity_->twist.angular.z * 3.14159/180; 

	//pose_ should have all the required info at this point - publish it
	pose_output_->publish(*pose_);
}
} // namespace assignment2d
