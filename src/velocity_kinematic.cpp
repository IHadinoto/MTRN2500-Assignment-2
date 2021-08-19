// Copyright 2019 Zhihao Zhang License MIT

#include "velocity_kinematic.hpp"

#include "student_helper.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <utility>

double bound_value (double value, double bound);

namespace assignment2
{
VelocityKinematic::VelocityKinematic(std::string const & zid,
    std::chrono::milliseconds const refresh_period, KinematicLimits config)
    : rclcpp::Node(helper::velocity_node_name(zid))
{	
	this->config_ = config; 

	//Initialise Initial Parameters 
	this->acceleration_ = std::make_unique<geometry_msgs::msg::AccelStamped>(); 	
	this->velocity_ = std::make_unique<geometry_msgs::msg::TwistStamped>(); 

	this->acceleration_->header.stamp.sec = 0; 
	this->acceleration_->header.stamp.nanosec = 0;

	this->velocity_->twist.linear.x = 0; 
	this->velocity_->twist.angular.z = 0; 
	this->velocity_->header.stamp = rclcpp::Node::now();
	this->velocity_->header.frame_id = zid; 

	//Subscribe to Acceleration 
	auto accel_callback_wrapper = std::bind(&VelocityKinematic::acceleration_callback, this, std::placeholders::_1);
	this->acceleration_input_ = create_subscription<geometry_msgs::msg::AccelStamped>
				    (std::string("/" + zid + "/acceleration"), 10, accel_callback_wrapper);

    //Create velocity publisher
	this->velocity_output_ = create_publisher<geometry_msgs::msg::TwistStamped>
				(std::string("/" + zid + "/velocity"), 10);

	//Set up wall timer
	auto velocity_callback_wrapper = std::bind(&VelocityKinematic::velocity_callback, this);
	this->timer_ = create_wall_timer(std::chrono::milliseconds{refresh_period}, velocity_callback_wrapper); 

}

auto VelocityKinematic::acceleration_callback(
    geometry_msgs::msg::AccelStamped::UniquePtr input_message) -> void
{
	//Move input_message data to acceleration_ 
	acceleration_->accel.linear.x = input_message->accel.linear.x * config_.max_linear_acceleration;
	acceleration_->accel.angular.z = input_message->accel.angular.z * config_.max_angular_acceleration;  

	//Calculate dt and update time stamp 
    double dt = (rclcpp::Node::now() - acceleration_->header.stamp).seconds();

	if (acceleration_->header.stamp.sec == 0 && acceleration_->header.stamp.nanosec == 0) dt = 0; 
	acceleration_->header.stamp = input_message->header.stamp; 

	//v(t+dt) = v(t) + dt * a(t)
	velocity_->twist.linear.x += dt * acceleration_->accel.linear.x;
	velocity_->twist.angular.z += dt * acceleration_->accel.angular.z;
	
	//Correct too big values f
	velocity_->twist.linear.x = bound_value(velocity_->twist.linear.x, config_.max_linear_speed); 
	velocity_->twist.angular.z = bound_value(velocity_->twist.angular.z, config_.max_angular_speed); 
	
	//Display Message
	std::cout << "DT: " << dt <<
		", Linear Acceleration: " << acceleration_->accel.linear.x <<
		", Linear Velocity: "<< velocity_->twist.linear.x <<
		", Angular Acceleration: " << acceleration_->accel.angular.z <<
		", Angular Velocity: " << velocity_->twist.angular.z <<  
	std::endl;
}

auto VelocityKinematic::velocity_callback() -> void
{

	//Don't start until the first message is received
	if (acceleration_->header.stamp.sec == 0 && acceleration_->header.stamp.nanosec == 0) return;
	if ((rclcpp::Node::now() - acceleration_->header.stamp).seconds() >= 10){
		//Reset acceleration and velocity 		
		acceleration_->accel.linear.x = 0;
		acceleration_->accel.angular.z = 0;
		acceleration_->header.stamp.sec = 0; 
		acceleration_->header.stamp.nanosec = 0; 

		velocity_->twist.linear.x = 0; 
		velocity_->twist.angular.z = 0; 

		std::cout << "Connection Lost" << std::endl; 
		return;
	}	

	//velocity_ should have all the required info at this point - publish it
	velocity_output_->publish(*velocity_);
}
} // namespace assignment2


//Take positive bound and return bounded value 
double bound_value (double value, double bound){
	if(value > bound) return bound;
	if(value < -bound) return -bound; 
	return value; 	
}
