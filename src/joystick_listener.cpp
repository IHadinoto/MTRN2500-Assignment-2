// Copyright 2019 Zhihao Zhang License MIT

#include "joystick_listener.hpp"

#include "student_helper.hpp"

#include <memory>
#include <string>
#include <utility>
#include <math.h> 

double range_map(double range_start, double range_end, 
		 double new_range_start, double new_range_end, double value);
double scale_with_deadzone(double deadzone, double value);

namespace assignment2
{
JoystickListener::JoystickListener(
    std::string const & zid, JoystickConfig config)
    : rclcpp::Node{helper::joy_node_name(zid)}
    , config_{config.speed_plus_axis,
				config.speed_minus_axis,
				config.steering_axis,
				config.steering_deadzone,
				config.speed_deadzone}
{
	
	//Subscribe to Joy
	auto callback = std::bind(&JoystickListener::joy_message_callback, this, std::placeholders::_1);
	this->joystick_input_ = create_subscription<sensor_msgs::msg::Joy>
				(std::string("/" + zid + "/joy"), 10, callback);

	//Create acceleration publisher
	this->acceleration_output_ = create_publisher<geometry_msgs::msg::AccelStamped>
				     (std::string("/" + zid + "/acceleration"), 10);
}

// ReSharper disable once CppMemberFunctionMayBeConst
auto JoystickListener::joy_message_callback(
	sensor_msgs::msg::Joy::UniquePtr joy_message) -> void
{
	//TASK 1A
    
	//Print Status of Each Axis
	std::for_each(joy_message->axes.begin(),
		joy_message->axes.end(),
		[](float axis) { std::cout << axis << "\t";}); 
    
 	//Print Number of Buttons Pressed
	std::cout << "\nTotal number of buttons pressed is "
			<< std::count(joy_message->buttons.begin(),joy_message->buttons.end(), 1) 
			<< "." << 
	std::endl;

	//TASK 1B

	double speed_plus_axis = joy_message->axes[config_.speed_plus_axis];
	double speed_minus_axis = joy_message->axes[config_.speed_minus_axis];
	double steering_axis = joy_message->axes[config_.steering_axis];
	double steering_deadzone = config_.steering_deadzone;
	double speed_deadzone = config_.speed_deadzone;

	//Scale Input Points outside of the deadzone 
	speed_plus_axis = scale_with_deadzone(speed_deadzone, speed_plus_axis); 
	speed_minus_axis = scale_with_deadzone(speed_deadzone, speed_minus_axis); 
	steering_axis = scale_with_deadzone(steering_deadzone, steering_axis); 

	//Map accelerations to their appropriate ranges 
	double pos_linear_accel = range_map(-1,1,0,1,speed_plus_axis);
	double neg_linear_accel = range_map(-1,1,0,-1,speed_minus_axis);
	double net_accel = pos_linear_accel + neg_linear_accel;
	double ang_accel = steering_axis;

	//Construct message to be published
	geometry_msgs::msg::AccelStamped acceleration_message;
	acceleration_message.accel.linear.x = net_accel;
	acceleration_message.accel.angular.z = ang_accel;
	acceleration_message.header.stamp = joy_message->header.stamp;
	acceleration_message.header.frame_id = joy_message->header.frame_id; 
	
	//Publish message 
	acceleration_output_->publish(acceleration_message);
}
} // namespace assignment2


//Maps Value from range -> new_range
double range_map(double range_start, double range_end, 
		 double new_range_start, double new_range_end, double value){	
	//Handle Zero Division 
	if (range_end - range_start == 0) return 0; 

	//Derived formula for linear map between 2 ranges 
	return 1.0*(value-range_start)/(range_end-range_start) * 
		   (new_range_end - new_range_start) + new_range_start; 
}

//Removes points in the deadzone 
double scale_with_deadzone(double deadzone, double value){
	//Values in the Deadzone are 0 
	if (fabs(deadzone) > fabs(value)) return 0;

	//Other Values are scaled appropriately  
	if(value < 0) return range_map(-deadzone,-1,0,-1,value); 
	return range_map(deadzone,1,0,1,value); 

}


