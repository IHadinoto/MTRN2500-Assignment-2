// Copyright 2019 Zhihao Zhang License MIT

#ifndef JOYSTICK_LISTENER_HPP_
#define JOYSTICK_LISTENER_HPP_

#include "config_parser.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp" // http://docs.ros.org/api/geometry_msgs/html/msg/AccelStamped.html
#include "rclcpp/rclcpp.hpp"       // http://docs.ros2.org/dashing/api/rclcpp/
#include "sensor_msgs/msg/joy.hpp" // http://wiki.ros.org/joy

#include <string>

namespace assignment2
{
class JoystickListener final : public rclcpp::Node
{
public:
    explicit JoystickListener(std::string const & zid, JoystickConfig config);

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_input_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr
        acceleration_output_;
    std::string const zid_;
    JoystickConfig const config_;
    auto joy_message_callback(sensor_msgs::msg::Joy::UniquePtr joy_message)
        -> void;
};
} // namespace assignment2

#endif // JOYSTICK_LISTENER_HPP_
