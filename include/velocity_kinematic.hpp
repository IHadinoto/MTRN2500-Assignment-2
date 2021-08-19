// Copyright 2019 Zhihao Zhang License MIT

#ifndef VELOCITY_KINEMATIC_HPP_
#define VELOCITY_KINEMATIC_HPP_

#include "config_parser.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp" // http://docs.ros.org/api/geometry_msgs/html/msg/AccelStamped.html
#include "geometry_msgs/msg/twist_stamped.hpp" // http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/

#include <chrono>
#include <string>

namespace assignment2
{
class VelocityKinematic final : public rclcpp::Node
{
public:
    explicit VelocityKinematic(std::string const & zid,
        std::chrono::milliseconds refresh_period, KinematicLimits config);

private:
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr
        acceleration_input_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_output_;
    geometry_msgs::msg::AccelStamped::UniquePtr acceleration_;
    geometry_msgs::msg::TwistStamped::UniquePtr velocity_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string const zid_;
    KinematicLimits config_;

    auto acceleration_callback(
        geometry_msgs::msg::AccelStamped::UniquePtr input_message) -> void;
    auto velocity_callback() -> void;
};
} // namespace assignment2

#endif // VELOCITY_KINEMATIC_HPP_
