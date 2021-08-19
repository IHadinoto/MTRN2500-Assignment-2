// Copyright 2019 Zhihao Zhang License MIT

#ifndef POSE_KINEMATIC_HPP_
#define POSE_KINEMATIC_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp" // http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
#include "geometry_msgs/msg/twist_stamped.hpp" // http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/

#include <chrono>
#include <string>

namespace assignment2
{
class PoseKinematic final : public rclcpp::Node
{
public:
    explicit PoseKinematic(
        std::string const & zid, std::chrono::milliseconds refresh_period);

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_input_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_output_;
    geometry_msgs::msg::TwistStamped::UniquePtr velocity_;
    geometry_msgs::msg::PoseStamped::UniquePtr pose_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::string const zid_;

    auto velocity_callback(
        geometry_msgs::msg::TwistStamped::UniquePtr input_message) -> void;
    auto pose_callback() -> void;
};

} // namespace assignment2
#endif // POSE_KINEMATIC_HPP_
