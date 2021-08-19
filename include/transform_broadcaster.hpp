// Copyright 2019 Zhihao Zhang License MIT

#ifndef TRANSFORM_BROADCASTER_HPP_
#define TRANSFORM_BROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // http://wiki.ros.org/tf2/Tutorials/Quaternions
#include "tf2_ros/transform_broadcaster.h" // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29

#include <string>

namespace assignment2
{
class TransformBroadcaster : public rclcpp::Node
{
public:
    explicit TransformBroadcaster(std::string const & zid);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_source_;
    tf2_ros::TransformBroadcaster transform_output_;
    std::string zid_;

    auto incoming_callback(
        geometry_msgs::msg::PoseStamped::UniquePtr pose_message) -> void;
};
} // namespace assignment2
#endif // TRANSFORM_BROADCASTER_HPP_
