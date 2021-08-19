// Copyright 2019 Zhihao Zhang License MIT

#ifndef MARKER_BROADCASTER_HPP_
#define MARKER_BROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace assignment2
{
class MarkerBroadcaster final : public rclcpp::Node
{
public:
    explicit MarkerBroadcaster(std::string const & zid,
        std::chrono::milliseconds refresh_period,
        std::shared_ptr<std::vector<visualization_msgs::msg::Marker>>
            shape_list);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> shape_list_;
    std::string zid_;

    auto marker_publisher_callback() -> void;
};
} // namespace assignment2
#endif // MARKER_BROADCASTER_HPP_
