// Copyright 2019 Zhihao Zhang License MIT

#include "marker_broadcaster.hpp"

#include "student_helper.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
auto constexpr marker_topic = [](std::string const & zid) {
    return "/" + zid + "/marker";
};
} // namespace

namespace assignment2
{
MarkerBroadcaster::MarkerBroadcaster(std::string const & zid,
    std::chrono::milliseconds const refresh_period,
    std::shared_ptr<std::vector<visualization_msgs::msg::Marker>> shape_list)
    : rclcpp::Node{helper::marker_node_name(zid)}
    , marker_publisher_{create_publisher<visualization_msgs::msg::Marker>(
          marker_topic(zid), 10)}
    , timer_{create_wall_timer(
          refresh_period, [this]() -> void { marker_publisher_callback(); })}
    , shape_list_{std::move(shape_list)}
    , zid_{zid}
{
    assert(shape_list_);
}

// ReSharper disable once CppMemberFunctionMayBeConst
auto MarkerBroadcaster::marker_publisher_callback() -> void
{
    assert(shape_list_);
    for (auto && shape : *shape_list_)
    {
        auto message = std::make_unique<visualization_msgs::msg::Marker>(shape);
        message->header.stamp = rclcpp::Time{0};
        marker_publisher_->publish(std::move(message));
    }
}
} // namespace assignment2
