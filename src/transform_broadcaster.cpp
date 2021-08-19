// Copyright 2019 Zhihao Zhang License MIT

#include "transform_broadcaster.hpp"

#include "student_helper.hpp"
#include "tf2/LinearMath/Quaternion.h" // http://wiki.ros.org/tf2/Tutorials/Quaternions

#include <memory>
#include <string>
#include <utility>

namespace assignment2
{
TransformBroadcaster::TransformBroadcaster(std::string const & zid)
    : rclcpp::Node{helper::transform_node_name(zid)}
    , pose_source_{create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + zid + "/pose", 10,
          [this](geometry_msgs::msg::PoseStamped::UniquePtr input_message)
              -> void { incoming_callback(std::move(input_message)); })}
    , transform_output_{*this}
    , zid_{zid}
{
}

auto TransformBroadcaster::incoming_callback(
    geometry_msgs::msg::PoseStamped::UniquePtr pose_message) -> void
{
    if (pose_message)
    {
        auto transform_message =
            std::make_unique<geometry_msgs::msg::TransformStamped>();
        transform_message->header.frame_id = "world_frame";
        transform_message->header.stamp = pose_message->header.stamp;
        transform_message->child_frame_id = helper::local_frame_name(zid_);

        transform_message->transform.translation.x =
            pose_message->pose.position.x;
        transform_message->transform.translation.y =
            pose_message->pose.position.y;

        auto const new_heading = pose_message->pose.orientation.z;
        auto const new_heading_quaternion =
            tf2::Quaternion{tf2::Vector3{0, 0, 1}, new_heading};
        transform_message->transform.rotation =
            tf2::toMsg(new_heading_quaternion);
        transform_output_.sendTransform(*transform_message);
    }
}
} // namespace assignment2
