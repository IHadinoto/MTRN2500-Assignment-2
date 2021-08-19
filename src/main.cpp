// SPDX-License-Identifier: MIT
/**
 *  \brief     Main program
 *  \details   Provide to student.
 *  \author    Zhihao Zhang
 *  \version   2.0
 *  \date      2019
 *  \copyright MIT.
 **/

#include "config_parser.hpp"
#include "joystick_listener.hpp"
#include "marker_broadcaster.hpp"
#include "pose_kinematic.hpp"
#include "rclcpp/rclcpp.hpp" // http://docs.ros2.org/dashing/api/rclcpp/
#include "student_helper.hpp"
#include "transform_broadcaster.hpp"
#include "velocity_kinematic.hpp"

#include <chrono> // chrono_literals, https://en.cppreference.com/w/cpp/header/chrono
#include <fstream> // ifstream, https://en.cppreference.com/w/cpp/header/fstream
#include <iostream> // cout, https://en.cppreference.com/w/cpp/header/iostream
#include <memory> // make_shared, https://en.cppreference.com/w/cpp/header/memory
#include <string> // string, https://en.cppreference.com/w/cpp/header/string
#include <vector> // vector, https://en.cppreference.com/w/cpp/header/vector

auto create_rectangle_marker(std::string const & zid)
    -> visualization_msgs::msg::Marker
{
    auto body = visualization_msgs::msg::Marker{};

    body.header.frame_id = helper::local_frame_name(zid);
    // body.header.stamp
    body.ns = zid;
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::CUBE;
    body.action = visualization_msgs::msg::Marker::ADD;

    body.pose.position.x = 0;
    body.pose.position.y = 0;
    body.pose.position.z = 0;
    body.pose.orientation.x = 0;
    body.pose.orientation.y = 0;
    body.pose.orientation.z = 0;
    body.pose.orientation.w = 1;

    body.scale.x = 1.0;
    body.scale.y = 0.5;
    body.scale.z = 0.25;

    // colour red, green, blue, alpha (transparency)
    body.color.r = 1.0;
    body.color.g = 0.0;
    body.color.b = 0.0;
    body.color.a = 1.0;

    // body.colors.emplace_back();
    using namespace std::chrono_literals;
    body.lifetime = rclcpp::Duration{1s};
    return body;
}

auto create_visualisation_node(
    std::string const & zid, std::chrono::milliseconds refresh_period)
    -> std::shared_ptr<assignment2::MarkerBroadcaster>
{
    auto shape_list =
        std::make_shared<std::vector<visualization_msgs::msg::Marker>>();
    shape_list->emplace_back(create_rectangle_marker(zid));

    return std::make_shared<assignment2::MarkerBroadcaster>(
        zid, refresh_period, shape_list);
}

auto main(int argc, char * argv[]) -> int
{
    using namespace std::chrono_literals;

    try
    {
        rclcpp::init(argc, argv); // Initialise ROS2

        // Executor will handle running multiple nodes at the same time for us.
        // https://index.ros.org/doc/ros2/Tutorials/Intra-Process-Communication/
        auto ros_worker = rclcpp::executors::SingleThreadedExecutor{};

        // Read config from std::in.
	std::ifstream config_file; 
	if(argc > 1) config_file.open(argv[1]); 
	else {
 		std::cout << "File not found, please specify file as a commandline argument" << std::endl;
		exit(EXIT_FAILURE);
	} 
		
        auto const config_strings = assignment2::ConfigReader{config_file};
        auto const config = assignment2::ConfigParser{config_strings};
			
		
	std::string zid = config.get_zid(); 
	// Creating all the nodes we need and register with executor that will
        // service those nodes.
        auto input_node = std::make_shared<assignment2::JoystickListener>(
            zid, config.get_joystick_config());
        ros_worker.add_node(input_node);

        auto velocity_node = std::make_shared<assignment2::VelocityKinematic>(
            zid, 200ms, config.get_kinematic_config());
        ros_worker.add_node(velocity_node);

        auto pose_node =
            std::make_shared<assignment2::PoseKinematic>(zid, 100ms);
        ros_worker.add_node(pose_node);

        auto visual_node = create_visualisation_node(zid, 100ms);
        ros_worker.add_node(visual_node);

        auto transform_node =
            std::make_shared<assignment2::TransformBroadcaster>(zid);
        ros_worker.add_node(transform_node);

        // Run the executor, nodes take turns working.
        ros_worker.spin();
    }
    catch (std::exception & e)
    {
        // Something wrong occured, printing error message.
        std::cerr << "Error message:" << e.what() << "\n";
    }

    rclcpp::shutdown(); // Cleaning up before exiting.
    return 0;
}
