cmake_minimum_required(VERSION 3.10)
project(assignment2)

# Use C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(-D_SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING )
  add_compile_options(-D_USE_MATH_DEFINES )
endif()

add_executable(assignment2 src/main.cpp)

target_sources(assignment2 PRIVATE
    include/joystick_listener.hpp
    include/velocity_kinematic.hpp
    include/pose_kinematic.hpp
    include/marker_broadcaster.hpp
    include/transform_broadcaster.hpp
	include/config_parser.hpp
	include/student_helper.hpp
)

target_sources(assignment2 PRIVATE
    src/joystick_listener.cpp
	src/velocity_kinematic.cpp
    src/pose_kinematic.cpp
    src/marker_broadcaster.cpp
    src/transform_broadcaster.cpp
	src/config_parser.cpp
	src/student_helper.cpp
)

target_include_directories(assignment2 PRIVATE include)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()
get_filename_component(PARENT_DIR ${PROJECT_SOURCE_DIR} PATH)

ament_target_dependencies(assignment2 rclcpp)
ament_target_dependencies(assignment2 sensor_msgs)
ament_target_dependencies(assignment2 geometry_msgs)
ament_target_dependencies(assignment2 visualization_msgs)
ament_target_dependencies(assignment2 tf2_geometry_msgs)
ament_target_dependencies(assignment2 tf2)
ament_target_dependencies(assignment2 tf2_ros)

install (DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(TARGETS assignment2 DESTINATION lib/${PROJECT_NAME})

ament_package()
