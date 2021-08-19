**The signature of the class you will need to implemented in this assignment is provided in `.hpp` file in the include directory. Write your code in the corresponding `.cpp` file. You can add any extra code to `student_helper.hpp` and `student_helper.cpp`**

# MTRN2500 Assignment 2 Specification

## Purpose
The purpose of this assignment is to familiarise yourself with larger programming projects. In this assignment, you will be applying your newly learned `C++` skills to allow for controller input to be consumed and processed by industry standard middleware (Robot Operating System - ROS) and visualised.


## Assignment Instructions
You will be submitting your code to the private Github assignment2 repository created for you.

After the repository was created, you must do the following:

1. Add an empty file to the root directory with your zid as the filename in the format `z0000000.txt`. 
2. Create a `review` branch. Style task will be assessed on that branch. 
3. Create a development branch. You should do work in a development branch and only merge working code into the master branch. 

**Note: failing to do the above will result you getting 0 marks for the assignment.**

`mtrn2500_ws` in the provided MTRN2500 virtual machine is the recommended location for you to clone the git repository.

Demonstration will be during your tutorial in week 9 (note this is one week later than advised in the course outline). Students in the Monday tutorial will demonstrate in week 10 due to the public holiday. 

Code style submission is due at 5pm Friday week 9 (15 November 2019).

For the style task, create a pull request of your commit from the `master` branch to `review` branch. Your demonstrator will do a code review on your pull request. Your demonstrator can give early feedback if you create the pull request and ask your demonstrator to review it in your lab session. 

Final assessment of the style task will be be based on the pull request to `review` from the last commit to the `master` branch before the code style submission due date.


## Useful Information

### `this`
There is a special pointer called `this` that is provided to all methods that are run from instantiated objects. It is a pointer to the instantiated object that the method is being run on. It is, however, not necessary to use when referring to member variables and methods as you are able to refer to member variables and member methods by their name without using the keyword `this`.

### Templates
In addition to regular parameters, some classes and function may take `types` as parameter. For example the standard template library container can be adapted to work with different types by taking in a type parameter. You will learn more about this later, at this stage you just need to know how to use them. The type parameter is passed in `< >` after name and before `( )` as in `std::vector <int> ()`.

### Lambda (anonymous) functions
Lambda functions are special functions that can be declared in `C++`. These functions are different to functions that you are familiar with when using regular standalone functions or methods on a class. They functions are declared within the scope of some other function and exist as a function pointer within the scope that they're declared in, which means they also abide by the scope that they are defined in (see the Week 6 Tutorial for more information about this). Unlike regular classes and functions which are generated once (during compilation of the program) and have a fixed name for the entire runtime of the program, lambda functions are dynamic in the sense that their function code (the code that is executed when the function is called) is generated at runtime but their names are not, as well as being not being a reusable function and generally representing functions which have no purpose being named. Additionally, they are generally short functions that should generally not be longer than five lines - at this point, you would benefit more from declaring a regular function.

An example of how to declare a lambda function can be found below:
```cpp
#include <algorithm>
#include <iostream>
#include <vector>

int main (void) {
  std::vector<double> numbers{1, 2, 3, 8, 4};

  // store the largest number in the maxValue variable
  double maxValue = 0;
  std::for_each(numbers.begin(), numbers.end(), [&maxValue](const double& value) {
    if (value > maxValue) {
      maxValue = value;
    }
  });

  std::cout << "Largest value is: " << maxValue << std::endl;
  return 0;
}
```

Let's work through this one section at a time.

```cpp
#include <algorithm>
#include <iostream>
#include <vector>
```

Including the necessary libraries for our `std::for_each` function, outputting to the terminal and for storing our values.

```cpp
  std::vector<double> numbers{1, 2, 3, 8, 4};
```

Creating a vector of numbers using an initialiser list (which is a special kind of constructor).

```cpp
  double maxValue = 0;
```

Creating a variable where we will be storing the largest value in the vector.

```cpp
  std::for_each(numbers.begin(), numbers.end(), [&maxValue](const double& value) {
    if (value > maxValue) {
      maxValue = value;
    }
  });
```

Here we have the main portion of this example - defining the lambda function. We first have the use of a `std::for_each` function, which takes in the starting and ending vector iterators which define where we want this lambda function to operate on. We then have a lambda function which is defined that is comprised of three sections:
* `[&maxValue]`: variables which are captured
* `(const double& value)`: the input arguments to this function, where the chosen algorithm (which is `std::for_each` in this case) passes each value between the start and end iterators
* `{ ... }`: the contents of the function

The parameters which are captured (the first section `[ ... ]`) declare which variables that exist in the scope that this lambda function is defined in which are to be passed to the lambda function. If the `&` was omitted, the parameter would be copied and the copy is passed to the lambda function. With the `&`, a reference to the original parameter is created and the reference is passed in to the lambda function. The lambda function can also store the `this` pointer by value using the syntex `[this]`

```cpp
  std::cout << "Largest value is: " << maxValue << std::endl;
```

Outputting the value of the largest number in the previously created vector to the terminal in a nicely formatted string.

## Introduction to Robot Operating System (ROS)

An introduction to ROS and the purpose behind it can be found in the Week 5 Tutorial. There are a few concepts that are integral to how ROS works.

### Packages
ROS organises all user-written software in a specific way. All libraries and executables are organised into separate packages which each represent a specific area of functionality. For example: given a bipedal humanoid robot, one would organise a package that deals purely with moving the limbs of the robot. Another would deal purely with camera input. Another would deal with high-level movement planning logic, etc. 

### Nodes
Given that packages are libraries and executables that are represent specific areas of functionality, nodes can be understood as specific tasks that are undergone to perform some processing, communication or visualisation. The nodes are the final executables which are orchestrated by ROS so that they can communicate between each other. There can be multiple ROS nodes in a single ROS package.

To create a node, you must first create a class which you wish to use to conceptually represent the node. Consider the example where you wish to write a node called `ColourImageProcessing_node` through the `ColorImageProcessing` class which performs colour image processing in a camera processing package. You can specify that you wish to have that class be a node in the following way:

```cpp
class ColorImageProcessing : public rclcpp::Node {
  ColorImageProcessing() : rclcpp::Node{std::string{"ColorImageProcessing_node"}} {}
}
```

### Topics
Topics are the main method of communicating between ROS packages. You can think of topics as channels of communication between nodes. ROS nodes can not only be run on a single machine (as you will be facilitating in this assignment), but they can also be run on multiple machines in a distributed fashion. ROS handles the networking and communication portion of this through these topics. They work using a Publisher / Subscriber model. This means that any node which subscribes to a particular topic will receive a message sent by a node which publishes to the same topic.

**FUN FACT:** Nodes can both publish and subscribe to multiple topics all at the same time.

### Subscribers
In order for a node to receive information from another node, they must first subscribe to a topic. There are three pieces of information required to allow a node to subscribe to a topic. These are:
* topic name (string naming the topic being subscribed to)
* queue length (unsigned integer representing the amount of messages that can be stored in the message cue before the node refuses to accept any further messages through this subscription) 
* callback (a function pointer for a function that is run when a new message is received on the subscribed topic)

Here is an example of how to create one:
```cpp
class Example : public rclcpp::Node {
  Example() : rclcpp::Node{std::string{"Example_node"}} {
    auto example_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
                                std::string("topic"),
                                10,
                                [this](sensor_msgs::msg::Joy::UniquePtr joy_message){
                                  // callback implementation
                                }
                              );
    }
  }
}
```

Or, if you would prefer to implement your callback in a separate function without using a lambda function:
```cpp
class Example : public rclcpp::Node {
  Example() : rclcpp::Node{std::string{"Example_node"}} {
    auto callback = std::bind(&JoystickListener::joy_message_callback, this, std::placeholders::_1);
    auto example_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(std::string("topic"), 10, callback);
  }

  void topic_callback(sensor_msgs::msg::Joy::UniquePtr joy_message) {
    // callback implementation
  }
}
```
Note that when we attach the callback function we need to pass in the `this` argument, however we cannot do that explicitly and therefore we must bind a partial function to the callback with `std::bind` to pass in the hidden `this` argument.  

### Publishers
In order for information to be communicated to other components of the system, it must first be packaged and sent. This is the role of a publisher. In order to publish a message, you must first create a data structure which defines this message, populate it with the required information and then publish it. An example can be found below.

```cpp
class Example : public rclcpp::Node {
  Example() : rclcpp::Node{std::string{"node_name"}} {
    auto string_publisher_ = create_publisher<geometry_msgs::msg::String>(std::string{"ExampleTopic"}, 10)
  }
};

// within some other function / method
auto string_information = sensor_msgs::msg::String();
// the fields of a particular structure can be found in the "msgs" directory of a "msgs" package if it is a custom message
string_information.message = std::string("This is some example text that will be sent through a topic");
string_publisher_->publish(stringInformation);
```

### Timers
Another method capable of automatically causing some events to be kicked off at a regular time interval is through the use of "wall timers", which allow you to set a callback function to be run continually at some time interval. The time requires two parameters - the time interval that the callback will be run in, and a function pointer to the callback itself. An example of how to set a wall timer can be found below: 

```cpp
#include <chrono>

class Example : public rclcpp::Node {
  Example() : rclcpp::Node{std::string{"Example_node"}} {
    auto timer_ = create_wall_timer(std::chrono::milliseconds{100},
                    [this]() {
                        // function which implements the regularly occurring task
                      }
                    );
  }
};
```

## Task 0: Launch joystick driver (no marks)
**NOTE: All the messages you need to send and receive will be in the ROS message namespace `/zxxxxxxx/`where `zxxxxxxx` will be provided to you on the day of demonstration. For the rest of this document we will use `/z0000000` as a place holder**

`joy_node` in the package `joy` is an executable that will interface with an Xbox controller and send inputs as `sensor_msgs::msg::Joy` message to the topic `/joy`. A launch file is provided to redirect the message to `/z0000000/joy`, the launch file will also configure `joy_node` to read from the correct device.

To launch the joystick driver, in the directory you cloned the git repository:
1. Install the launch script using the command: `. mtrn2500_make`
2. Launch joy_node using the command: `ros2 launch assignment2 joy_node.py`
3. A quick way to show message being sent to a topic is use the command `ros2 topic echo /z0000000/joy`. You can check the list of current topic by using the command `ros2 topic list`.

You can change the topic the `joy` message is redirected to by editing `joy_node.py` and 
`joy_node_config.yaml` file in the `launch` folder.


## Task 1: Read the joystick (6 marks) 
This task is handled by the class `JoystickListener` in the header `joystick_listener.hpp`.

 The purpose of this task is to decode the `sensor_msgs::msg::Joy` message from the topic `zxxxxxxx/joy` to get the player actions. Send the player actions as a `geometry_msgs::msg::AccelStamped` message to the topic `/z0000000/acceleration`. i.e. the `JoystickListener` class will convert joystick inputs into acceleration values and publish these to be used by any subsequent nodes.

`JoystickListener` class contains:
* `joystick_input_` is a `std::shared_ptr` to a subscriber used to listen to `/z0000000/joy`.
* `acceleration_output_` is a `std::shared_ptr` to a publisher used to send the acceleration data to `/z0000000/acceleration`.
* `zid_` is a string that contains `z0000000`.
* `config_` is a `joystick_config` struct hold your joystick configuration data.
* `joy_message_callback` is a method that will be called every time a new message is received by `joystick_input_`, the new message will be passed in as `joy_message` parameter. You need to make sure to register this function when you create `joystick_input_`.
 
`JoystickConfig` struct is defined in `config_parser.hpp` as:
```c++
struct JoystickConfig
{
public:
    std::size_t speed_plus_axis;
    std::size_t speed_minus_axis;
    std::size_t steering_axis;
    double steering_deadzone;
    double speed_deadzone;
};
```

[`sensor_msgs::msg::Joy`](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Joy.msg) contains 
* `header`
	* `string frame_id`: `std::string` containing the transform frame with which this data is associated.
	* `builtin_interfaces/Time stamp`:  Two-integer timestamp that is expressed as seconds and nanoseconds.
* `float32[] axes`:  `std::vector<float>` containing the axes measurements from a joystick.
* `int32[] buttons` `std::vector<int32_t>` containing the buttons measurements from a joystick.

All the other ROS2 message has similar structure. Documentation can be found at [ROS2 common interface](https://github.com/ros2/common_interfaces)


### Subtask A  : Print all axis and buttons status to `std::cout` (2 marks)

In the method `joy_message_callback`:
1.  use `std::for_each` to iterate through the `axes` vector and print out the value of each axes in a single line. Separate each value with a tab. Format: `0.0\t1.234567\t1.0\t2.0\t...\t\n`.
2. Use `std::count` or `std::count_if` to find out how many buttons have been pressed simultaneously. Print the value to `std::cout` in the format: `Total number of buttons pressed is {}.\n`, replace `{}` with total number of buttons pressed.

### Sub-task B: Calculate linear and angular acceleration inputs (4 marks)
The axes we will need to read from are defined in `config_` .
* speed_plus_axis: Positive linear acceleration axis value
* speed_minus_axis: Negative linear acceleration axis
* steering_axis: Angular acceleration axis

The axis could be a trigger or a joystick on the Xbox controller.

In the method `joy_message_callback`:
1. Calculate positive linear acceleration. Axis input in the range of [-1.0, 1.0] corresponds to an acceleration output in the range of [0.0, 1.0].
2. Calculate negative linear acceleration. Axis input in the range of [-1.0, 1.0] corresponds to an acceleration output in the range of [0.0, -1.0].
3.  Calculate the angular acceleration. Axis input in the range of [-1.0, 1.0] corresponds to an acceleration output in the range of [-1.0, 1.0].
4. Calculate the net linear acceleration equal to the sum of positive and negative linear acceleration in the range of [-1.0, 1.0].
5. Due to the physical design the joystick value may be a small non-zero value at the neutral position. This is known as deadzone. We want to treat those inputs as zero. Treat input within plus or minus deadzone value as zero. Scale the input such that input [deadzone , 1.0] scales to [0.0 , 1.0] and [-deadzone , -1.0] to [0.0, and -1.0], The deadzone value is specified in `config_.speed_deadzone` and `config_.steering_deadzone`. 
6. Send a `geometry_msgs::msg::AccelStamped` using `publish` method of the publisher pointed by`acceleration_output_`.

The `geometry_msgs::msg::AccelStamped` message should contain:
* Net acceleration as the `x` component of the `linear` part of `accel` member in the message.
* Angular acceleration as the `z` component of the `angular` part of `accel` member in the message.
* Use the time `stamp` of the joystick message as the time `stamp` in the `header`
* Use `zid` as the `header.frame_id`.

You can check the message by echoing it to the terminal by using the command:

7. `ros2 topic list` 
8. `ros2 topic echo /z0000000/joy`

## TASK 2: Velocity and Pose (4.5 marks)
Calculate the velocity and pose of the vehicle at regular interval using the acceleration input. Velocity is calculated with respect to the vehicle body and pose with respect to the global coordinate frame.

### Sub-task A: Calculate linear and angular velocity (1.5 marks)
This task is handled by the class `VelocityKinematic` in the header `velocity_kinematic.hpp`. This class will subscribe to the topic `/z0000000/acceleration` from the previous task and integrate the acceleration to obtain velocity. Periodically send the calculated velocity as a `geometry_msgs::msg::TwistStamped` message to the topic `/z0000000/velocity`.

1. Assume initial velocities are both zero.
2. Calculate time difference `dt` between the last time `stamp` and the current time. You can get the current time by calling the function `now()`. Hint: The ROS time class has a method called `seconds()`;
3. Integrate the velocity at the rate `refresh_period` milliseconds. `refresh_period` is a constructor parameter.
4. You can use the formula: `v(t+dt) = v(t) + dt * a(t)`
5. Print the accelerations and velocities to the screen: Format: `DT: {}, Linear Acceleration: {}, Linear velocity: {}, Angular acceleration: {}, Angular velocity: {}`. `{}` indicates the position of each value and should not be printed.
6. Send a `geometry_msgs::msg::TwistStamped` to the topic `/z0000000/velocity` at frequency of `1/refresh_period`. 

The `geometry_msgs::msg::TwistStamped` message should contain:
* Linear velocity as the `x` component of the `linear` part of `twist` member in message.
* Angular velocity as the `z` component of the `angular` part of `twist` member in message.
* Set the time used to calculate `dt` as the time `stamp` in the `header`
* Use `zid` as the `header.frame_id`.

### Sub-task B: Error handling (1.5 marks)
* Limit maximum velocity to based on the plus/minus maximum velocity set in the `kinematic_limits config_`.
* Do nothing until the first `geometry_msgs::msg::AccelStamped` has been received.
* If the last velocity message received was older then 10 seconds, consider the communication lost and set the acceleration and velocity to zero. Print "Communication lost.\n" and stop sending `geometry_msgs::msg::TwistStamped` until a new `geometry_msgs::msg::AccelStamped` has been received.

### Sub-task C: Calculate position and orientation (pose) (1.5 marks)
This task is handled by the class `PoseKinematic` in the header `pose_kinematic.hpp`. This class will subscribe to the topic `/z0000000/velocity` from the last part and integrate the velocity to get current pose. Periodically send the calculated pose as a `geometry_msgs::msg::PoseStamped` message to the topic `/z0000000/pose`.
* Assume starting at the origin (0.0, 0.0) with heading = 0.
* Calculate time difference between the last time `stamp` and the current time.
* Convert the velocity to the global coordinate frame using trigonometric functions.
* Calculate the position and orientation at the rate `refresh_period` milliseconds. `refresh_period` is a constructor parameter.

The `geometry_msgs::msg::PoseStamped` message should contain"
* (x,y) coordinate as the `position` part of `pose` member in message.
* Angular acceleration as the `z` component of the `orientation` part of `pose` member in message.
* Use the time used to calculate `dt` as the time `stamp` in the `header`
* Use `zid` as the `header.frame_id`.

## Task 3: Parse config file (4.5 marks)
This task is handled by the class `ConfigReader` and `ConfigParser` in the header `config_parser.hpp`. 

Read and parse the config file as opposed to using hard-coded configuration parameters. `config_reader` will read in the config file into a `std::unordered_map<std::string,std::string>`, while `config_parse` will convert the result from `config_reader` to actual configurations.


### Sub-task A: Read the config file (1.5 marks)
This task is handled by the class `ConfigReader` in the header `config_parser.hpp`.

1. Location of the config file will be passed in as a commandline argument. 
2. Modify the main program to read from a config file instead of `std::cin`.
3. Read each line config file in the `config_reader` constructor. Print each line to the terminal in the format: `Line {number}: {string}\n`. `{}` indicate the position of each value and should not be printed.
4. Finish reading the file when an empty line was encounter.

### Sub-task B: Read the config file (1.5 marks)
This task is handled by the class `ConfigReader` in the header `config_parser.hpp`.

The config file will be in the format `key : value`.
All the keys will be unique. Key and value may be surrounded by arbitrary amount of spaces. Each line consist of only a single pair of `key` and `value`.

1. Split each line into `key` and `value` string.
2. Trim leading and trailing space from `key` and `value`.
3. Store the config in `config_` as pair of `key` and `value` strings.
4. After all the lines have been read, iterate through config_ and print `key` and `value` pairs. Format `key: \"{key}\", value: \"{value}\"\n`. `{}` indicates the position of the value and should be not printed.

### Sub-task C: Parse the config file (1.5 marks)
This task is handled by the class `ConfigParser` in the header `config_parser.hpp`.

1. config_parser constructor should read from `config` and initialise all the members.
```
    std::string const zid_;
    std::chrono::milliseconds const refresh_period_;
    joystick_config const joy_config_;
    kinematic_limits const kinematic_config_;
```
2. Also implement the getter methods.


## Style and Source Control (5 mark)

The most important thing with style is to make your code understandable to other people. 

* Follow the course style guide which is based on ROS2 development guide.
* Neat and tidy code, style is consistent throughout. 
* Good choice of names that are meaningful. 
* Good use of functions to split code into manageable segments and avoid code duplication.
* Good use of c++ features.
* Good documentation and comments. 
* No error or warning message when compiling. 
* `master` branch must be able to compile and work. 
* Consistent use of source management. Create development branches. Keep your Github repository up to date with you work. Write meaningful commit message. 

## Finally
For full marks on each component you will need to demonstrate it working and be able to explain the code you have implemented to your demonstrator within your demonstration.

For assistance, please post questions in the course MS Teams Assignment2 channel.

Enjoy getting started in C++, as the following assignment will build on much of this content.
