// Copyright 2019 Zhihao Zhang License MIT

#ifndef CONFIG_PARSER_HPP_
#define CONFIG_PARSER_HPP_

#include <chrono>
#include <istream>
#include <string>
#include <unordered_map>

namespace assignment2
{
struct JoystickConfig
{
public:
    std::size_t speed_plus_axis;
    std::size_t speed_minus_axis;
    std::size_t steering_axis;
    double steering_deadzone;
    double speed_deadzone;
};

struct KinematicLimits
{
public:
    double max_linear_speed;
    double max_angular_speed;
    double max_linear_acceleration;
    double max_angular_acceleration;
};

class ConfigReader
{
public:
    explicit ConfigReader(std::istream & config_file);
    [[nodiscard]] auto find_config(std::string const & key,
        std::string const & default_value) const -> std::string;

private:
    std::unordered_map<std::string, std::string> config_;
};

class ConfigParser
{
public:
    explicit ConfigParser(ConfigReader const & config);

    [[nodiscard]] auto get_zid() const -> std::string;
    [[nodiscard]] auto get_refresh_period() const -> std::chrono::milliseconds;
    [[nodiscard]] auto get_joystick_config() const -> JoystickConfig;
    [[nodiscard]] auto get_kinematic_config() const -> KinematicLimits;

private:
    std::string const zid_;
    std::chrono::milliseconds const refresh_period_;
    JoystickConfig const joy_config_;
    KinematicLimits const kinematic_config_;
};
} // namespace assignment2
#endif // CONFIG_PARSER_HPP_
