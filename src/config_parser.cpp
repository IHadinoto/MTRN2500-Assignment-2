// Copyright 2019 Zhihao Zhang License MIT

#include "config_parser.hpp"

#include <iostream>
#include <string>
#include <algorithm>    // std::remove_if
#include <sstream>      // std::istringstream
#include <unordered_map>

namespace assignment2
{
ConfigReader::ConfigReader(std::istream & config_file)
{
	//Get line by line of config_file
	std::string key; 
	std::string value; 

	int n_lines = 0; 
	while(std::getline(config_file, key, ':') && std::getline(config_file, value)){
		std::cout << "Line " << ++n_lines << ": " << key << ":" << value << std::endl; 
		//Purge whitespace from lines 

		key.erase(remove_if(key.begin(), key.end(), isspace),key.end());
		value.erase(remove_if(value.begin(), value.end(), isspace),value.end());

		//Add to unordered map config_
		config_[key] = value; 
	} 
	
	//Print out unordered map config_ 
	for (const auto & [ key, value ] : config_)
		std::cout << "key: \"" << key << "\", " 
			<< "value: \"" << value << "\"" << std::endl;
}

auto ConfigReader::find_config(std::string const & key,
    std::string const & default_value) const -> std::string
{
	auto config_iterator = config_.find(key); 
	if (!(config_iterator == config_.end())) return config_iterator->second; 
	return default_value;
}

ConfigParser::ConfigParser(ConfigReader const & config)
    : zid_{config.find_config("zid", std::string{"z0000000"})}
	, refresh_period_{std::chrono::duration<long>(stol(config.find_config("refresh_rate", std::string{"10"})))}
    , joy_config_{  stoul(config.find_config("speed_plus_axis", std::string{"1"})),
					stoul(config.find_config("speed_minus_axis", std::string{"2"})), 
					stoul(config.find_config("steering_axis", std::string{"3"})),
					stod(config.find_config("steering_deadzone", std::string{"0.1"})),
					stod(config.find_config("speed_deadzone", std::string{"0.5"}))}
    , kinematic_config_{stod(config.find_config("max_linear_speed", std::string{"1"})),
						stod(config.find_config("max_angular_speed", std::string{"45"})), 
						stod(config.find_config("max_linear_acceleration", std::string{"0.5"})),
						stod(config.find_config("max_angular_acceleration", std::string{"10"}))
    }	
{
	

}

auto ConfigParser::get_zid() const -> std::string { return zid_; }

auto ConfigParser::get_refresh_period() const -> std::chrono::milliseconds
{
    return refresh_period_;
}

auto ConfigParser::get_joystick_config() const -> JoystickConfig
{
    return joy_config_;
}

auto ConfigParser::get_kinematic_config() const -> KinematicLimits
{
    return kinematic_config_;
}
} // namespace assignment2
