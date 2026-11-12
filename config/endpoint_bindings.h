#ifndef RC_AGENT_ENDPOINT_BINDINGS_H
#define RC_AGENT_ENDPOINT_BINDINGS_H

#include "configprocessor.h"

#include <cstdlib>
#include <stdexcept>
#include <string>

// Endpoint names are trusted local libconfig values, never remote protobuf input.
// An omitted value preserves the legacy ROS2 name; a malformed/empty value fails closed.
inline std::string endpointOrDefault(ConfigProcessor &config,
                                    const std::string &key,
                                    const std::string &fallback)
{
    const auto type = config.getType(key);
    if (type == libconfig::Setting::TypeNone) {
        return fallback;
    }
    std::string value;
    if (type != libconfig::Setting::TypeString ||
        config.searchString(key, value) != EXIT_SUCCESS || value.empty()) {
        throw std::invalid_argument("Invalid robot endpoint setting: " + key);
    }
    return value;
}

struct BaseControlEndpointProfile {
    std::string odom_topic;
    std::string cmd_vel_topic;
    std::string cmd_vel_type;
    std::string cmd_vel_frame;
    std::string cmd_vel_smoothed_topic;
    std::string cmd_vel_nav_topic;
};

// Both primary and auxiliary legacy outputs must be scoped to the selected
// robot's locally configured ROS graph. This does not enable v2 teleop.
inline BaseControlEndpointProfile loadBaseControlEndpointProfile(ConfigProcessor &config)
{
    return {
        endpointOrDefault(config, "Amur.API.endpoints.odom_topic", "/odom"),
        endpointOrDefault(config, "Amur.API.endpoints.cmd_vel_topic", "/cmd_vel"),
        endpointOrDefault(config, "Amur.API.endpoints.cmd_vel_type", "twist"),
        endpointOrDefault(config, "Amur.API.endpoints.cmd_vel_frame", "base_link"),
        endpointOrDefault(config, "Amur.API.endpoints.cmd_vel_smoothed_topic", "/cmd_vel_smoothed"),
        endpointOrDefault(config, "Amur.API.endpoints.cmd_vel_nav_topic", "/cmd_vel_nav")
    };
}

#endif // RC_AGENT_ENDPOINT_BINDINGS_H
