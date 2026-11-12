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


// Trusted Nav2 bindings are loaded together so the running gateway and
// the config tests use exactly the same names and legacy fallback values.
// No action types are selected by the remote operator.
struct Nav2EndpointProfile {
    std::string navigate_to_pose_action;
    std::string navigate_through_poses_action;
    std::string follow_waypoints_action;
    std::string spin_action;
    std::string initial_pose_topic;
};

inline Nav2EndpointProfile loadNav2EndpointProfile(ConfigProcessor &config)
{
    return {
        endpointOrDefault(config, "Amur.API.endpoints.navigate_to_pose_action", "navigate_to_pose"),
        endpointOrDefault(config, "Amur.API.endpoints.navigate_through_poses_action", "navigate_through_poses"),
        endpointOrDefault(config, "Amur.API.endpoints.follow_waypoints_action", "follow_waypoints"),
        endpointOrDefault(config, "Amur.API.endpoints.spin_action", "spin"),
        endpointOrDefault(config, "Amur.API.endpoints.initial_pose_topic", "initialpose")
    };
}

#endif // RC_AGENT_ENDPOINT_BINDINGS_H
