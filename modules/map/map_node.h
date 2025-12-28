#ifndef MAP_NODE_HPP
#define MAP_NODE_HPP

#include <mutex>
#include <memory>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "robot.grpc.pb.h"

using namespace map_service;

/**
 * @brief Node class for handling map data and TF updates.
 *
 * This class subscribes to a ROS topic publishing nav_msgs::msg::OccupancyGrid messages,
 * processes the map data to fill a protobuf OccupancyGrid, and periodically updates the
 * robot pose using TF2.
 */
class MapNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for MapNode.
     *
     * Initializes the MapNode with the provided GetMapResponse shared pointer,
     * subscribes to the specified map topic, and sets up a timer for TF updates.
     *
     * @param mapPtr Shared pointer to the GetMapResponse object which will be filled.
     * @param mapTopicName Name of the ROS 2 topic to subscribe to (e.g., "/map").
     * @param grpc_mutex Reference to a mutex used for thread-safe operations on gRPC data.
     * @param options Optional rclcpp::NodeOptions for node configuration.
     */
    explicit MapNode(std::shared_ptr<GetMapResponse> mapPtr,
                     std::string mapTopicName,
                     std::string zonesTopicName,
                     std::mutex &grpc_mutex,
                     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    /**
     * @brief Callback function for processing incoming map messages.
     *
     * This function is called whenever a new nav_msgs::msg::OccupancyGrid message
     * is received. It calls fillOccupancyGridProtobuf() to update the protobuf map.
     *
     * @param msg Shared pointer to a nav_msgs::msg::OccupancyGrid message.
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void zones_callback(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Callback function for periodic TF updates.
     *
     * This function is triggered by a timer to update the robot pose by calling getRobotPose().
     */
    void tfUpdateCallback();

    /**
     * @brief Retrieves the current robot pose using TF2.
     *
     * Looks up the transform from the specified map frame to the robot frame and fills
     * the provided robotPose protobuf message.
     *
     * @param robotPose Reference to a Pose protobuf message to be filled.
     * @param mapTF Name of the map frame (e.g., "map").
     * @param robotTF Name of the robot frame (e.g., "base_link").
     * @return bool Returns true if the transformation was successfully retrieved, otherwise false.
     */
    bool getRobotPose(Pose &robotPose, std::string mapTF, std::string robotTF);

    /**
     * @brief Fills a protobuf OccupancyGrid based on the latest ROS OccupancyGrid.
     *
     * This function converts the ROS OccupancyGrid message data into the corresponding
     * fields of a protobuf OccupancyGrid object.
     *
     * @param proto_map Reference to the protobuf OccupancyGrid that will be filled.
     * @param latest_map Shared pointer to the latest nav_msgs::msg::OccupancyGrid message.
     */
    void fillOccupancyGridProtobuf(OccupancyGrid &proto_map, const nav_msgs::msg::OccupancyGrid::SharedPtr& latest_map);

    /// Subscriber for nav_msgs::msg::OccupancyGrid messages
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    /// Subscriber for mapannotator zone data
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zones_sub_;

    /// Delay (in milliseconds) for TF update timer
    int tfUpdateDelayMs = 200;
    /// Incremented on each incoming map update.
    std::uint64_t map_seq_{0};
    /// Incremented on each incoming zones update.
    std::uint64_t zone_seq_{0};

    /// Shared pointer to the GetMapResponse protobuf object
    std::shared_ptr<GetMapResponse> proto_map_;

    /// Reference to a mutex for protecting gRPC-related data
    std::mutex &grpc_mutex_;

    // TF2 members:
    /// Buffer for TF2 transforms
    tf2_ros::Buffer tf_buffer_;
    /// Listener for TF2, using tf_buffer_
    tf2_ros::TransformListener tf_listener_;
    /// Timer for triggering TF updates
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MAP_NODE_HPP
