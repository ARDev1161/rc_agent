#include "map_node.h"
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

MapNode::MapNode(std::shared_ptr<GetMapResponse> mapPtr,
                 std::string mapTopicName,
                 std::string zonesTopicName,
                 std::mutex &grpc_mutex,
                 const rclcpp::NodeOptions & options)
    : rclcpp::Node("map_node", options),
      proto_map_(mapPtr),
      grpc_mutex_(grpc_mutex),
      tf_buffer_(this->get_clock()), // Initialize tf_buffer_ with node's clock
      tf_listener_(tf_buffer_) // Initialize tf_listener_ using tf_buffer_
{
    // Create a subscription for nav_msgs::msg::OccupancyGrid messages
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        mapTopicName,
        10,
        std::bind(&MapNode::map_callback, this, std::placeholders::_1)
    );
    zones_sub_ = this->create_subscription<std_msgs::msg::String>(
        zonesTopicName,
        10,
        std::bind(&MapNode::zones_callback, this, std::placeholders::_1)
    );

    tfUpdateDelayMs = this->declare_parameter("pose_update_ms", tfUpdateDelayMs);
    // Create a timer with a period defined by tfUpdateDelayMs
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tfUpdateDelayMs),
        std::bind(&MapNode::tfUpdateCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "MapNode subscribed to %s and %s",
                mapTopicName.c_str(), zonesTopicName.c_str());
}

void MapNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "MapHandler received new map data");
    {
        std::lock_guard<std::mutex> lock(grpc_mutex_);
        fillOccupancyGridProtobuf(*proto_map_->mutable_map(), msg);
        proto_map_->set_map_seq(++map_seq_);
    }
}

void MapNode::zones_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(grpc_mutex_);
    if (!msg) {
        proto_map_->clear_zone_map();
        proto_map_->set_zone_seq(++zone_seq_);
        return;
    }

#ifdef HAVE_NLOHMANN_JSON
    try {
        const auto json = nlohmann::json::parse(msg->data);
        auto *zone_map = proto_map_->mutable_zone_map();
        zone_map->clear_zones();

        if (json.contains("zones") && json["zones"].is_array()) {
            for (const auto &zone : json["zones"]) {
                auto *proto_zone = zone_map->add_zones();
                proto_zone->set_id(zone.value("id", 0));
                proto_zone->set_type_id(zone.value("type_id", 0));
                proto_zone->set_name(zone.value("name", ""));
                proto_zone->set_type(zone.value("type", ""));

                if (zone.contains("centroid")) {
                    const auto &c = zone["centroid"];
                    proto_zone->mutable_centroid()->set_x(c.value("x", 0.0));
                    proto_zone->mutable_centroid()->set_y(c.value("y", 0.0));
                }
                if (zone.contains("color")) {
                    const auto &c = zone["color"];
                    proto_zone->mutable_color()->set_r(c.value("r", 0));
                    proto_zone->mutable_color()->set_g(c.value("g", 0));
                    proto_zone->mutable_color()->set_b(c.value("b", 0));
                }
                if (zone.contains("chain_code") && zone["chain_code"].is_array()) {
                    for (const auto &pt : zone["chain_code"]) {
                        auto *p = proto_zone->add_chain_code();
                        p->set_x(pt.value("x", 0.0));
                        p->set_y(pt.value("y", 0.0));
                    }
                }
                if (zone.contains("passages") && zone["passages"].is_array()) {
                    for (const auto &passage : zone["passages"]) {
                        auto *proto_passage = proto_zone->add_passages();
                        proto_passage->set_to_id(passage.value("to_id", 0));
                        if (passage.contains("chain_code") && passage["chain_code"].is_array()) {
                            for (const auto &pt : passage["chain_code"]) {
                                auto *pp = proto_passage->add_chain_code();
                                pp->set_x(pt.value("x", 0.0));
                                pp->set_y(pt.value("y", 0.0));
                            }
                        }
                    }
                }
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse zones JSON: %s", e.what());
        proto_map_->clear_zone_map();
    }
#else
    (void)msg;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Zones JSON received, but nlohmann_json is not available; ignoring.");
    proto_map_->clear_zone_map();
#endif

    proto_map_->set_zone_seq(++zone_seq_);
}

void MapNode::fillOccupancyGridProtobuf(OccupancyGrid &proto_map, const nav_msgs::msg::OccupancyGrid::SharedPtr& latest_map)
{
    if (!latest_map) {
        // Initialize proto_map with empty values
        proto_map.set_width(0);
        proto_map.set_height(0);
        proto_map.set_resolution(0.0f);
        proto_map.set_data("");
        proto_map.set_frame_id("");

        // Initialize origin with zero values
        Pose origin;
        origin.set_position_x(0.0f);
        origin.set_position_y(0.0f);
        origin.set_position_z(0.0f);
        origin.set_orientation_x(0.0f);
        origin.set_orientation_y(0.0f);
        origin.set_orientation_z(0.0f);
        origin.set_orientation_w(1.0f);
        proto_map.mutable_origin()->CopyFrom(origin);

        return;
    }

    // Fill in the basic map fields
    proto_map.set_width(latest_map->info.width);
    proto_map.set_height(latest_map->info.height);
    proto_map.set_resolution(latest_map->info.resolution);

    // Copy the map data as bytes
    proto_map.set_data(std::string(
        reinterpret_cast<char*>(latest_map->data.data()),
        latest_map->data.size()
    ));

    // Fill in the origin
    Pose origin;
    origin.set_position_x(latest_map->info.origin.position.x);
    origin.set_position_y(latest_map->info.origin.position.y);
    origin.set_position_z(latest_map->info.origin.position.z);
    origin.set_orientation_x(latest_map->info.origin.orientation.x);
    origin.set_orientation_y(latest_map->info.origin.orientation.y);
    origin.set_orientation_z(latest_map->info.origin.orientation.z);
    origin.set_orientation_w(latest_map->info.origin.orientation.w);
    proto_map.mutable_origin()->CopyFrom(origin);

    // Set the frame_id
    proto_map.set_frame_id(latest_map->header.frame_id);
}

void MapNode::tfUpdateCallback() {
    getRobotPose(*proto_map_->mutable_robotpose(), "map", "base_link");
}

bool MapNode::getRobotPose(Pose &robotPose, std::string mapTF, std::string robotTF)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        // Lookup transform from mapTF to robotTF
        transformStamped = tf_buffer_.lookupTransform(mapTF, robotTF, tf2::TimePointZero,
                                                      std::chrono::milliseconds(42));
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return false;
    }

    // Set translation values
    robotPose.set_position_x(transformStamped.transform.translation.x);
    robotPose.set_position_y(transformStamped.transform.translation.y);
    robotPose.set_position_z(transformStamped.transform.translation.z);

    // Set rotation values
    robotPose.set_orientation_x(transformStamped.transform.rotation.x);
    robotPose.set_orientation_y(transformStamped.transform.rotation.y);
    robotPose.set_orientation_z(transformStamped.transform.rotation.z);
    robotPose.set_orientation_w(transformStamped.transform.rotation.w);

    return true;
}
