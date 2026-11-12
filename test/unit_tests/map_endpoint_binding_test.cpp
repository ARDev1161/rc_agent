#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <functional>
#include <cstdint>
#include "modules/map/map_node.h"

class MapEndpointBindingTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(MapEndpointBindingTest, LegacyProfile)
{
    auto map = std::make_shared<GetMapResponse>();
    auto mutex = std::make_shared<std::mutex>();
    MapNode node(map, "/map", "/mapannotator/rc_agent_zones", mutex);
    EXPECT_EQ(node.get_subscriptions_info_by_topic("/map").size(), 1u);
    EXPECT_EQ(node.get_parameter("map.tf_map_frame").as_string(), "map");
    EXPECT_EQ(node.get_parameter("map.tf_robot_frame").as_string(), "base_link");
}

TEST_F(MapEndpointBindingTest, AlternateRobotProfile)
{
    auto map = std::make_shared<GetMapResponse>();
    auto mutex = std::make_shared<std::mutex>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("map.tf_map_frame", "robot_b/map"),
        rclcpp::Parameter("map.tf_robot_frame", "robot_b/base_link")
    });
    MapNode node(map, "/robot_b/map", "/robot_b/zones", mutex, options);
    EXPECT_EQ(node.get_subscriptions_info_by_topic("/robot_b/map").size(), 1u);
    EXPECT_EQ(node.get_subscriptions_info_by_topic("/robot_b/zones").size(), 1u);
    EXPECT_EQ(node.get_parameter("map.tf_map_frame").as_string(), "robot_b/map");
    EXPECT_EQ(node.get_parameter("map.tf_robot_frame").as_string(), "robot_b/base_link");
}

TEST_F(MapEndpointBindingTest, EmptyFrameRejected)
{
    auto map = std::make_shared<GetMapResponse>();
    auto mutex = std::make_shared<std::mutex>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("map.tf_robot_frame", "")});
    EXPECT_THROW(MapNode(map, "/map", "/zones", mutex, options), std::invalid_argument);
}


// P2: simultaneous ROS2 map profiles must not cross-populate their protobuf
// snapshots. A later update to A must leave B's sequence and payload intact.
TEST_F(MapEndpointBindingTest, TwoProfilesKeepOccupancyGridSnapshotsIsolated)
{
    using Grid = nav_msgs::msg::OccupancyGrid;
    auto snapshot_a = std::make_shared<GetMapResponse>();
    auto snapshot_b = std::make_shared<GetMapResponse>();
    auto mutex_a = std::make_shared<std::mutex>();
    auto mutex_b = std::make_shared<std::mutex>();
    auto node_a = std::make_shared<MapNode>(
        snapshot_a, "/profile_a/map", "/profile_a/zones", mutex_a);
    auto node_b = std::make_shared<MapNode>(
        snapshot_b, "/profile_b/map", "/profile_b/zones", mutex_b);
    auto publisher_node = std::make_shared<rclcpp::Node>("two_profile_map_publisher");
    auto publisher_a = publisher_node->create_publisher<Grid>("/profile_a/map", 10);
    auto publisher_b = publisher_node->create_publisher<Grid>("/profile_b/map", 10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_a);
    executor.add_node(node_b);
    executor.add_node(publisher_node);

    auto make_grid = [](int value) {
        Grid grid;
        grid.header.frame_id = "map";
        grid.info.width = 1;
        grid.info.height = 1;
        grid.info.resolution = 0.1f;
        grid.info.origin.orientation.w = 1.0;
        grid.data.push_back(static_cast<int8_t>(value));
        return grid;
    };
    const auto grid_a = make_grid(11);
    const auto grid_b = make_grid(22);
    auto spin_until = [&](const std::function<bool()> &predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            publisher_a->publish(grid_a);
            publisher_b->publish(grid_b);
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        return predicate();
    };
    ASSERT_TRUE(spin_until([&] {
        return snapshot_a->map_seq() > 0 && snapshot_b->map_seq() > 0;
    }));
    EXPECT_EQ(static_cast<unsigned char>(snapshot_a->map().data()[0]), 11u);
    EXPECT_EQ(static_cast<unsigned char>(snapshot_b->map().data()[0]), 22u);

    // Drain queued initial samples before checking that only A changes.
    for (int i = 0; i < 20; ++i) executor.spin_some();
    const auto b_sequence = snapshot_b->map_seq();
    const auto updated_a = make_grid(33);
    const auto a_sequence = snapshot_a->map_seq();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (snapshot_a->map_seq() == a_sequence &&
           std::chrono::steady_clock::now() < deadline) {
        publisher_a->publish(updated_a);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    ASSERT_GT(snapshot_a->map_seq(), a_sequence);
    EXPECT_EQ(static_cast<unsigned char>(snapshot_a->map().data()[0]), 33u);
    EXPECT_EQ(snapshot_b->map_seq(), b_sequence);
    EXPECT_EQ(static_cast<unsigned char>(snapshot_b->map().data()[0]), 22u);
}
