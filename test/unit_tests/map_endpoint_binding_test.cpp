#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <memory>
#include <stdexcept>
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
