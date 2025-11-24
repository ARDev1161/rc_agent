#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "network/networkcontroller.h"
#include "modules/battery/battery_state_node.h"
#include "modules/map/map_node.h"
#include "modules/basecontrol/basecontrol.h"
#include "modules/nav2/robot_navigator.h"

using namespace Robot;
using namespace std::chrono_literals;

/**
 * @brief Node that publishes the last connected IP address.
 *
 * The AddressSender node publishes a string message containing the last connected IP address
 * retrieved from the NetworkController at a rate of one message per second.
 */
class AddressSender : public rclcpp::Node {
public:
  /**
   * @brief Constructs a new AddressSender node.
   *
   * @param network Shared pointer to a NetworkController instance used to obtain the IP address.
   */
  AddressSender(std::shared_ptr<NetworkController> network) :
  network_(network),
  Node("address_sender")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("remote_control_station/address", 10);

    // Start a timer to publish a message every second
    publish_timer_ = this->create_wall_timer(1s, std::bind(&AddressSender::publish_callback, this));
  }

private:
  /**
   * @brief Timer callback to publish the last connected IP address.
   *
   * This function retrieves the last connected IP from the NetworkController and publishes
   * it as a std_msgs::msg::String message.
   */
  void publish_callback() {
    message_.data = network_->getLastConnectedIP();
    publisher_->publish(message_);
    RCLCPP_DEBUG(this->get_logger(), "Publishing message: %s", message_.data.c_str());
  }

  std::shared_ptr<NetworkController> network_;                        ///< Shared pointer to the NetworkController.
  std_msgs::msg::String message_;                                     ///< Message object used for publishing the IP address.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;     ///< Publisher for the IP address message.
  rclcpp::TimerBase::SharedPtr publish_timer_;                        ///< Timer for periodic publishing.
};

/**
 * @brief Main function that initializes ROS 2 nodes and runs the executor.
 *
 * The main function initializes the ROS 2 environment, creates several nodes (including the network controller,
 * map node, and navigator node), waits for a connection to the server, and then spins a multi-threaded executor.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status.
 */
int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  if (!rclcpp::ok()) {
    std::cerr << "ROS2 initialization failed!" << std::endl;
    return 1;
  }

  // Create shared pointers for Controls, Sensors, and Map data
  auto controlsPtr = std::make_shared<Controls>();
  auto sensorsPtr = std::make_shared<Sensors>();
  auto mapPtr = std::make_shared<GetMapResponse>();

  // Set the broadcast and arping ports
  int bcastPort = 11111;
  int arpingPort = 11112;

  // Create a NetworkController node and start the arping service
  std::shared_ptr<NetworkController> network = std::make_shared<NetworkController>(controlsPtr, sensorsPtr, mapPtr);
  network->startArpingService(bcastPort, arpingPort);

  // Create an AddressSender node that publishes the last connected IP address
  auto addressSenderNode = std::make_shared<AddressSender>(network);

  // Wait until the client successfully connects to the server
  while(true){
    if (!rclcpp::ok())
      return 0;

    if(network->runClient() == 0){
      std::cout << "Connected to server" << std::endl;
      network->stopArpingService();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Wait 10 milliseconds
  }

  // Initialize additional nodes (commented out nodes can be enabled if needed).
//   auto batteryStateNode = std::make_shared<BatteryStateNode>(sensorsPtr->mutable_batterystate(), "battery_state");
  auto mapNode = std::make_shared<MapNode>(mapPtr, "/map", network->getClientInstance()->getMapMutex());
  auto baseControlNode = std::make_shared<BaseControlNode>(controlsPtr->mutable_basecontrol());
  auto navigatorNode = std::make_shared<RobotNavigator>(
    "basic_navigator_node",
    std::shared_ptr<NavCommandRequest>(controlsPtr->mutable_navcontrol(), [](NavCommandRequest*){}),
    std::shared_ptr<NavCommandResponse>(sensorsPtr->mutable_navcontrolstatus(), [](NavCommandResponse*){}),
    network->getClientInstance()->getMutex()
  );

  // Create a MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the nodes to the executor
  executor.add_node(addressSenderNode);
//   executor.add_node(batteryStateNode);
  executor.add_node(mapNode);
  executor.add_node(baseControlNode);
  executor.add_node(navigatorNode);

  // Spin the executor to keep the nodes running
  executor.spin();

  // Shutdown the ROS 2 environment
  rclcpp::shutdown();

  return 0;
}
