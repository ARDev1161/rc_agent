#ifndef NAV_COMMAND_H
#define NAV_COMMAND_H

#include <memory>
#include "navigation.pb.h"

class RobotNavigator;

/**
 * @brief Base interface for navigation commands.
 *
 * Command instances encapsulate how to act upon the RobotNavigator based on a
 * specific NavCommandRequest payload.
 */
class NavCommand {
public:
  virtual ~NavCommand() = default;
  virtual void execute(RobotNavigator &navigator) = 0;
};

/**
 * @brief Factory that builds concrete navigation commands from requests.
 */
class NavCommandFactory {
public:
  static std::unique_ptr<NavCommand> create(const Navigation::NavCommandRequest &request);
};

#endif // NAV_COMMAND_H
