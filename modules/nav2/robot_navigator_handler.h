#ifndef ROBOT_NAVIGATOR_HANDLER_H
#define ROBOT_NAVIGATOR_HANDLER_H

#include "navigation.pb.h"

class RobotNavigator;

Navigation::NavCommandResponse createNavCommandResponse(RobotNavigator &navigator_node,
                                                        const Navigation::NavCommandRequest &request);

void processNavCommand(const Navigation::NavCommandRequest &request, RobotNavigator &navigator_node);

#endif // ROBOT_NAVIGATOR_HANDLER_H
