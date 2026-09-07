#ifndef AUTOLABOR_OPERATOR_GUI_ROS_GRAPH_RECONNECT_H
#define AUTOLABOR_OPERATOR_GUI_ROS_GRAPH_RECONNECT_H

#include <string>

namespace autolabor_operator_gui
{
// Restore only this process's existing ROS endpoints after a master restart.
// No messages/goals are published. Latched inputs are reconnected for replay.
bool restoreRosGraphRegistration(std::string* error);
}

#endif
