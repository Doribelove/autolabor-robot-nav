#include <autolabor_operator_gui/ros_graph_reconnect.h>

#include <ros/master.h>
#include <ros/network.h>
#include <ros/this_node.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace autolabor_operator_gui
{
bool restoreRosGraphRegistration(std::string* error)
{
  const std::string caller = ros::this_node::getName();
  const std::string uri = ros::XMLRPCManager::instance()->getServerURI();
  std::string host;
  uint32_t port = 0;
  if (!ros::network::splitURI(uri, host, port))
  {
    *error = "Invalid local ROS XMLRPC URI";
    return false;
  }
  XmlRpc::XmlRpcClient self(host.c_str(), port, "/");
  XmlRpc::XmlRpcValue caller_args;
  caller_args[0] = caller;
  // The slave API gives the live endpoint types, including RViz's internally
  // owned TF listener/tools. Rebuilding a NodeHandle alone cannot re-register
  // subscriptions still shared by that listener after the master lost them.
  for (const bool subscribing : {false, true})
  {
    XmlRpc::XmlRpcValue endpoints;
    if (!self.execute(subscribing ? "getSubscriptions" : "getPublications",
                      caller_args, endpoints) ||
        endpoints.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        endpoints.size() != 3 || static_cast<int>(endpoints[0]) != 1)
    {
      *error = "Cannot enumerate this GUI's ROS endpoints";
      return false;
    }
    for (int index = 0; index < endpoints[2].size(); ++index)
    {
      XmlRpc::XmlRpcValue args, response, publishers;
      args[0] = caller;
      args[1] = endpoints[2][index][0];
      args[2] = endpoints[2][index][1];
      args[3] = uri;
      if (!ros::master::execute(subscribing ? "registerSubscriber" : "registerPublisher",
                                args, response, publishers, false))
      {
        *error = "Master rejected endpoint registration: " +
                 static_cast<std::string>(args[1]);
        return false;
      }
      if (!subscribing)
        continue;
      XmlRpc::XmlRpcValue update, result;
      update[0] = caller;
      update[1] = args[1];
      update[2].setSize(0);
      // Drop stale transport connections, then consume the publisher list
      // returned by registerSubscriber. This also replays /tf_static and /map.
      for (int reconnect = 0; reconnect < 2; ++reconnect)
      {
        if (reconnect)
          update[2] = publishers;
        if (!self.execute("publisherUpdate", update, result) ||
            result.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            result.size() != 3 || static_cast<int>(result[0]) != 1)
        {
          *error = "Cannot reconnect local subscription: " +
                   static_cast<std::string>(args[1]);
          return false;
        }
      }
    }
  }
  return true;
}
}
