#ifndef _agent_pose_stamped_h_
#define _agent_pose_stamped_h_

#include <pedsim/ped_vector.h>
#include <ros/ros.h>

class AgentPoseStamped {
 public:
  AgentPoseStamped() {};
  AgentPoseStamped(ros::Time timestamp_in, Ped::Tvector pos_in, double theta_in);

  ros::Time timestamp;
  Ped::Tvector pos;
  double theta;

};

#endif
