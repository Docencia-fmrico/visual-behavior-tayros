
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <geometry_msgs/Twist.h>

#include "behavior_trees/DodgeObstacle.h"
#include "behaviortree_cpp_v3/behavior_tree.h"


#include "ros/ros.h"

namespace behavior_trees
{

DodgeObstacle::DodgeObstacle(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {}), counter_(0)
{
  std::string pub_vel_path =  nh_.param("pub_vel_path", std::string("/mobile_base/commands/velocity"));
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>(pub_vel_path, 1);
}

void
DodgeObstacle::halt()
{
  ROS_INFO("DodgeObstacle halt");
}

BT::NodeStatus
DodgeObstacle::tick()
{
  geometry_msgs::Twist cmd;

  ROS_INFO("DodgeObstacle tick");
  cmd.linear.x = -1;
  cmd.angular.z = 1;


  pub_vel_.publish(cmd);
  return BT::NodeStatus::RUNNING;
}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::DodgeObstacle>("DodgeObstacle");
}