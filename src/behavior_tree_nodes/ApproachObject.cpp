
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

#include "behavior_trees/ApproachObject.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "std_msgs/Int32.h"
#include "ros/ros.h"

namespace behavior_trees
{

ApproachObject::ApproachObject(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  std::string sub_movement_topic =  nh_.param("movement_topic", std::string("/visual_behavior/move_tf"));
  mov_pub_ = nh_.advertise<std_msgs::Int32>(sub_movement_topic, 1);
}

void
ApproachObject::halt()
{
  move_.data = 0;
  mov_pub_.publish(move_);
}

BT::NodeStatus
ApproachObject::tick()
{
  std::string Target = getInput<std::string>("target").value();
  printf("%s\n", Target.c_str());
  move_.data = 1;
  mov_pub_.publish(move_);

  return BT::NodeStatus::RUNNING;

}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::ApproachObject>("ApproachObject");
}