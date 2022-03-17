
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
#include "behavior_trees/DetectObject.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "visual_behavior/position.h"


namespace behavior_trees
{

DetectObject::DetectObject(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  person_sub_ = nh_.subscribe<visual_behavior::position>("/visual_behavior/person/position", 1, &DetectObject::personCallback, this);
}

void
DetectObject::personCallback(const visual_behavior::position::ConstPtr& position_in)
{
  person_ = true;
  ROS_INFO("Human detected");
}

void
DetectObject::halt()
{
  person_ = false;
  ROS_INFO("DetectObject halt");
}

BT::NodeStatus
DetectObject::tick()
{
  ROS_INFO("2 detected");
  

  if (person_)
  {
    setOutput<std::string>("object", "Human");
    person_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    setOutput<std::string>("object", "Nothing");
    return BT::NodeStatus::FAILURE;
  }

}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::DetectObject>("DetectObject");
}