
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
#include "behavior_trees/DetectBall.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "visual_behavior/position.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"
#include <ros/ros.h>
#include "visual_behavior/PIDController.h"
#include "visual_behavior/Movement.h"
#include "std_msgs/Int32.h"

namespace behavior_trees
{

DetectBall::DetectBall(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
DetectBall::halt()
{
  ball_ = false;
}

BT::NodeStatus
DetectBall::tick()
{
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(1), &error_))
  {
    ball_ = true;
    }
  else
  {
    ball_ = false;
  }

  if (ball_)
  {
    setOutput<std::string>("object", "Ball");
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
  factory.registerNodeType<behavior_trees::DetectBall>("DetectBall");
}