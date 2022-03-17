
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

#ifndef BEHAVIOR_TREES_DETECTBALL_H
#define BEHAVIOR_TREES_DETECTBALL_H

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

class DetectBall : public BT::ActionNodeBase
{
  public:
    explicit DetectBall(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();
    bool ball_;

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("object")};
    }

  private:
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped bf2object_msg_;
    tf2::Stamped<tf2::Transform> bf2object_;
    std::string error_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_DETECTBall_H