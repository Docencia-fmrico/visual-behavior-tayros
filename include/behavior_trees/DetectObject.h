
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

#ifndef BEHAVIOR_TREES_DETECTOBJECT_H
#define BEHAVIOR_TREES_DETECTOBJECT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "visual_behavior/position.h"

#include <string>

#include "ros/ros.h"

namespace behavior_trees
{

class DetectObject : public BT::ActionNodeBase
{
  public:
    explicit DetectObject(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();
    void personCallback(const visual_behavior::position::ConstPtr& position_in);
    ros::Subscriber person_sub_;
    bool person_;

    float last_detection_time;

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("object")};
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;

    int counter_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_DETECTOBJECT_H