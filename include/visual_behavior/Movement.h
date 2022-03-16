// Copyright 2022 TayRos
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

#ifndef VISUAL_BEHAVIOUR_MOVEMENT_H
#define VISUAL_BEHAVIOUR_MOVEMENT_H

#include <string>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"
#include "visual_behavior/transforms.h"
#include "ros/ros.h"
#include "visual_behavior/PIDController.h"
#include "std_msgs/Int32.h"
#include "visual_behavior/position.h"

namespace visual_behavior
{

class Movement
{
public:
    Movement();

    void MoveRobot();

    ros::NodeHandle n;

private:

    ros::Publisher vel_pub_;
    ros::Subscriber mov_sub_;
    ros::Subscriber person_sub_;

    void callback(const std_msgs::Int32::ConstPtr& msg);
    void personCallback(const visual_behavior::position::ConstPtr& position_in);
    void get_dist_angle_tf();

    geometry_msgs::TransformStamped bf2object_msg_;
    tf2::Stamped<tf2::Transform> bf2object_;
    std::string error_;

    double dist_;
    double angle_;
    int movement_;

    br2_tracking::PIDController pan_pid_, tilt_pid_;
};
    
}  // namespace visual_behavior

#endif // VISUAL_BEHAVIOUR_MOVEMENT_H