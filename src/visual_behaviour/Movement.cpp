// Copyright 2021 Intelligent Robotics Lab
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
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"
#include "visual_behaviour/transforms.h"
#include "ros/ros.h"
#include "visual_behaviour/PIDController.h"
#include "visual_behaviour/Movement.h"

namespace visual_behaviour
{

Movement::Movement()
: pan_pid_(0.0, 1.0, 0.0, 0.3),
  tilt_pid_(-1.0, 1.0, 0.0, 0.1)
{
  vel_pub_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
}

void
Movement::MoveRobot()
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(1), &error_))
    {
      bf2object_msg_ = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

      tf2::fromMsg(bf2object_msg_, bf2object_);

      dist_ = bf2object_.getOrigin().length();
      angle_ = atan2(bf2object_.getOrigin().y(),bf2object_.getOrigin().x());

      double control_pan = pan_pid_.get_output(angle_);
      double control_tilt = tilt_pid_.get_output(dist_);

      ROS_INFO("base_footprint -> object [%lf, %lf] dist = %lf angle = %lf control_pan = %lf control_tilt = %lf ago",
        bf2object_.getOrigin().x(),
        bf2object_.getOrigin().y(),
        dist_,
        angle_, 
        control_pan,
        control_tilt,
        (ros::Time::now() - bf2object_.stamp_).toSec());

      geometry_msgs::Twist vel_msgs;
      vel_msgs.linear.x = dist_ - control_tilt -1.0;
      vel_msgs.angular.z = angle_ - control_pan;
      vel_pub_.publish(vel_msgs);
    }
    else
    {
      ROS_ERROR("%s", error_.c_str());
    }
}

}  // namespace visual_behaviour