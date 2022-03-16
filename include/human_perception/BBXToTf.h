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

#ifndef BALL_PERCEPTION_BBXTOTF_H
#define BALL_PERCEPTION_BBXTOTF_H

#include <time.h>
#include <math.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include "ros/ros.h"
#include "visual_behavior/position.h"

namespace human_perception
{

class BBXToTf
{
public:
  BBXToTf();
	void positionCallback(const visual_behavior::position::ConstPtr& position_in);

private:
  ros::NodeHandle nh;

  visual_behavior::position person_pos;
  ros::Subscriber position_sub;

  tf::TransformBroadcaster tfBroadcaster_;
	tf::StampedTransform  result_tf;
	tf::Quaternion q;

	const float OBJ_PUB_RATE = 10.0;

  std::string detectedObject_;
  std::string objectFrameId_;
  std::string workingFrameId_;
};
}

#endif  // BALL_PERCEPTION_BBXTOTF_H