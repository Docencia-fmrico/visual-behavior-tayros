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

#ifndef HUMAN_PERCEPTION_BBXDETECTOR_H
#define HUMAN_PERCEPTION_BBXDETECTOR_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "visual_behavior/position.h"
#include <string>

namespace human_perception
{

class BBXDetector
{
public:
  BBXDetector();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

private:
  ros::NodeHandle nh;

  visual_behavior::position person_pos;
  ros::Publisher position_pub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

  const float CAMERA_XCENTER = 340;

  std::string detectedObject_;
  std::string TopicID;
  std::string workingFrameId_;
};
}  // namespace human_perception

#endif  // HUMAN_PERCEPTION_BBXDETECTOR_H
