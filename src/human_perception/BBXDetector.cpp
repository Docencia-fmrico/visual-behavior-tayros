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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "human_perception/BBXDetector.h"
#include "visual_behavior/position.h"


namespace human_perception
{

BBXDetector::BBXDetector()
: nh(),
 workingFrameId_("/base_footprint"),
  TopicID("/visual_behavior/person/position"),
  image_depth_sub(nh, "/camera/depth/image_raw", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
  {
    position_pub = nh.advertise<visual_behavior::position>(TopicID, 1);
    // message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub);
    sync_bbx.registerCallback(boost::bind(&BBXDetector::callback_bbx, this, _1, _2));

  }

void BBXDetector::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr_depth;

  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }
  
  for (const auto & box : boxes->bounding_boxes) {
    int px = (box.xmax + box.xmin) / 2;
    int py = (box.ymax + box.ymin) / 2;

    float dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
   
    if(box.Class == "person"){
      std::cerr << box.Class << " at (" << "Dist: "<< dist << " X: " <<px << " Y: " << py << std::endl;
      person_pos.x_position = px;
      person_pos.y_position = py;
      person_pos.z_position = dist;
      person_pos.detected_object = "person";
      person_pos.header.frame_id = workingFrameId_;
      person_pos.header.stamp = ros::Time::now();
    }

  }
}
void BBXDetector::publish_position(){
  position_pub.publish(person_pos);
}
}