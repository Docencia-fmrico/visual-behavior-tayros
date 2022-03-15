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

#ifndef BALL_PERCEPTION_RGBDTF_H
#define BALL_PERCEPTION_RGBDTF_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/algorithm/string.hpp>

namespace ball_perception
{

class RGBDtf
{
public:
  RGBDtf();
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

private:
  ros::NodeHandle nh_;

  tf::TransformBroadcaster tfBroadcaster_;
  tf::TransformListener tfListener_;

  tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSub_;

  std::string objectFrameId_;
  std::string workingFrameId_;
  std::string cameraTopicId_;
};
}

#endif  // BALL_PERCEPTION_RGBDTF_H