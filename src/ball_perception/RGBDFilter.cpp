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


#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/algorithm/string.hpp>
#include "ball_perception/RGBDFilter.h"

namespace ball_perception
{

RGBDFilter::RGBDFilter()
  {
    initHSV();

    cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &RGBDFilter::cloudCB, this);

    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }
}

void RGBDFilter::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_in, *pcrgb);

    for (int i=0; i < MAX_CHANNELS; i++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
      for (it = pcrgb->begin(); it != pcrgb->end(); ++it)
      {
        if (!std::isnan(it->x))
        {
          pcl::PointXYZHSV hsv;
          pcl::PointXYZRGBtoXYZHSV(*it, hsv);

          if (isValid(i, hsv))
            pcrgb_out->push_back(*it);
        }
      }

      sensor_msgs::PointCloud2 cloud_out;
      pcl::toROSMsg(*pcrgb_out, cloud_out);

      cloud_out.header.frame_id = cloud_in->header.frame_id;
      cloud_out.header.stamp = ros::Time::now();

      hsvFilters_[i].cloud_pub_.publish(cloud_out);
    }
  }

void RGBDFilter::hsvCB(const ros::MessageEvent<std_msgs::Float32 const>& event)
  {
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");

    const std_msgs::Float32::ConstPtr& msg = event.getMessage();

    std::vector<std::string> fields;
    boost::split(fields, topic, boost::is_any_of("/"));

    int channel = atoi(fields[2].c_str());

    if (fields[3] == "h")  hsvFilters_[channel].hsv[IDX_h] =  msg->data;
    if (fields[3] == "H")  hsvFilters_[channel].hsv[IDX_H] =  msg->data;
    if (fields[3] == "s")  hsvFilters_[channel].hsv[IDX_s] =  msg->data;
    if (fields[3] == "S")  hsvFilters_[channel].hsv[IDX_S] =  msg->data;
    if (fields[3] == "v")  hsvFilters_[channel].hsv[IDX_v] =  msg->data;
    if (fields[3] == "V")  hsvFilters_[channel].hsv[IDX_V] =  msg->data;
  }


void RGBDFilter::printHSV()
  {
    for (int i=0; i < MAX_CHANNELS; i++)
    {
      ROS_INFO("[%d] H[%f - %f]\tS[%f - %f]\tV[%f - %f]", i,
          hsvFilters_[i].hsv[IDX_h], hsvFilters_[i].hsv[IDX_H],
          hsvFilters_[i].hsv[IDX_s], hsvFilters_[i].hsv[IDX_S],
          hsvFilters_[i].hsv[IDX_v], hsvFilters_[i].hsv[IDX_V]);
    }
  }

bool RGBDFilter::isValid(int channel, const pcl::PointXYZHSV& hsv)
  {
    pcl::PointXYZHSV hsv_scaled;
    hsv_scaled.s = hsv.s * 100.0;
    hsv_scaled.v = hsv.v * 100.0;
    hsv_scaled.h = hsv.h;

    if ( hsv_scaled.h < hsvFilters_[channel].hsv[IDX_h] ||  hsv_scaled.h > hsvFilters_[channel].hsv[IDX_H] ||
        hsv_scaled.s < hsvFilters_[channel].hsv[IDX_s] ||  hsv_scaled.s > hsvFilters_[channel].hsv[IDX_S] ||
        hsv_scaled.v < hsvFilters_[channel].hsv[IDX_v] ||  hsv_scaled.v > hsvFilters_[channel].hsv[IDX_V])
      return false;
    else
      return true;
  }

void RGBDFilter::initHSV()
  {
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
      hsvFilters_[i].hsv[IDX_h] = hsvFilters_[i].hsv[IDX_s] = hsvFilters_[i].hsv[IDX_v] = 0.0;
      hsvFilters_[i].hsv[IDX_S] = hsvFilters_[i].hsv[IDX_V] = 255.0;
      hsvFilters_[i].hsv[IDX_H] = 360.0;

      char topic_id[256];
      sprintf(topic_id, "/hsv_filter/%d/h", i);
      hsvFilters_[i].hsv_subs[IDX_h] = nh_.subscribe(topic_id, 1, &RGBDFilter::hsvCB, this);
      sprintf(topic_id, "/hsv_filter/%d/H", i);
      hsvFilters_[i].hsv_subs[IDX_H] = nh_.subscribe(topic_id, 1, &RGBDFilter::hsvCB, this);
      sprintf(topic_id, "/hsv_filter/%d/s", i);
      hsvFilters_[i].hsv_subs[IDX_s] = nh_.subscribe(topic_id, 1, &RGBDFilter::hsvCB, this);
      sprintf(topic_id, "/hsv_filter/%d/S", i);
      hsvFilters_[i].hsv_subs[IDX_S] = nh_.subscribe(topic_id, 1, &RGBDFilter::hsvCB, this);

      sprintf(topic_id, "/hsv_filter/%d/v", i);
      hsvFilters_[i].hsv_subs[IDX_v] = nh_.subscribe(topic_id, 1, &RGBDFilter::hsvCB, this);
      sprintf(topic_id, "/hsv_filter/%d/V", i);
      hsvFilters_[i].hsv_subs[IDX_V] = nh_.subscribe(topic_id, 1, &RGBDFilter::hsvCB, this);

      sprintf(topic_id, "/cloud_filtered/%d", i);
      hsvFilters_[i].cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_id, 1, false);
    }
  }
}
