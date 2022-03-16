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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include "ball_perception/ColorFilter.h"

namespace ball_perception
{

ColorFilter::ColorFilter()
: it_(nh_)
  {

    ros::NodeHandle nh_params("~");
    hsvValues_->hsv[1] = hupper_ = nh_params.param("HUPPER", 255);
    hsvValues_->hsv[0] = hlower_ = nh_params.param("HLOWER", 0);
    hsvValues_->hsv[3] = supper_ = nh_params.param("SUPEER", 255);
    hsvValues_->hsv[2] = slower_ = nh_params.param("SLOWER", 0);
    hsvValues_->hsv[5] = vupper_ = nh_params.param("VUPPER", 255);
    hsvValues_->hsv[4] = vlower_ = nh_params.param("VLOWER", 255);

    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ColorFilter::imageCb, this);
    image_pub_ = it_.advertise("/hsv/image_filtered", 1);

    
    for (int i=0; i < MAX_CHANNELS; i++)
      initChannel(&hsvValues_[i], i);
      

    publishHSV();
  }

void ColorFilter::initChannel(HSVInfo * hsvinfo, int i)
 {

    char topic_id[256];

    sprintf(topic_id, "/hsv_filter/%d/h", i);
    hsvinfo->hsv_pubs[IDX_h] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
    sprintf(topic_id, "/hsv_filter/%d/H", i);
    hsvinfo->hsv_pubs[IDX_H] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);

    sprintf(topic_id, "/hsv_filter/%d/s", i);
    hsvinfo->hsv_pubs[IDX_s] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
    sprintf(topic_id, "/hsv_filter/%d/S", i);
    hsvinfo->hsv_pubs[IDX_S] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);

    sprintf(topic_id, "/hsv_filter/%d/v", i);
    hsvinfo->hsv_pubs[IDX_v] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
    sprintf(topic_id, "/hsv_filter/%d/V", i);
    hsvinfo->hsv_pubs[IDX_V] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
 }
  

  void ColorFilter::imageCb(const sensor_msgs::Image::ConstPtr& msg)
  {

    hsvValues_[channel_].hsv[IDX_h] = hlower_;
    hsvValues_[channel_].hsv[IDX_H] = hupper_;
    hsvValues_[channel_].hsv[IDX_s] = slower_;
    hsvValues_[channel_].hsv[IDX_S] = supper_;
    hsvValues_[channel_].hsv[IDX_v] = vlower_;
    hsvValues_[channel_].hsv[IDX_V] = vupper_;

    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_imageout = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

    int height = hsv.rows;
    int width = hsv.cols;
    int step = hsv.step;
    int channels = 3;  // RGB


    for (int i=0; i < height; i++ )
      for (int j=0; j < width; j++ )
      {
        int posdata = i * step + j * channels;

        if (!((hsv.data[posdata] >= hsvValues_[channel_].hsv[IDX_h]) &&
             (hsv.data[posdata] <= hsvValues_[channel_].hsv[IDX_H]) &&
            (hsv.data[posdata+1] >= hsvValues_[channel_].hsv[IDX_s]) &&
             (hsv.data[posdata+1] <= hsvValues_[channel_].hsv[IDX_S]) &&
            (hsv.data[posdata+2] >= hsvValues_[channel_].hsv[IDX_v]) &&
             (hsv.data[posdata+2] <= hsvValues_[channel_].hsv[IDX_V])))
        {
          cv_imageout->image.data[posdata] = 0;
          cv_imageout->image.data[posdata+1] = 0;
          cv_imageout->image.data[posdata+2] = 0;
        }
      }

    // Output modified video stream
    image_pub_.publish(cv_imageout->toImageMsg());
  }

  void ColorFilter::publishHSV()
  {
    for (int i=0; i < MAX_CHANNELS; i++)
    {
      std_msgs::Float32 msg;

      msg.data = hsvValues_[i].hsv[IDX_h];
      hsvValues_[i].hsv_pubs[IDX_h].publish(msg);
      msg.data = hsvValues_[i].hsv[IDX_H];
      hsvValues_[i].hsv_pubs[IDX_H].publish(msg);

      msg.data = hsvValues_[i].hsv[IDX_s];
      hsvValues_[i].hsv_pubs[IDX_s].publish(msg);
      msg.data = hsvValues_[i].hsv[IDX_S];
      hsvValues_[i].hsv_pubs[IDX_S].publish(msg);

      msg.data = hsvValues_[i].hsv[IDX_v];
      hsvValues_[i].hsv_pubs[IDX_v].publish(msg);
      msg.data = hsvValues_[i].hsv[IDX_V];
      hsvValues_[i].hsv_pubs[IDX_V].publish(msg);
    }
  }
}