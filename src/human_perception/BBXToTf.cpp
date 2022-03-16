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



#include <time.h>
#include <math.h>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ros/ros.h"
#include "visual_behavior/position.h"
#include "human_perception/BBXToTf.h"

namespace human_perception
{
BBXToTf::BBXToTf():
    workingFrameId_("/base_footprint"),
    objectFrameId_("/visual_behavior/person/position"),
    detectedObject_("person")
  {
   
    ROS_INFO("objectFrameId: [%s]", objectFrameId_.c_str());
    ROS_INFO("detectedObject : [%s]", detectedObject_.c_str());

    position_sub = nh.subscribe(objectFrameId_, 1, &BBXToTf::positionCallback, this);
    q.setRPY(0, 0, 0);
  
  }

void BBXToTf::positionCallback(const visual_behavior::position::ConstPtr& position_in){

    result_tf.setOrigin(tf::Vector3(0, 0,0));
    result_tf.setRotation(q);
    result_tf.stamp_ = ros::Time::now();
    result_tf.frame_id_ = workingFrameId_;
    result_tf.child_frame_id_ = objectFrameId_;

    try
    {
        tfBroadcaster_.sendTransform(result_tf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }


  }
}