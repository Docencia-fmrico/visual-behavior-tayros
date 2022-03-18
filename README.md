[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870050&assignment_repo_type=AssignmentRepo)
# visual_behaviour

<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/readme/resources/xtion.jpg?raw=true" alt="explode"></a>
</div>

<h3 align="center"> Viusal Behavior </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
<img width=90px src="https://img.shields.io/badge/team-TayRos-yellow" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Perception](#perception)
- [Movement](#movement)
- [Logic](#logic)
- [Behavior Tree diagram](#behavior-tree-diagram)
- [Parameters](#parameters)
- [Team](#team)
- [Licencia](#licencia)

## Perception

The perception of this practice is divided into two libraries: __human perception__ and __ball perception__:

Both designed with the idea of maintaining a modular form and allowing an easy extension of these, like all parts of the project.

### Ball Perception

For the perception of the ball, a **HSV color filter** has been used, which is transformed into a **point cloud** in order to obtain the 3D point and thus convert the detected object into a **tf**. To achieve this we have relied on the **OpenCV** library.

To store the filter values so that they can be easily modified we have created a configuration file (**color_filter.yaml**). 

-----------------------------------------------------------------------
Snippet(cloudCB):
``` cpp
      ...
      x = x/c;
      y = y/c;
      z = z/c;


      tf::StampedTransform transform;
      transform.setOrigin(tf::Vector3(x, y, z));
      transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

      transform.stamp_ = ros::Time::now();
      transform.frame_id_ = workingFrameId_;
      transform.child_frame_id_ = objectFrameId_;

      try
      {
        tfBroadcaster_.sendTransform(transform);
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting         callback");
        return;
```
-----------------------------------------------------------------------

### Human Perception

Human perception has been made been using **Bounding Boxes** from **Darknet ROS library**.

We have also **modified certain parameters of the Darket Ros** library to improve performance and accuracy. We limit the detection to only when its probability exceeds 0.6 and only detecting people. If we want to detect more objects we should only add these to **yolov2-tiny.yaml**.

To communicate this node with the movement and behavior tree we have created a **custom message(position.msg)** which allows us to publish the **distance** to the object detected with darknet together with the **angle error** between our robot and it (scale: -1 , 1).

It also adds a **string indicating the type of object** (future implementations) and a **std_msgs header** to store extra information such as the time stamp.

Position.msg:
-----------------------------------------------------------------------
``` cpp
std_msgs/Header header

string detected_object
float64  angle
float64  distance
```
-----------------------------------------------------------------------


-----------------------------------------------------------------------
Snippet(callback_bbx):
``` cpp
if(box.Class == "person"){
      std::cerr << box.Class << " at (" << "Dist: "<< dist << " Ang: " <<ang << std::endl;
      person_pos.angle = ang;
      person_pos.distance = dist;
      person_pos.detected_object = "person";
      person_pos.header.frame_id = workingFrameId_;
      person_pos.header.stamp = ros::Time::now();
      position_pub.publish(person_pos);
    }
```
-----------------------------------------------------------------------

## Movement

In this section you will find how movement behaviour works.

The __movement node__ creates a movement object and in each loop of the while(ros::ok()) calls to MoveRobot(). This function manage a little logical behaviour based in the following points:

- If movement_ value is STOP(0), no object has been detected.

- If movement_ value is BALL(1), the detected object from the perception is ball

- If movement_ value is PERSON(2), the detected object is a human.

Also, if movement is not 0, the __PID Controller values__ are calculated in order to apply to the velocity.

-----------------------------------------------------------------------
Snippet(MoveRobot):
``` cpp
{
  if (movement_ == 1)
  {
      get_dist_angle_tf();
  }

  if (movement_ != 0)
  {
    double control_pan = pan_pid_.get_output(angle_);
    double control_tilt = tilt_pid_.get_output(dist_);

    geometry_msgs::Twist vel_msgs;
    vel_msgs.linear.x = (dist_ - 1.0) * 0.1;
    vel_msgs.angular.z = angle_ - control_pan;
    vel_pub_.publish(vel_msgs);
  }
}
```
-----------------------------------------------------------------------

The following TF Listener made in get_dist_angle_tf() function "listens" to the TFs published before by the TF Broadcaster:

-----------------------------------------------------------------------
Snippet(get_dist_angle_tf):
``` cpp
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(1), &error_))
 {
    bf2object_msg_ = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

    tf2::fromMsg(bf2object_msg_, bf2object_);

    dist_ = bf2object_.getOrigin().length();
    angle_ = atan2(bf2object_.getOrigin().y(),bf2object_.getOrigin().x());
    }
    else
    {
      ROS_ERROR("%s", error_.c_str());
    }
}
```
-----------------------------------------------------------------------

If the object is detected but its too far away from the robot, dist value is NaN. We set the dist_ value (that is used in the velocity) to 1, in order to avoid the robot going backwards when we set the value for the linear velocity.

-----------------------------------------------------------------------
Snippet(personCallback):
``` cpp
{
  if(movement_ == 2){
    dist_ = position_in->distance;
    if(isnan(dist_)){
      dist_ = 1;
    }
    angle_ = position_in->angle;
    ROS_INFO("PERSON DETECTED!");
  }
}
```
-----------------------------------------------------------------------

[Funcionamiento](https://urjc-my.sharepoint.com/:v:/g/personal/a_madinabeitia_2020_alumnos_urjc_es/EVJ-py7o05lFue8xN7RCl_sBfrTUU4eoEiLc4CIp1Q89Qw?e=f0PQHP)

## Logic

## Behavior Tree Diagram 

In the following image you can see the Behaviour Tree made in __Groot__:

### Follow Human

<div align="center">
<img width=600px height=655px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/readme/resources/BTfollowHuman.png?raw=true" alt="explode"></a>
</div>

-----------------------------------------------------------------------
**Execution**:
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/main/resources/LaunchOnlyPerson.gif?raw=true" alt="explode"></a>
</div>

-----------------------------------------------------------------------

The first objective was to make the robot follow a human using Bounding Boxes from __DarknetROS__

The BT is always running, the base is a reactive sequence, under the sequence we have a fallback wich tries to detect the human, it writes in a port called toFollow "Human", if the node doesn't detect nothig the robots starts turning. Turn always returns SUCCESS. When one human is detected the reactive sequnce is calling in loop to detect object and approbachObject. ApprobachObject always is reading the port toFollow and creates a topic with a int to indicate what's the object detect_object is detecting (An human)

You can watch a short demonstration in the following [video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/EfThSuSHDLZBiZ6kEb4ogJ0Bm4t4KhwBVoKoPJDNyGEI_Q?e=aUXScF)

### Follow Ball

<div align="center">
<img width=600px height=655px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/readme/resources/BTfollowBall.png?raw=true" alt="explode"></a>
</div>

-----------------------------------------------------------------------
**Execution**:
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/main/resources/LaunchOnlyBall.gif?raw=true" alt="explode"></a>
</div>

-----------------------------------------------------------------------

Another objective of this project was to make the robot able to follow a ball using __TFs__ and __color filtering__.

This BT is the same that the previous one. The difference is that this BT detects a Ball, so DetectBall writes Ball in the port toFollow.

You can watch a short demonstration in the following [video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/ESpUkj2sZzxAj7NWjWMu_iUB9z5rCxZEkpkqZmBAzyWIUA?e=uqVJLe)

### Human & Ball

<div align="center">
<img width=600px height=655px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/readme/resources/BTFollowBoth.png?raw=true" alt="explode"></a>
</div>

-----------------------------------------------------------------------
**Execution**:
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/visual-behavior-tayros/blob/main/resources/LaunchPersonAndBall.gif?raw=true" alt="explode"></a>
</div>

-----------------------------------------------------------------------

The last objective was to make the robot able to follow both ball and human, but giving preference to the ball.

In this BT we added the node DetectBall and DetectHuman in the Fallback, if we dont find the ball we search the human, else the robots start turning searching one of both. The one that finds his objective writes in the same port toFollow. ApprobachObject reads the port and publish the following object.

You can watch a short demonstration in the following [video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/EVsVHefVfa5KqGEOiqivIgIBhYkk-oGhJcOQpwDXBEjMWg?e=eNcInM)

## Parameters
We used .yaml and .launch files:

### .yaml

In the yamel we write the parameters we wont to use and the topic we want to publish/subscribe. 

Here an example of the HSV color values stablished for the color filter:

-----------------------------------------------------------------------
Snippet(color_filter.yaml):
``` yaml
HUPPER: 124
HLOWER: 43
SUPEER: 255
SLOWER: 56
VUPPER: 255
VLOWER: 83
```
-----------------------------------------------------------------------

We also make .yaml files for following objects. Here an example of follow_ball.yaml:

-----------------------------------------------------------------------
Snippet(follow_ball.yaml):
``` yaml
detection_topic: /ejemplo
movement_topic: /mobile_base/commands/velocity

turning_vel: 0.5
```
-----------------------------------------------------------------------

### .launch

-----------------------------------------------------------------------
Snippet(follow_person_and_ball.launch):
``` launch
<launch>   

    <include file="$(find visual_behavior)/launch/human_perception.launch"/> 
    <include file="$(find visual_behavior)/launch/ball_perception.launch"/> 

    <node 
       pkg="visual_behavior" type="movement_node" name="Movement" output="screen">
    </node>

    <node 
	pkg="visual_behavior" type="follow_2objects" name="BTFinal" output="screen">
    </node>

</launch>
```
-----------------------------------------------------------------------

## Team
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/bump-and-go-with-fsm-tayros/blob/main/resources/grupo.jpg?raw=true"  alt="explode"></a>
</div>
<h5 align="center">TayRos 2022</h5
  
- [Saul Navajas](https://github.com/SaulN99)
- [Guillermo Alcocer](https://github.com/GuilleAQ)
- [Adrian Madinabeitia](https://github.com/madport)
- [Ivan Porras](https://github.com/porrasp8)

## Licencia 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(TayRos) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0

