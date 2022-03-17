[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870050&assignment_repo_type=AssignmentRepo)
# visual_behaviour

<div align="center">
<img width=400px src="https://github.com/Docencia-fmrico/bump-and-go-with-fsm-tayros/blob/main/resources/kuboki.jpg?raw=true" alt="explode"></a>
</div>

<h3 align="center">Bump And Go </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [How to execute the programs](#How-to-execute-the-programs)
- [Behavior Tree diagram](#Behavior-Tree-Diagram)
- [Perception](#perception)
- [Movement](#movement)
- [Logic](#logic)
- [Parameters](#Parameters)
- [Team](#team)
- [Licencia](#licencia)


## How to execute the programs

<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/bump-and-go-with-fsm-tayros/blob/main/resources/FibalBumpGo_launch.gif?raw=true" alt="explode"></a>
</div>

First connect the base and the camera, then :
-----------------------------------------------------------------------
Snippet(launch base):
``` bash
roslaunch kobuki_node minimal.launch # Driver of the kobuki
roslaunch LANZAR_TODO
```
-----------------------------------------------------------------------

## Behavior Tree Diagram 

In the following image you can see the Behaviour Tree made in __Groot__:

### Follow Human

The first objective was to make the robot follow a human using Bounding Boxes from __DarknetROS__

You can watch a short demonstration in the following [video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/EfThSuSHDLZBiZ6kEb4ogJ0Bm4t4KhwBVoKoPJDNyGEI_Q?e=aUXScF)

### Follow Ball

Another objective of this project was to make the robot able to follow a ball using __TFs__ and __color filtering__.

You can watch a short demonstration in the following [video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/ESpUkj2sZzxAj7NWjWMu_iUB9z5rCxZEkpkqZmBAzyWIUA?e=uqVJLe)

### Human & Ball

The last objective was to make the robot able to follow both ball and human, but giving preference to the ball.

You can watch a short demonstration in the following [video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/EVsVHefVfa5KqGEOiqivIgIBhYkk-oGhJcOQpwDXBEjMWg?e=eNcInM)

## Perception

The perception behaviour splits in two main behaviours: __human perception__ and __ball perception__:

### Ball Perception

The ball perception has been made using __HSV color filtering in OpenCV__ and also using __TFs__.

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

Human perception has been made been using Bounding Boxes from Darknet ROS library.

Once the camera detects a Human, we can take plenty of usefull information like distance, angle, frame_id or time stamp.

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

We publish this information in order to change from Bounding Boxes to TFs:

-----------------------------------------------------------------------
Snippet(positionCallback):
``` cpp
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
```
-----------------------------------------------------------------------

The TF Broadcaster allow us to listen the TFs to make the movement behaviour.

## Movement

In this section you will find how movement behaviour works.

The __movement node__ creates a movement object and in each loop of the while(ros::ok()) calls to MoveRobot(). This function manage a little logical behaviour based in the following points:

- If movement_ value is 1, the detected object from the perception is ball

- If movement value is 2, the detected object is a human.

- If movement value is 0, no object has been detected.

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
Snippet(perception launch):
``` launch
<launch>

	<node pkg="visual_behavior" type="color_filter_node" name="PerceptionFilter" output="screen">
		<rosparam command="load" file="$(find visual_behavior)/config/color_filter.yaml"/>
	</node>

	<node 
		pkg="cameras_cpp" type="nodo_rgbd_filter" name="PerceptionCloud" output="screen">
	</node>

	<node 
		pkg="visual_behavior" type="rgbd_tf_node" name="PerceptionTf" output="screen">
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

