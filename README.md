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
- [Behaviour Tree diagram](#BTdiagram)
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

## Behaviour Tree diagram 

In the following image you can see the Behaviour Tree made in __Groot__:

## Perception

The perception behaviour splits in two main behaviours: __human perception__ and __ball perception__:

### Ball Perception

The ball perception has been made using __HSV color filtering in OpenCV__ and also using __TFs__.

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
Snippet(sensorCallback):
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

The function obtainDistance starts on the start of the array (center) and turns in a range (especificated on the yaml), and then the next for starts on the final
value (center too) and turns the same range on the opposite side.

-----------------------------------------------------------------------
Snippet(obtainDistance):
``` cpp
lidarBumpGo::obtainDistance(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();
  center_ = 0;

  float nearest_distance = 100;
  for (int i = 0; i < range_; i++)
  {
    if (nearest_distance > msg->ranges[i])
    {
      nearest_distance = msg->ranges[i];
      nearest_position_ = 1;  /* Turn right */
    }
  }

  for (int i = size-1; i > (size - range_); i--)
  {
    if (nearest_distance > msg->ranges[i])
    {
        nearest_distance = msg->ranges[i];
        nearest_position_ = 0;  /* Turn left */
    }
  }

  return nearest_distance;
```
-----------------------------------------------------------------------

[Funcionamiento](https://urjc-my.sharepoint.com/:v:/g/personal/a_madinabeitia_2020_alumnos_urjc_es/EVJ-py7o05lFue8xN7RCl_sBfrTUU4eoEiLc4CIp1Q89Qw?e=f0PQHP)

## Logic

## Parameters
We used .yaml and .launch files:

### .yaml

In the yamel we write the parameters we wont to use and the topic we want to publish/subscribe. 

-----------------------------------------------------------------------
Snippet(yaml example):
``` yaml
sub_sensor_path: /scan_filtered

DETECTION_DISTANCE: 0.5
RANGE: 60
```
-----------------------------------------------------------------------

### .launch

It launch the nodes, the parameters and the yamls we want to use in our program. 

*include* is for other launchers, in this program we don't launch the kobuki and lidar drivers 
because if the lidar doesn't works the kuboki works without the sensor and crashes.

*node* is for our nodes and rosparam loads our yaml.

-----------------------------------------------------------------------
Snippet(launch example):
``` launch
<launch>
  <!--<include file="$(find robots)/launch/kobuki_rplidar.launch"/> -->
  
  <node pkg="fsm_bump_go" type="lidarBumpgo_node" name="lidarBumpgo"> 
    <rosparam command="load" file="$(find fsm_bump_go)/config/lidarBumpGoParms.yaml"/>
  </node>
</launch>
```
-----------------------------------------------------------------------

### How to get the params

In this program we got the params in the constructor. We used *.param* because if there is no launcher, it uses a default value.


In the snippet we used a parameter to define wich topic we hace to suuscribe.

-----------------------------------------------------------------------
Snippet(Getting params example):
``` launch
std::string sub_sensor_topic =  n_.param("sub_sensor_path", std::string("/scan_filtered"));
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

