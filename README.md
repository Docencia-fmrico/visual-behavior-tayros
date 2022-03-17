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

FinalBumpGo inherits from the SensorGo base class and implements a sensorCallback function that allows it to communicate with the kobuki bumpers to take the state and indicate it to the state machine. 

If it detects that the left bumper has been activated, it will indicate a right turn and if the right or central bumper is activated, it will turn left. 

-----------------------------------------------------------------------
Snippet(sensorCallback):
``` cpp
void
FinalBumpGo::sensorCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  pressed_ = msg->state;
  bumper_ = msg->bumper;

  if (bumper_ == LEFT)
  {
    turn_direction_ = TURN_RIGHT;
  }

  // If bumper detects left or center, the robot turn left
  else
  {
    turn_direction_ = TURN_LEFT;
  }
}
```
-----------------------------------------------------------------------

## Movement

lidarBumpGo inherits too from the SensorGo base class. In the sensorCallback function, it calls a function that returns the nearest distance the sensor is taking.

Then, if the distance is less than the value *MIN_DISTANCE_* it changes the attribute *pressed_*, before that, the program decide if turn left or turn right depending the side of the nearest distance.

In the program the range is 60; (60 * 0.008) = 0.48 rad. In the tests of the real robot was the best value. 

-----------------------------------------------------------------------
Snippet(sensorCallback):
``` cpp
void
lidarBumpGo::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float nearest_distance = obtainDistance(msg);

  if (nearest_distance <= MIN_DISTANCE_)
  {
    pressed_ = true;
  }
  else
  {
    pressed_ = false;
  }

  if (nearest_position_ == LEFT)
  {
    turn_direction_ = TURN_RIGHT;
  }

  // If bumper detects left or center, the robot turn left
  if (nearest_position_ == RIGHT)
  {
    turn_direction_ = TURN_LEFT;
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

