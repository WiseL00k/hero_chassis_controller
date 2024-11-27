

# hero_chassis_controller

## Overview

This is a hero_chassis_controller to control  a virtual hero_robot_chassis in gazebo.

**Keywords:** ROS,ros_control,PID

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: WiseL00k<br />
Affiliation: [WiseL00k](https://github.com/WiseL00k)<br />
Maintainer: WiseL00k, 1656438881@qq.com**

The hero_chassis_controller package has been tested under ROS Noetic on respectively Ubuntu 20.04. 

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [rm_description](https://github.com/YoujianWu/rm_description_for_task.git)
- controller_interface
- hardware_interface
- forward_command_controller
- pluginlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:WiseL00k/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with

	roslaunch hero_chassis_controller run_simulation_and_controller.launch

Or use `mon`:

```
roscore
mon launch hero_chassis_controller run_simulation_and_controller.launch
```

## Config files

* **controllers.yaml** The Params of hero_chassis_controller and joint_state_controller.

## Launch files

* **run_simulation_and_controller.launch:** Simulate the hero chassis in gazebo and load hero_chassis_controller and joint_state_controller.


## Nodes

### ros_package_template

Reads temperature measurements and computed the average.

#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.

#### Published Topics

...

#### Services

* **`get_average`** ([std_srvs/Trigger])

  Returns information about the current average. For example, you can trigger the computation from the console with

  	rosservice call /ros_package_template/get_average

#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

  The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

  The size of the cache.

### teleop_twist_keyboard

Generic Keyboard Teleop for ROS

#### Subscribed Topics

- **`/cmd_vel`** ([geometry_msgs/Twist])

	Expected chassis speed.

#### Launch
Run.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

With custom values.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.9 _turn:=0.8
```

Publishing to a different topic (in this case `my_cmd_vel`).
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=my_cmd_vel
```

#### Usage
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

#### Repeat Rate

If your mobile base requires constant updates on the cmd\_vel topic, teleop\_twist\_keyboard can be configured to repeat the last command at a fixed interval, using the `repeat_rate` private parameter.

For example, to repeat the last command at 10Hz:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0
```

It is _highly_ recommened that the repeat rate be used in conjunction with the key timeout, to prevent runaway robots.

#### Key Timeout

Teleop\_twist\_keyboard can be configured to stop your robot if it does not receive any key presses in a configured time period, using the `key_timeout` private parameter.

For example, to stop your robot if a keypress has not been received in 0.6 seconds:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6
```

It is recommended that you set `key_timeout` higher than the initial key repeat delay on your system (This delay is 0.5 seconds by default on Ubuntu, but can be adjusted).

#### Twist with header
Publishing a `TwistStamped` message instead of `Twist` can be enabled with the `stamped` private parameter. Additionally the `frame_id` of the `TwistStamped` message can be set with the `frame_id` private parameter.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _stamped:=True _frame_id:=base_link
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues).


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

