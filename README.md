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
- control_toolbox
- realtime_tools
- nav_msgs
- tf

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

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues).


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

