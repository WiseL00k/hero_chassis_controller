#pragma once

#ifndef HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_H_

// ROS
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace hero_chassis_controller
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

private:
  int state_{};
  ros::Time last_change_;
};
} /* hero_chassis_controller */

#endif