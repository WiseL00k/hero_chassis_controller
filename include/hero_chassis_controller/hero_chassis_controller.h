#pragma once

#ifndef HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_H_

// ROS
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>

namespace hero_chassis_controller
{

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

  /*!
   * \brief Give set velocity of the joint for next update: revolute (angle) and prismatic (velocity)
   *
   * \param double pos Velocity command to issue
   */
  //void setCommand(double cmd);

  /*!
   * \brief Get latest velocity command to the joint: revolute (angle) and prismatic (velocity).
   */
  //void getCommand(double & cmd);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  //void starting(const ros::Time& time) override;


  /**
   * \brief Get the PID parameters
   */
  //void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /**
   * \brief Set the PID parameters
   */
  //void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

  /**
   * \brief Get the name of the joint this controller uses
   */
  //std::string getJointName();


  double command_{};                                /**< Last commanded velocity. */

private:
  int state_{};
  ros::Time last_change_;

  int loop_count_;
  ros::Subscriber sub_command_;
  /**< Internal PID controller. */
  control_toolbox::Pid front_left_joint_pid_controller_,front_right_joint_pid_controller_,back_left_joint_pid_controller_,back_right_joint_pid_controller_;

  std::unique_ptr<
      realtime_tools::RealtimePublisher<
          control_msgs::JointControllerState> > controller_state_publisher_ ;

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void set_chassis_state(const geometry_msgs::Twist::ConstPtr &msg);

}; /* hero_chassis_controller */

}

#endif