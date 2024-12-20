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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

  std::vector<hardware_interface::JointHandle> joint_handles_{};
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

private:
  int state_{};
  ros::Time current_time, last_time;
  ros::Time last_cmd_vel_stamp_;
  ros::Duration period_;
  double timeout_{};
  double dt{};
  double current_vel[5]{};
  double target_vel[5]{};
  double ref_vel[5]{};
  double error[5]{};
  double commanded_effort[5]{};
  double acceleration_{};

  double Vx_target{};
  double Vy_target{};
  double W_target{};
  double wheelRadius{};
  double wheelTrack{};
  double wheelBase{};

  double x{};
  double y{};
  double th{};
  double Vx_current{};
  double Vy_current{};
  double W_current{};
  double Vx_last{};
  double Vy_last{};
  double W_last{};

  double power_limit{};
  double effort_coeff_{};
  double velocity_coeff_{};
  double power_offset_{};
  int loop_count_{};
  ros::Subscriber sub_command;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  tf::StampedTransform transform;
  /**< Internal PID controller. */
  std::vector<control_toolbox::Pid> joint_pid_controller_{};
  control_toolbox::Pid front_left_joint_pid_controller_, front_right_joint_pid_controller_,
      back_left_joint_pid_controller_, back_right_joint_pid_controller_;

  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_;

  bool use_global_velocity_{};
  tf::TransformListener tf_listener_;
  geometry_msgs::Vector3Stamped global_vel, base_vel;
  /**
   * \brief Callback from /cmd_vel subscriber
   */
  void set_chassis_state(const geometry_msgs::Twist::ConstPtr& msg);
  /**
   * \brief update each wheel velocity for current_vel
   */
  void get_wheel_vel();
  /**
   * \brief update each wheel velocity for target_vel
   */
  void calc_wheel_vel();
  /**
   * \brief update current_chassis_vel
   */
  void calc_chassis_vel();
  /**
   * \brief control chassis
   */
  void chassis_control();
  /**
   * \brief update odometry
   */
  void compute_odometry();
  /**
   * \brief  odometry publish /odom and update transform
   */
  void updateOdometry();
  /**
   * \brief  chassis acceleration control
   */
  void simple_acceleration_planner();
  /**
   * \brief  state publish
   */
  void controller_state_publish();
  /**
   * \brief  chassis power limit
   */
  void powerLimit();
}; /* hero_chassis_controller */

}  // namespace hero_chassis_controller

#endif