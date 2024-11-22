#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get params
  controller_nh.getParam("WheelRadius", wheelRadius);
  controller_nh.getParam("WheelTrack", wheelTrack);
  controller_nh.getParam("WheelBase", wheelBase);

  // effort_joint init
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // load PID Controller using gains set on parameter server
  front_left_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "front_left_joint_pid"));
  front_right_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "front_right_joint_pid"));
  back_left_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "back_left_joint_pid"));
  back_right_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "back_right_joint_pid"));

  // Start realtime state publisher
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(controller_nh, "state", 1));

  // Start command subscriber
  sub_command_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::set_chassis_state, this);

  return true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // Inverse Kinematics and PID control
  calc_wheel_vel();
  current_vel[1] = front_left_joint_.getVelocity();
  current_vel[2] = front_right_joint_.getVelocity();
  current_vel[3] = back_left_joint_.getVelocity();
  current_vel[4] = back_right_joint_.getVelocity();

  for (int i = 1; i <= 4; ++i)
  {
    error[i] = target_vel[i] - current_vel[i];

    // Set the PID error and compute the PID command with nonuniform time
    // step size. The derivative error is computed from the change in the error
    // and the timestep dt.
    commanded_effort[i] = front_left_joint_pid_controller_.computeCommand(error[i], period);
  }

  front_left_joint_.setCommand(commanded_effort[1]);
  front_right_joint_.setCommand(commanded_effort[2]);
  back_left_joint_.setCommand(commanded_effort[3]);
  back_right_joint_.setCommand(commanded_effort[4]);

  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = current_vel[1];
      controller_state_publisher_->msg_.error = error[1];
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_effort[1];

      double dummy;
      bool antiwindup;
      front_left_joint_pid_controller_.getGains(controller_state_publisher_->msg_.p,
                                                controller_state_publisher_->msg_.i,
                                                controller_state_publisher_->msg_.d,
                                                controller_state_publisher_->msg_.i_clamp, dummy, antiwindup);
      controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  //  if ((time - last_change_).toSec() > 2)
  //  {
  //    state_ = (state_ + 1) % 6;
  //    last_change_ = time;
  //  }
}

void HeroChassisController::set_chassis_state(const geometry_msgs::Twist::ConstPtr& msg)
{
  Vx = msg->linear.x;
  Vy = msg->linear.y;
  W = msg->angular.z;
}

void HeroChassisController::calc_wheel_vel()
{
  target_vel[1] = (Vx - Vy - ((wheelTrack + wheelBase) / 2) * W) / wheelRadius;
  target_vel[2] = (Vx + Vy + ((wheelTrack + wheelBase) / 2) * W) / wheelRadius;
  target_vel[3] = (Vx + Vy - ((wheelTrack + wheelBase) / 2) * W) / wheelRadius;
  target_vel[4] = (Vx - Vy + ((wheelTrack + wheelBase) / 2) * W) / wheelRadius;
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
