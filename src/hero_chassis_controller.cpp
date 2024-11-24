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
  controller_nh.param("Use_Global_Vel", use_global_velocity_, false);

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
  sub_command = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::set_chassis_state, this);
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);
  last_time = ros::Time::now();
  return true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  current_time = time;
  period_ = period;

  // get each current_wheel_vel
  current_vel[1] = front_left_joint_.getVelocity();
  current_vel[2] = front_right_joint_.getVelocity();
  current_vel[3] = back_left_joint_.getVelocity();
  current_vel[4] = back_right_joint_.getVelocity();

  // odometry
  calc_chassis_vel();
  compute_odometry();
  updateOdometry();

  if (use_global_velocity_)
  {
    global_vel.header.frame_id = "odom";
    //    global_vel.header.stamp = current_time - ros::Duration(0.004);
    global_vel.header.stamp = ros::Time(0);
    global_vel.vector.x = Vx_target;
    global_vel.vector.y = Vy_target;
    global_vel.vector.z = 0.0;

    tf_listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(3.0));
    //        tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), transform);
    tf_listener_.transformVector("base_link", global_vel, base_vel);

    Vx_target = base_vel.vector.x;
    Vy_target = base_vel.vector.y;
  }

  // Inverse Kinematics and PID control
  calc_wheel_vel();

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
      controller_state_publisher_->msg_.set_point = command;
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
  last_time = current_time;
}

void HeroChassisController::set_chassis_state(const geometry_msgs::Twist::ConstPtr& msg)
{
  Vx_target = msg->linear.x;
  Vy_target = msg->linear.y;
  W_target = msg->angular.z;
}

void HeroChassisController::calc_wheel_vel()
{
  target_vel[1] = (Vx_target - Vy_target - ((wheelTrack + wheelBase) / 2) * W_target) / wheelRadius;
  target_vel[2] = (Vx_target + Vy_target + ((wheelTrack + wheelBase) / 2) * W_target) / wheelRadius;
  target_vel[3] = (Vx_target + Vy_target - ((wheelTrack + wheelBase) / 2) * W_target) / wheelRadius;
  target_vel[4] = (Vx_target - Vy_target + ((wheelTrack + wheelBase) / 2) * W_target) / wheelRadius;
}

void HeroChassisController::calc_chassis_vel()
{
  Vx_current = (current_vel[1] + current_vel[2] + current_vel[3] + current_vel[4]) * wheelRadius / 4;
  Vy_current = (-current_vel[1] + current_vel[2] + current_vel[3] - current_vel[4]) * wheelRadius / 4;
  W_current = (-current_vel[1] + current_vel[2] - current_vel[3] + current_vel[4]) *
              (wheelRadius / (4 * ((wheelTrack + wheelBase) / 2)));
}

void HeroChassisController::compute_odometry()
{
  double dt = period_.toSec();
  double delta_x = (Vx_current * cos(th) - Vy_current * sin(th)) * dt;
  double delta_y = (Vx_current * sin(th) + Vy_current * cos(th)) * dt;
  double delta_th = W_current * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;
}

void HeroChassisController::updateOdometry()
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);

  nav_msgs::Odometry odom_data{};
  odom_data.header.stamp = current_time;
  odom_data.header.frame_id = "odom";
  odom_data.child_frame_id = "base_link";

  odom_data.pose.pose.position.x = x;
  odom_data.pose.pose.position.y = y;
  odom_data.pose.pose.position.z = 0.0;
  odom_data.pose.pose.orientation = odom_quat;

  odom_data.twist.twist.linear.x = Vx_current;
  odom_data.twist.twist.linear.y = Vy_current;
  odom_data.twist.twist.angular.z = W_current;

  odom_pub.publish(odom_data);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
