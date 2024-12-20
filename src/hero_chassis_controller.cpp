#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get params
  if (!controller_nh.getParam("power/effort_coeff", effort_coeff_) ||
      !controller_nh.getParam("power/vel_coeff", velocity_coeff_) ||
      !controller_nh.getParam("power/power_offset", power_offset_))
  {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  controller_nh.getParam("wheel_radius", wheelRadius);
  controller_nh.getParam("wheel_track", wheelTrack);
  controller_nh.getParam("wheel_base", wheelBase);
  controller_nh.param("timeout", timeout_, 0.1);
  controller_nh.param("use_global_vel", use_global_velocity_, false);
  controller_nh.param("acceleration", acceleration_, 65.0);
  controller_nh.param("power_limit", power_limit, 450.0);

  // effort_joint init
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
  joint_handles_.push_back(front_left_joint_);
  joint_handles_.push_back(front_right_joint_);
  joint_handles_.push_back(back_left_joint_);
  joint_handles_.push_back(back_right_joint_);

  // load PID Controller using gains set on parameter server
  front_left_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "front_left_joint_pid"));
  front_right_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "front_right_joint_pid"));
  back_left_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "back_left_joint_pid"));
  back_right_joint_pid_controller_.init(ros::NodeHandle(controller_nh, "back_right_joint_pid"));
  joint_pid_controller_.push_back(front_left_joint_pid_controller_);
  joint_pid_controller_.push_back(front_right_joint_pid_controller_);
  joint_pid_controller_.push_back(back_left_joint_pid_controller_);
  joint_pid_controller_.push_back(back_right_joint_pid_controller_);

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

  // avoid loss of control due to lack of command
  if ((current_time - last_cmd_vel_stamp_).toSec() > timeout_)
  {
    Vx_target = 0.;
    Vy_target = 0.;
    W_target = 0.;
  }

  // get each current_wheel_vel
  get_wheel_vel();

  // odometry
  calc_chassis_vel();
  compute_odometry();
  updateOdometry();

  if (use_global_velocity_)
  {
    try
    {
      global_vel.header.frame_id = "odom";
      //    global_vel.header.stamp = current_time;
      global_vel.header.stamp = ros::Time(0);
      global_vel.vector.x = Vx_target;
      global_vel.vector.y = Vy_target;
      global_vel.vector.z = 0.0;

      tf_listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(3.0));
      tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), transform);
      tf_listener_.transformVector("base_link", global_vel, base_vel);

      Vx_target = base_vel.vector.x;
      Vy_target = base_vel.vector.y;
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("global_vel->base_vel error: %s", ex.what());
    }
  }

  chassis_control();

  controller_state_publish();

  last_time = current_time;
}

void HeroChassisController::set_chassis_state(const geometry_msgs::Twist::ConstPtr& msg)
{
  last_cmd_vel_stamp_ = ros::Time::now();
  Vx_target = msg->linear.x;
  Vy_target = msg->linear.y;
  W_target = msg->angular.z;
}

void HeroChassisController::get_wheel_vel()
{
  current_vel[1] = front_left_joint_.getVelocity();
  current_vel[2] = front_right_joint_.getVelocity();
  current_vel[3] = back_left_joint_.getVelocity();
  current_vel[4] = back_right_joint_.getVelocity();
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
  Vx_last = Vx_current;
  Vy_last = Vy_current;
  W_last = W_target;

  Vx_current = (current_vel[1] + current_vel[2] + current_vel[3] + current_vel[4]) * wheelRadius / 4;
  Vy_current = (-current_vel[1] + current_vel[2] + current_vel[3] - current_vel[4]) * wheelRadius / 4;
  W_current = (-current_vel[1] + current_vel[2] - current_vel[3] + current_vel[4]) *
              (wheelRadius / (4 * ((wheelTrack + wheelBase) / 2)));
}

void HeroChassisController::simple_acceleration_planner()
{
  for (int i = 1; i <= 4; ++i)
  {
    if (target_vel[i] > current_vel[i])
    {
      ref_vel[i] += acceleration_ * dt;
      if (ref_vel[i] >= target_vel[i])
      {
        ref_vel[i] = target_vel[i];
      }
    }
    if (target_vel[i] < current_vel[i])
    {
      ref_vel[i] -= acceleration_ * dt;
      if (ref_vel[i] <= target_vel[i])
      {
        ref_vel[i] = target_vel[i];
      }
    }
  }
}

void HeroChassisController::chassis_control()
{
  // Inverse Kinematics and PID control
  calc_wheel_vel();
  simple_acceleration_planner();

  for (int i = 1; i <= 4; ++i)
  {
    error[i] = ref_vel[i] - current_vel[i];

    // Set the PID error and compute the PID command with nonuniform time
    // step size. The derivative error is computed from the change in the error
    // and the timestep dt.
    commanded_effort[i] = joint_pid_controller_[i - 1].computeCommand(error[i], period_);
  }

  powerLimit();

  front_left_joint_.setCommand(commanded_effort[1]);
  front_right_joint_.setCommand(commanded_effort[2]);
  back_left_joint_.setCommand(commanded_effort[3]);
  back_right_joint_.setCommand(commanded_effort[4]);
}

void HeroChassisController::compute_odometry()
{
  dt = period_.toSec();
  double delta_x = (Vx_current * cos(th) - Vy_current * sin(th)) * dt;
  double delta_y = (Vx_current * sin(th) + Vy_current * cos(th)) * dt;
  double delta_th = W_current * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;
}

void HeroChassisController::updateOdometry()
{
  try
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
  catch (tf::TransformException& ex)
  {
    ROS_WARN("updateOdometry error: %s", ex.what());
  }
}

void HeroChassisController::controller_state_publish()
{
  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = current_time;
      controller_state_publisher_->msg_.set_point = target_vel[1];
      controller_state_publisher_->msg_.process_value = current_vel[1];
      controller_state_publisher_->msg_.error = error[1];
      controller_state_publisher_->msg_.time_step = period_.toSec();
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
    loop_count_ = 0;
  }
  loop_count_++;
}

void HeroChassisController::powerLimit()
{
  // Three coefficients of a quadratic equation in one variable
  double t_2_sum = 0., P_out_sum = 0., w_2_sum = 0.;
  for (const auto& joint : joint_handles_)
  {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    t_2_sum += cmd_effort * cmd_effort;
    P_out_sum += std::abs(cmd_effort * real_vel);
    w_2_sum += real_vel * real_vel;
  }
  t_2_sum *= effort_coeff_;
  w_2_sum = w_2_sum * velocity_coeff_ - power_limit - power_offset_;
  // Solving zoom_coeff equations
  double zoom_coeff = ((P_out_sum * P_out_sum) - 4 * t_2_sum * w_2_sum) > 0 ?
                          ((-P_out_sum + sqrt((P_out_sum * P_out_sum) - 4 * t_2_sum * w_2_sum)) / (2 * t_2_sum)) :
                          0.;
  for (auto joint : joint_handles_)
    joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
