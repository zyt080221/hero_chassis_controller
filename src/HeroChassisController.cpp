#include "hero_chassis_controller/HeroChassisController.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface *effort_joint_interface,
    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  std::vector<std::string> joint_names;

  for (int i = 0; i < 4; ++i) {
    pids_[i].initPid(10.0, 0.1, 0.0, 10.0, -10.0);
  }
  dyn_srv_.reset(
      new dynamic_reconfigure::Server<hero_chassis_controller::PidConfig>(
          controller_nh));
  dynamic_reconfigure::Server<
      hero_chassis_controller::PidConfig>::CallbackType callback;
  callback =
      boost::bind(&HeroChassisController::reconfigureCallback, this, _1, _2);
  dyn_srv_->setCallback(callback);

  if (!controller_nh.getParam("joints", joint_names) ||
      joint_names.size() != 4) {
    ROS_ERROR("hero_chassis_controller requires exactly 4 'joints'.");
    return false;
  }

  joints_.clear();
  for (const std::string &name : joint_names) {
    try {
      joints_.push_back(effort_joint_interface->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM("Could not get joint handle '" << name
                                                     << "': " << ex.what());
      return false;
    }
  }

  controller_nh.param("wheel_radius", wheel_radius_, wheel_radius_);
  controller_nh.param("lx", lx_, lx_);
  controller_nh.param("ly", ly_, ly_);
  controller_nh.param("kp", kp_, kp_);
  controller_nh.param("max_effort", max_effort_, max_effort_);

  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 50);
  cmd_vel_sub_ = root_nh.subscribe(
      "/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);
  return true;
}

void HeroChassisController::update(const ros::Time &time,
                                   const ros::Duration &period) {
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double error = desire_vel_[i] - joints_[i].getVelocity();
    double effort = pids_[i].computeCommand(error, period);
    effort = std::max(-max_effort_, std::min(max_effort_, effort));
    joints_[i].setCommand(effort);
  }

  // 1. Actual wheel angular velocities
  const double v0 = joints_[0].getVelocity();
  const double v1 = joints_[1].getVelocity();
  const double v2 = joints_[2].getVelocity();
  const double v3 = joints_[3].getVelocity();

  // 2. Mecanum forward kinematics
  const double vx = (v0 + v1 + v2 + v3) * wheel_radius_ / 4.0;
  const double vy = (-v0 + v1 + v2 - v3) * wheel_radius_ / 4.0;
  const double vth =
      (-v0 + v1 - v2 + v3) * wheel_radius_ / (4.0 * (lx_ + ly_));

  // 3. Integrate velocity to pose
  const double dt = period.toSec();
  x_ += (vx * std::cos(th_) - vy * std::sin(th_)) * dt;
  y_ += (vx * std::sin(th_) + vy * std::cos(th_)) * dt;
  th_ += vth * dt;

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  const geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(th_);
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster_.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;
  odom_pub_.publish(odom);
}

void HeroChassisController::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  const double vx = msg->linear.x;
  const double vy = msg->linear.y;
  const double wz = msg->angular.z;
  const double k = lx_ + ly_;
  const double r = wheel_radius_;

  desire_vel_[0] = (vx - vy - k * wz) / r;
  desire_vel_[1] = (vx + vy + k * wz) / r;
  desire_vel_[2] = (vx + vy - k * wz) / r;
  desire_vel_[3] = (vx - vy + k * wz) / r;
  ROS_INFO_STREAM_THROTTLE(
      1.0, "Received cmd_vel! Target Wheel Speed: " << desire_vel_[0]);
}

void HeroChassisController::reconfigureCallback(
    hero_chassis_controller::PidConfig &config, uint32_t level) {
  (void)level;
  ROS_INFO("Dynamic Reconfigure: P=%.2f, I=%.2f, D=%.2f", config.p, config.i,
           config.d);
  for (int i = 0; i < 4; ++i) {
    pids_[i].setGains(config.p, config.i, config.d, 10.0, -10.0);
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
