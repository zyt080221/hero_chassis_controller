#include "hero_chassis_controller/HeroChassisController.hpp"

#include <algorithm>
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

  cmd_vel_sub_ = root_nh.subscribe(
      "/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);
  return true;
}

void HeroChassisController::update(const ros::Time & /*time*/,
                                   const ros::Duration & period) {
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double error = desire_vel_[i] - joints_[i].getVelocity();
    double effort =pids_[i].computeCommand(error, period);
    effort = std::max(-max_effort_, std::min(max_effort_, effort));
    joints_[i].setCommand(effort);
  }
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
  ROS_INFO_STREAM_THROTTLE(1.0, "Received cmd_vel! Target Wheel Speed: " << desire_vel_[0]);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
