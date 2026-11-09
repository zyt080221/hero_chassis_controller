#pragma once

#include <vector>

#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  std::vector<hardware_interface::JointHandle> joints_;
 
  control_toolbox::Pid pids_[4];

  ros::Subscriber cmd_vel_sub_;

  double wheel_radius_{0.05};
  double lx_{0.2};
  double ly_{0.2};
  double kp_{0.5};
  double max_effort_{5.0};
  double desire_vel_[4]{0.0, 0.0, 0.0, 0.0};
};

}  // namespace hero_chassis_controller
