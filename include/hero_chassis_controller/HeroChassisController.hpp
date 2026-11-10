#pragma once

#include <cstdint>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <hero_chassis_controller/PidConfig.h>

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
  void reconfigureCallback(hero_chassis_controller::PidConfig &config,
                           uint32_t level);

  std::vector<hardware_interface::JointHandle> joints_;
  control_toolbox::Pid pids_[4];
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;
  boost::shared_ptr<dynamic_reconfigure::Server<
      hero_chassis_controller::PidConfig>>
      dyn_srv_;

  double wheel_radius_{0.05};
  double lx_{0.2};
  double ly_{0.2};
  double kp_{0.5};
  double max_effort_{5.0};
  double desire_vel_[4]{0.0, 0.0, 0.0, 0.0};

  // odometry state
  double x_{0.0};
  double y_{0.0};
  double th_{0.0};
};

}  // namespace hero_chassis_controller
