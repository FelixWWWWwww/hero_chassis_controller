#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "hardware_interface/joint_command_interface.h"
#include "controller_interface/controller.h"
#include "control_toolbox/pid.h"

namespace hero_chassis_controller {
    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh,
                  ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &scan_msg);

    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        double wheel_track=0.4;
        double wheel_base=0.4;
        double r=0.07625;
        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;
        control_toolbox::Pid pid_controller_;
        geometry_msgs::Twist msg;
    };

} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_H