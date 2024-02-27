#include "hero_chassis_controller/hero_chassis_controller.h"

namespace hero_chassis_controller {
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh,
                                     ros::NodeHandle &controller_nh) {
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
        ROS_INFO("00000");

        // 从参数服务器获取PID控制器的参数
        double p_gain;
        controller_nh.param<double>("p_gain", p_gain, 0.1);  // 设置默认值为0.1

        // 初始化PID控制器
//        pid_controller_.init();
//        pid_controller_.setGains(p_gain, 0.0, 0.0);
        // 初始化PID控制器
//        pid_controller_.initPid(p_gain, 0.0, 0.0);
        // 初始化pid控制器
        pid_controller_.init(controller_nh);
        pid_controller_.setGains(0.1, 0.1, 0.1, 0.0, 0.0);

        // 订阅/cmd_vel消息
        sub = controller_nh.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);

        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
        ros::NodeHandle nh;


        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // TODO 校验算法是否正确
        double w1 = (msg.linear.x - msg.linear.y - (wheel_track + wheel_base)) / r;
        double w2 = (msg.linear.x + msg.linear.y + (wheel_track + wheel_base)) / r;
        double w3 = (msg.linear.x + msg.linear.y - (wheel_track + wheel_base)) / r;
        double w4 = (msg.linear.x - msg.linear.y + (wheel_track + wheel_base)) / r;

        double real_w1 = front_left_joint_.getVelocity();
        double real_w2 = front_right_joint_.getVelocity();
        double real_w3 = back_left_joint_.getVelocity();
        double real_w4 = back_right_joint_.getVelocity();

        double error1 = w1 - real_w1;
        double error2 = w2 - real_w2;
        double error3 = w3 - real_w3;
        double error4 = w4 - real_w4;

        double command_effort1 = pid_controller_.computeCommand(error1, period);
        double command_effort2 = pid_controller_.computeCommand(error2, period);
        double command_effort3 = pid_controller_.computeCommand(error3, period);
        double command_effort4 = pid_controller_.computeCommand(error4, period);

        front_left_joint_.setCommand(command_effort1);
        front_right_joint_.setCommand(command_effort2);
        back_left_joint_.setCommand(command_effort3);
        back_right_joint_.setCommand(command_effort4);

        pub.publish(msg); // 发布消息
        //ROS_INFO("44444444");
    }

    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &scan_msg) {
        // TODO 确保能触发这里的代码
        //std::cout << "tick" << std::endl;
//        printf("tick\n");
        // TODO 将9宫格按键转为4个轮值的向前向后值（浮点）
        msg.linear.x = scan_msg->linear.x; // 设置线速度
        msg.linear.y = scan_msg->linear.y;
        msg.angular.z = scan_msg->angular.z; // 设置角速度
        printf("x%lf\n",msg.linear.x);
        printf("y%lf\n",msg.linear.y);
        printf("z%lf\n",msg.angular.z);
    }

} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)