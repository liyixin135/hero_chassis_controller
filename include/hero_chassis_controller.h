//
// Created by xin on 23-1-4.
//

#ifndef HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#define RADIUS 0.07625

namespace hero_chassis_controller{

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        ros::Subscriber cmd_sub;
        ros::Publisher odom_pub;

        tf::TransformBroadcaster odom_broadcaster;
        tf::TransformListener listener;
        tf::StampedTransform transform;

        geometry_msgs::TransformStamped odom_trans;
        geometry_msgs::Quaternion odom_quat;

        nav_msgs::Odometry odom;

        control_toolbox::Pid pid1_controller, pid2_controller, pid3_controller, pid4_controller;

        hardware_interface::JointHandle
                left_front_joint, right_front_joint, left_back_joint,
                right_back_joint;
    private:
        int loop_count;
        //期望角速度
        double vel_cmd[5]{0.0, 0.0, 0.0, 0.0, 0.0};
        //真实角速度
        double vel_act[5];
        //期望速度
        double Vxe{0.0}, Vye{0.0}, yawe{0.0};
        //真实速度
        double Vxa, Vya, yawa;
        double dt;
        double x{0.0}, y{0.0}, th{0.0};
        double Wheel_Track;
        double Wheel_Base;

        ros::Time last_time;
        ros::Time now;

        std::unique_ptr<
            realtime_tools::RealtimePublisher<
                control_msgs::JointControllerState> > controller_state_publisher_;
        void get_chassis_state(const geometry_msgs::TwistConstPtr &msg);
        void cpt_mecvel();
        void cpt_chassis_velocity();
        void Transform_broadcast();
        void Odometry_publish();
    };

} // hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_
