//
// Created by xin on 23-1-4.
//

#include "hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <memory>

namespace hero_chassis_controller
{
    HeroChassisController::~HeroChassisController()
    {
        cmd_sub.shutdown();
        odom_pub.shutdown();
    }

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        controller_nh.getParam("Wheel_Track", Wheel_Track);
        controller_nh.getParam("Wheel_Base", Wheel_Base);

        //从hardware interface获取joint handle
        left_front_joint = effort_joint_interface->getHandle("left_front_wheel_joint");
        right_front_joint = effort_joint_interface->getHandle("right_front_wheel_joint");
        left_back_joint = effort_joint_interface->getHandle("left_back_wheel_joint");
        right_back_joint = effort_joint_interface->getHandle("right_back_wheel_joint");

        //load PID Controller using gains set on parameter server
        pid1_controller.init(ros::NodeHandle(controller_nh, "pid1"));
        pid2_controller.init(ros::NodeHandle(controller_nh, "pid2"));
        pid3_controller.init(ros::NodeHandle(controller_nh, "pid3"));
        pid4_controller.init(ros::NodeHandle(controller_nh, "pid4"));

        //初始化last_time
        last_time = ros::Time::now();

        //Start realtime state publisher
        controller_state_publisher_ = std::make_unique < realtime_tools::RealtimePublisher < control_msgs::JointControllerState >>(controller_nh, "state", 1);
        //start command subscriber
        cmd_sub = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_chassis_state,this);

        odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 50);

        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period)
    {
        now = time;
        //读取真实角速度
        vel_act[1] = right_front_joint.getVelocity();
        vel_act[2] = left_front_joint.getVelocity();
        vel_act[3] = left_back_joint.getVelocity();
        vel_act[4] = right_back_joint.getVelocity();
        //利用真实角速度，计算真实线速度，正运动学
        cpt_chassis_velocity();
        //broadcast Transform from "base_link" to "odom"
        Transform_broadcast();
        //publish the odometry message over ROS
        Odometry_publish();
        //利用期望正交分解速度计算期望角速度，逆运动学
        cpt_mecvel();
        double error1 = vel_cmd[1] - vel_act[1];
        double error2 = vel_cmd[2] - vel_act[2];
        double error3 = vel_cmd[3] - vel_act[3];
        double error4 = vel_cmd[4] - vel_act[4];

        //set command for wheels
        right_front_joint.setCommand(pid1_controller.computeCommand(error1, period));
        left_front_joint.setCommand(pid2_controller.computeCommand(error2, period));
        left_back_joint.setCommand(pid3_controller.computeCommand(error3, period));
        right_back_joint.setCommand(pid4_controller.computeCommand(error4, period));

        if (loop_count % 10 == 0)
        {
            if (controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = now;
                controller_state_publisher_->msg_.set_point = vel_cmd[1];
                controller_state_publisher_->msg_.process_value = vel_act[1];
                controller_state_publisher_->msg_.error = error1;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = pid1_controller.computeCommand(error1, period);

                double dummy;
                bool antiwindup;
                pid1_controller.getGains(controller_state_publisher_->msg_.p,
                                          controller_state_publisher_->msg_.i,
                                          controller_state_publisher_->msg_.d,
                                          controller_state_publisher_->msg_.i_clamp,
                                          dummy,
                                          antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count++;
        last_time = now;
    }

    void HeroChassisController::get_chassis_state(const geometry_msgs::TwistConstPtr &msg)
    {
        Vxe = msg->linear.x;
        Vye = msg->linear.y;
        yawe = msg->angular.z;

    }

    void HeroChassisController::cpt_chassis_velocity()
    {
        Vxa = (vel_act[1] + vel_act[2] + vel_act[3] + vel_act[4]) * RADIUS / 4;
        Vya = (vel_act[1] - vel_act[2] + vel_act[3] - vel_act[4]) * RADIUS / 4;
        yawa = (vel_act[1] - vel_act[2] - vel_act[3] + vel_act[4]) * RADIUS / 2 / (Wheel_Track + Wheel_Base);
    }

    void HeroChassisController::cpt_mecvel()
    {
        vel_cmd[1] = (Vxe + Vye + yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
        vel_cmd[2] = (Vxe - Vye - yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
        vel_cmd[3] = (Vxe + Vye - yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
        vel_cmd[4] = (Vxe - Vye + yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
    }

    //里程计
    void HeroChassisController::Transform_broadcast()
    {
        dt = (now - last_time).toSec();
        double delta_x = (Vxa * cos(th) - Vya * sin(th)) * dt;
        double delta_y = (Vxa * sin(th) + Vya * cos(th)) * dt;
        double delta_th = yawa * dt;
        x += delta_x;
        y += delta_y;
        th += delta_th;

        odom_quat = tf::createQuaternionMsgFromYaw(th);

        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
    }

    void HeroChassisController::Odometry_publish()
    {
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = Vxa;
        odom.twist.twist.linear.y = Vya;
        odom.twist.twist.angular.z = yawa;

        //publish the message
        odom_pub.publish(odom);
    }
}// hero_chassis_controller

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)