//
// Created by xin on 23-1-4.
//

//#include "../include/hero_chassis_controller.h"

#include "hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <memory>

namespace hero_chassis_controller
{
    HeroChassisController::~HeroChassisController()
    {
        sub_command.shutdown();
    }

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        controller_nh.getParam("Wheel_Track", Wheel_Track);
        controller_nh.getParam("Wheel_Base", Wheel_Base);

        //get joint handle from hardware interface
        front_left_joint_ = effort_joint_interface->getHandle("front_left_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("front_right_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("back_left_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("back_right_wheel_joint");

        //load PID Controller using gains set on parameter server
        pid1_controller_.init(ros::NodeHandle(controller_nh, "pid1"));
        pid2_controller_.init(ros::NodeHandle(controller_nh, "pid2"));
        pid3_controller_.init(ros::NodeHandle(controller_nh, "pid3"));
        pid4_controller_.init(ros::NodeHandle(controller_nh, "pid4"));

        //Start realtime state publisher
        controller_state_publisher_ = std::make_unique < realtime_tools::RealtimePublisher < control_msgs::JointControllerState >>(controller_nh, "state", 1);
        //start command subscriber
        sub_command = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_chassis_state,this);//118

        odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 50);

        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period)
    {
        vel_act[1] = front_right_joint_.getVelocity();
        vel_act[2] = front_left_joint_.getVelocity();
        vel_act[3] = back_left_joint_.getVelocity();
        vel_act[4] = back_right_joint_.getVelocity();

        //calculate the speed of chassis
        compute_chassis_velocity();//133

        //calculate expected speed of wheels
        compute_mecvel();//105
//            compute_vel_rte();//113
        //the error of wheels
        double error1 = vel_cmd[1] - vel_act[1];
        double error2 = vel_cmd[2] - vel_act[2];
        double error3 = vel_cmd[3] - vel_act[3];
        double error4 = vel_cmd[4] - vel_act[4];

        //set command for wheels
        front_right_joint_.setCommand(pid1_controller_.computeCommand(error1, period));
        front_left_joint_.setCommand(pid2_controller_.computeCommand(error2, period));
        back_left_joint_.setCommand(pid3_controller_.computeCommand(error3, period));
        back_right_joint_.setCommand(pid4_controller_.computeCommand(error4, period));

        if (loop_count_ % 10 == 0)
        {
            if (controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = now;
                controller_state_publisher_->msg_.set_point = vel_cmd[1];
                controller_state_publisher_->msg_.process_value = vel_act[1];
                controller_state_publisher_->msg_.error = error1;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = pid1_controller_.computeCommand(error1, period);

                double dummy;
                bool antiwindup;
                pid1_controller_.getGains(controller_state_publisher_->msg_.p,
                                          controller_state_publisher_->msg_.i,
                                          controller_state_publisher_->msg_.d,
                                          controller_state_publisher_->msg_.i_clamp,
                                          dummy,
                                          antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;
    }

    void HeroChassisController::get_chassis_state(const geometry_msgs::TwistConstPtr &msg)
    {
        Vxe = msg->linear.x;
        Vye = msg->linear.y;
        yawe = msg->angular.z;

    }

    void HeroChassisController::compute_mecvel()
    {
        vel_cmd[1] = (Vxe + Vye + yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
        vel_cmd[2] = (Vxe - Vye - yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
        vel_cmd[3] = (Vxe + Vye - yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
        vel_cmd[4] = (Vxe - Vye + yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
    }

//        void HeroChassisController::compute_vel_rte()
//        {
//            int i;
//            for (i = 1; i <= 4; i++)
//            {
//                if (vel_cmd[i] > vel_act[i])
//                {
//                    vel_rte[i] += dt * Angle_Acceleration;
//                    if (vel_rte[i] >= vel_cmd[i])
//                        vel_rte[i] = vel_cmd[i];
//                }
//                if (vel_cmd[i] < vel_act[i])
//                {
//                    vel_rte[i] -= dt * Angle_Acceleration;
//                    if (vel_rte[i] <= vel_cmd[i])
//                        vel_rte[i] = vel_cmd[i];
//                }
//            }
}

    void HeroChassisController::compute_chassis_velocity()
    {
        Vxa = (vel_act[1] + vel_act[2] + vel_act[3] + vel_act[4]) * RADIUS / 4;
        Vya = (vel_act[1] - vel_act[2] + vel_act[3] - vel_act[4]) * RADIUS / 4;
        yawa = (vel_act[1] - vel_act[2] - vel_act[3] + vel_act[4]) * RADIUS / 2 / (Wheel_Track + Wheel_Base);
    }


}// hero_chassis_controller

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)