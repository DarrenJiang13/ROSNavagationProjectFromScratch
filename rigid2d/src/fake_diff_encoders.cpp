/// \file: fake_diff_encoders.cpp
/// \brief: create a ros node called fake_diff_encoders
///
/// PARAMETERS:
///     empty
/// PUBLISHES:
///     /joint_states (sensor_msgs::JointState): publish the wheel encoder for the robot in rviz
/// SUBSCRIBES:
///     /turtle1/cmd_vel (geometry_msgs::Twist): get the command velocity published to the turtlesim.
///                                             this velocity would be changed to the wheel encoders.

#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "rigid2d/diff_drive.hpp"

using namespace std;
using namespace rigid2d;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& ts);

ros::Publisher js_pub;
ros::Subscriber vel_sub;
DiffDrive diffRobot;
static string left_wheel_joint,right_wheel_joint;
Encoder current_encoder;
double freq_of_cmd_vel=120.0;

int main(int argc, char **argv) {

    ros::init(argc, argv, "fake_diff_encoders");

    ros::NodeHandle n;
    ros::NodeHandle params;

    double wheel_base, wheel_radius;
    params.getParam("odometer/left_wheel_joint",left_wheel_joint);
    params.getParam("odometer/right_wheel_joint",right_wheel_joint);
    params.getParam("/wheel_base", wheel_base);
    params.getParam("/wheel_radius", wheel_radius);

    vel_sub=n.subscribe("/turtle1/cmd_vel", 200, cmdVelCallback);
    js_pub=n.advertise<sensor_msgs::JointState>("joint_states", 200);

    ros::spin();
    return 0;
};

/// \brief callback function
/// \param  ts - geometry_msgs::Twist
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& ts)
{
    sensor_msgs::JointState js;

    /// 1. get twist from input, move the robot
    /// why divided by 60?
    ///     60 is command frequency in tsim/src/turtleway.cpp
    ///     we only call this function every 1/60 second, so we only need 1/60 of the twist.
    Twist2D twist;
    twist.w=ts->angular.z/freq_of_cmd_vel;
    twist.vx=ts->linear.x/freq_of_cmd_vel;
    twist.vy=ts->linear.y/freq_of_cmd_vel;
    diffRobot.feedforward(twist);

    /// 2.publish joint states.
    current_encoder=diffRobot.wheelEncoderState();
    js.header.stamp = ros::Time::now();
    js.name.push_back("left_wheel_axle");
    js.name.push_back("right_wheel_axle");
    js.position.push_back(current_encoder.left);
    js.position.push_back(current_encoder.right);
    js_pub.publish(js);
};

