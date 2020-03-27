/// \file: turtle_interface.cpp
/// \brief: create a ros node caller turtle_interface
///
/// PARAMETERS:
///    encoder_ticks_per_rev: the number of encoder ticks per revolution of the wheels.
///    max_trans_robot: maximum translational velocity of the robot in m/s
///    max_rot_robot: maximum rotational velocity of the robot, in rad/s
///    max_rot_motor: maximum rotational velocity of the motor, in rad/s
/// PUBLISHES:
///     /wheel_cmd (nuturtlebot::WheelCommands): transform the command twist to command velocity of wheels and publish.
///     /joint_states (sensor_msgs::JointState): transform the sensor data to joint states and publish.
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs::Twist): get the command velocity of the robot
///     /sensor_data (nuturtlebot::SensorData): get the sensor data of the robot

#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/SetPose.h"
#include "rigid2d/diff_drive.hpp"

class Interface{
public:
    Interface();
private:
    ///functions
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& ts);
    void sensorDataCallback(const nuturtlebot::SensorData::ConstPtr& sd);
    geometry_msgs::Twist limitInput(const geometry_msgs::Twist::ConstPtr& ts);
    nuturtlebot::WheelCommands limitOutput(const rigid2d::WheelVelocities wheel_vel);

    ///node handles
    ros::NodeHandle n;
    ros::NodeHandle params;

    ///robot parameters
    double max_trans_robot,max_rot_robot,max_rot_motor;
    int encoder_ticks_per_rev;
    int left_encoder, right_encoder, left_offset, right_offset;
    ros::Time time_stamp_encoder=ros::Time::now();

    ///topics
    ros::Subscriber vel_sub;
    ros::Subscriber sensor_sub;
    ros::Publisher wheel_pub;
    ros::Publisher js_pub;

    ///robot model
    rigid2d::DiffDrive intRobot;//only for using the function twisttowheels
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_interface");

    Interface interface;

    ros::Rate my_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        my_rate.sleep();
    }

    return 0;
};

/// \brief constructor function
/// initialize the parameters
Interface::Interface() {

    ///wait for message
    geometry_msgs::TwistConstPtr ss=ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_vel");

    ///subscriber and publisher topics
    vel_sub = n.subscribe("/cmd_vel", 200, &Interface::cmdVelCallback,this);
    sensor_sub = n.subscribe("/sensor_data", 200, &Interface::sensorDataCallback,this);
    wheel_pub = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 200,true);
    js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 200,true);

    ///load robot params
    params.getParam("encoder_ticks_per_rev",encoder_ticks_per_rev);
    params.getParam("max_trans_robot",max_trans_robot);
    params.getParam("max_rot_robot",max_rot_robot);
    params.getParam("max_rot_motor",max_rot_motor);

    ///get the sensor data offset for initializaton.
    nuturtlebot::SensorDataConstPtr encoder_offset=ros::topic::waitForMessage<nuturtlebot::SensorData>("/sensor_data");
    left_offset=encoder_offset->left_encoder;
    right_offset=encoder_offset->right_encoder;
};

/// \brief callback function
/// \param  ts - geometry_msgs::Twist
void Interface::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& ts)
{
    /// 1. get twist from input, transform it to joint states
    geometry_msgs::Twist limit_ts=limitInput(ts);
    rigid2d::Twist2D twist;
    twist.w=limit_ts.angular.z;
    twist.vx=limit_ts.linear.x;
    twist.vy=limit_ts.linear.y;
    rigid2d::WheelVelocities wheel_vel=intRobot.twistToWheels(twist);
    //ROS_INFO("cmd_vel %f, %f, %f",twist.w,twist.vx,twist.vy);
    /// 2.publish joint states.
    nuturtlebot::WheelCommands limit_wheel_cmd=limitOutput(wheel_vel);
    wheel_pub.publish(limit_wheel_cmd);
};

/// \brief callback function
/// \param  sd - nuturtlebot::SensorData
void Interface::sensorDataCallback(const nuturtlebot::SensorData::ConstPtr& sd){

    /// 1.update velocity
    double time_interval=ros::Time::now().toSec()-time_stamp_encoder.toSec();
    double vel_left=(sd->left_encoder-left_encoder)*1.0/(encoder_ticks_per_rev*time_interval);//in radians
    double vel_right=(sd->right_encoder-right_encoder)*1.0/(encoder_ticks_per_rev*time_interval);

    /// 2.update encoder and time stamp
    left_encoder=(sd->left_encoder-left_offset);
    right_encoder=(sd->right_encoder-right_offset);
    time_stamp_encoder = ros::Time::now();

    /// 3.publish joint states.
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name.push_back("left_wheel_axle");
    js.name.push_back("right_wheel_axle");
    js.position.push_back(left_encoder*1.0/encoder_ticks_per_rev*2*rigid2d::PI);
    js.position.push_back(right_encoder*1.0/encoder_ticks_per_rev*2*rigid2d::PI);
    js.velocity.push_back(vel_left);
    js.velocity.push_back(vel_right);
    js_pub.publish(js);
}

/// \brief limit input cmd_vel
/// \param  ts - geometry_msgs::Twist
geometry_msgs::Twist Interface::limitInput(const geometry_msgs::Twist::ConstPtr& ts){

    geometry_msgs::Twist new_ts;
    /// 1.translation limit
    if (abs(ts->linear.x)>max_trans_robot){
        new_ts.linear.x=ts->linear.x/abs(ts->linear.x)*max_trans_robot;
    }
    else{
        new_ts.linear.x=ts->linear.x;
    }
    new_ts.linear.y=0;
    new_ts.linear.z=0;

    /// 2.rotation limit
    new_ts.angular.x=0;
    new_ts.angular.y=0;
    if (abs(ts->angular.z)>max_rot_robot){
        new_ts.angular.z=ts->angular.z/abs(ts->angular.z)*max_trans_robot;     std::cout<<"limitInput"<<std::endl;
    }
    else{
        new_ts.angular.z=ts->angular.z;
    }
    return new_ts;
}

/// \brief limit output wheel velocities
/// \param  wheel_vel - rigid2d::WheelVelocities, wheel velocities
nuturtlebot::WheelCommands Interface::limitOutput(const rigid2d::WheelVelocities wheel_vel){
    nuturtlebot::WheelCommands limit_wheel_vel;

    /// 1.left velocity limit
    if (abs(wheel_vel.left)>max_rot_motor){limit_wheel_vel.left_velocity=wheel_vel.left/abs(wheel_vel.left)*265; std::cout<<"limitOutput"<<std::endl;}
    else{limit_wheel_vel.left_velocity=wheel_vel.left*265/max_rot_motor;}

    /// 2.right velocity limit
    if (abs(wheel_vel.right)>max_rot_motor){limit_wheel_vel.right_velocity=wheel_vel.right/abs(wheel_vel.right)*265; std::cout<<"limitOutput"<<std::endl;}
    else{limit_wheel_vel.right_velocity=wheel_vel.right*265/max_rot_motor;}

    return limit_wheel_vel;
}

