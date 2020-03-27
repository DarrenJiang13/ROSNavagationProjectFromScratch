/// \file: rotation.cpp
/// \brief: create a ros node caller rotation, test the rotation of real robot
///
/// PARAMETERS:
///    max_trans_robot: max translation velocity of the robot
/// PUBLISHES:
///     /vel_pub (geometry_msgs::Twist): publish the command velocity of the robot
/// SUBSCRIBES:
///     /start (nuturtle_robot::Start): reset the odometer and start the motion of the robot

#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "nuturtle_robot/Start.h"
#include "rigid2d/SetPose.h"
#include "rigid2d/diff_drive.hpp"

///callback functions
bool startSrvCallback(nuturtle_robot::Start::Request &req,nuturtle_robot::Start::Response &res);
void timerCallback(const ros::TimerEvent& event);

///variables
int rotate_direction=2;
int rotation_count;//trans_count:current count of velocity command sent in one period.
int count_bound;//count_bound:maximum of trans_count in one period
int rotation_cycles;//trans_steps:number of current period. e.g.: trans_steps = 10 means 10th period
int steps_bound;//steps_bound:how many trans_steps the robot need to move
double frac_vel,max_rot_robot;//[0,1], fraction of the max trans speed of robot

///topics and services
ros::ServiceServer start_srv;
ros::Publisher vel_pub;


int main(int argc, char **argv) {
    ros::init(argc, argv, "rotation");

    ///load robot param
    ros::NodeHandle params;
    params.getParam("max_rot_robot",max_rot_robot);
    frac_vel=0.5*rigid2d::PI/max_rot_robot;
    //frac_vel=1;

    ///define topics and services
    ros::NodeHandle n;
    start_srv = n.advertiseService("/start", startSrvCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 120);

    ///define motion parameters
    ros::service::waitForService("/start");
    ros::NodeHandle nh;
    rotation_count=0;
    rotation_cycles=0;
    steps_bound=20;
    count_bound=round(2*rigid2d::PI/(frac_vel*max_rot_robot)*120);//frequency=120

    ///ros::Timer for call velocity in a constant frequency
    ros::Timer timer = n.createTimer(ros::Duration(1.0/120), timerCallback);

    ros::spin();
    return 0;
};

/// \brief ros::Timer callback function
/// send velocity command in constant frequency
bool startSrvCallback(nuturtle_robot::Start::Request &req,nuturtle_robot::Start::Response &res)
{

    /// reset the odometers (both fake and actual)
    rigid2d::SetPose setPose;
    setPose.request.w=0;
    setPose.request.vx=0;
    setPose.request.vy=0;
    ros::service::call("/set_pose",setPose);
    ros::service::call("/fake/set_pose",setPose);

    ///set the direction of translation
    rotate_direction=req.positive;
    if (req.positive){
        ROS_INFO("Rotate anti-clockwise.");
    }
    else{
        ROS_INFO("Rotate clockwise.");
    }

    ///reset the move counts
    rotation_cycles=0;
    rotation_count=0;
    return true;
}

/// \brief ros::Timer callback function
/// send velocity command in constant frequency
void timerCallback(const ros::TimerEvent& event){
    geometry_msgs::Twist ts;

    if (rotate_direction==1 || rotate_direction==0){// if the translation direction is set correctly
        if (rotation_cycles<steps_bound){//if step to move is less than max step
            rotation_count+=1;

            if (rotation_count<=count_bound){//if velocity count in each step is less than max
                ts.angular.z=frac_vel*max_rot_robot*(rotate_direction-0.5)*2;
                vel_pub.publish(ts);
            }
            else{
                ts.angular.z=0;
                vel_pub.publish(ts);
                if (rotation_count>=count_bound/20*21){//stop for 1/20 time of one full circle
                    rotation_count=0;//reset trans_count
                    rotation_cycles+=1;
                }
            }
        }
        else{// if the translation direction is set incorrectly, no movement
            ts.angular.z=0;
            vel_pub.publish(ts);
        }
    }
    else{
        vel_pub.publish(ts);
        ROS_INFO("waiting for start command");
    }
}
