/// \file: translation.cpp
/// \brief: create a ros node caller translation, test the translation of real robot
///
/// PARAMETERS:
///    max_trans_robot: max translation velocity of the robot
/// PUBLISHES:
///     /vel_pub (geometry_msgs::Twist): publish the command velocity of the robot
/// SUBSCRIBES:
///     /start (nuturtle_robot::Start): reset the odometer and start the motion of the robot

#include "ros/ros.h"
#include <iostream>
#include "rigid2d/SetPose.h"
#include "nuturtle_robot/Start.h"
#include "nuturtlebot/SensorData.h"
#include "nuturtlebot/WheelCommands.h"
#include "rigid2d/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

///callback functions
bool startSrvCallback(nuturtle_robot::Start::Request &req,nuturtle_robot::Start::Response &res);
void timerCallback(const ros::TimerEvent& event);

///variables
int trans_direction=2;
int trans_count;//trans_count:current count of velocity command sent in one period.
int count_bound;//count_bound:maximum of trans_count in one period
int trans_steps;//trans_steps:number of current period. e.g.: trans_steps = 10 means 10th period
int steps_bound;//steps_bound:how many trans_steps the robot need to move
double frac_vel,max_trans_robot;//[0,1], fraction of the max trans speed of robot

///topics and services
ros::ServiceServer start_srv;
ros::Publisher vel_pub;


int main(int argc, char **argv) {
    ros::init(argc, argv, "translation");

    ///load robot param
    ros::NodeHandle params;
    params.getParam("max_trans_robot",max_trans_robot);
    frac_vel=0.2/max_trans_robot;
    //frac_vel=1;

    ///define topics and services
    ros::NodeHandle n;
    start_srv = n.advertiseService("/start", startSrvCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 120,true);

    ///define motion parameters
    ros::service::waitForService("/start");
    trans_count=0;
    trans_steps=0;
    steps_bound=10;
    count_bound=round(0.2/(frac_vel*max_trans_robot)*120);//frequency=120

    ///ros::Timer for call velocity in a constant frequency
    ros::Timer timer = n.createTimer(ros::Duration(1.0/120), timerCallback);

    ros::spin();
    return 0;
};

/// \brief start service callback function
/// call the start service to
///     1.reset the odometers (both fake and actual)
///     2.reset the move counts
///     3.set the direction of translation
/// \params  req - nuturtle_robot::Start : value is 1 or 0. 1 means moving forward, while 0 means moving backward.
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
    trans_direction=req.positive;
    if (req.positive){
        ROS_INFO("Backward Moving.");
    }
    else{
        ROS_INFO("Forward Moving.");
    }

    ///reset the move counts
    trans_steps=0;
    trans_count=0;
    return true;
}

/// \brief ros::Timer callback function
/// send velocity command in constant frequency
void timerCallback(const ros::TimerEvent& event){

    geometry_msgs::Twist ts;

    if (trans_direction==1 || trans_direction==0){// if the translation direction is set correctly
        if (trans_steps<steps_bound){//if step to move is less than max step
            trans_count+=1;
            if (trans_count<=count_bound){ //if velocity count in each step is less than max
                ts.linear.x=frac_vel*max_trans_robot*(trans_direction-0.5)*2;
                vel_pub.publish(ts);
            }
            else{
                ts.linear.x=0;
                vel_pub.publish(ts);
                if (trans_count>=count_bound/20.0*21.0){//stop for 1/20 time of one full circle
                    trans_count=0;//reset trans_count
                    trans_steps+=1;
                }
            }
        }
        else{// if the translation direction is set incorrectly, no movement
            ts.linear.x=0;
            vel_pub.publish(ts);
        }
    }
    else{
        vel_pub.publish(ts);
        ROS_INFO("waiting for start command");
    }
}
