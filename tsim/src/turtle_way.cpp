/// \file: turtle_way.cpp
/// \brief: create a new node called turtle_way to make the turtle follow a trajectory of user-specified waypoints
/// PARAMETERS:
///    empty
/// PUBLISHES:
///     /cmd_vel (geometry_msgs::Twist): publish the command velocity
///     /poseError (tsim::PoseError): publish the error between estimated pose and actual pose
/// SUBSCRIBES:
///     /turtle1/pose (turtlesim::Pose): to get the actual pose of the turtlesim

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include "tsim/PoseError.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include <std_srvs/Empty.h>
#include <rigid2d/waypoints.hpp>
#include <rigid2d/rigid2d.hpp> // include the header file

using namespace std;
using namespace rigid2d;

void SetMyPen(int r, int g, int b, int width, bool off);
void TeleportAbsolute(float x, float y, float theta);
void PoseCallback(const turtlesim::Pose::ConstPtr &pose_message);

turtlesim::Pose pose_message_recieve;
ros::Subscriber pose_sub;
ros::Publisher vel_pub;
ros::Publisher pose_error_pub;

int main(int argc, char **argv) {

    ros::init(argc, argv, "turtle_way");
    ros::NodeHandle n;
    ros::NodeHandle poseError;
    int freq_cmd=60;

    /// 1. define a waypoints class with input points
    vector<Vector2D> vecTraj;
    Vector2D vec_tmp;
    double waypoint_x[]={6,6.618034,5,3.381966,4};
    double waypoint_y[]={2,3.902113,5.0776835,3.902113,2};
    for(int i=0;i<sizeof(waypoint_x)/sizeof(waypoint_x[0]);i++){
        vec_tmp.x=waypoint_x[i];
        vec_tmp.y=waypoint_y[i];
        vecTraj.push_back(vec_tmp);
    }
    Waypoints myWaypoints(vecTraj,freq_cmd);

    /// 2. define nodes to pub or sub
    pose_sub = n.subscribe("/turtle1/pose" , 100 , PoseCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_error_pub = poseError.advertise<tsim::PoseError>("/pose_error", 1000);

    /// 3. Lift the pen,  teleport the turtle to the left corner,  put the pen down.
    SetMyPen(255, 255, 255, 2, 1);
    TeleportAbsolute(waypoint_x[0],waypoint_y[0],0.08);//offset
    SetMyPen(255, 255, 255, 2, 0);

    /// 4. Main part
    ros::Rate loop_rate(freq_cmd);
    Twist2D wpTwist;
    Twist2D expect_pose;
    tsim::PoseError pose_error;
    geometry_msgs::Twist vel;

    while(ros::ok()){
        /// get command velocity in waypoints
        wpTwist=myWaypoints.nextWaypoint();

        /// publish command velocity to turtlesim
        vel.linear.x = wpTwist.vx*freq_cmd;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = wpTwist.w*freq_cmd;
        vel_pub.publish(vel);

        /// publish pose error
        expect_pose=myWaypoints.diffRobot.pose();
        pose_error.x_error=abs(expect_pose.vx - pose_message_recieve.x);
        pose_error.y_error=abs(expect_pose.vy - pose_message_recieve.y);
        pose_error.theta_error=abs(normalize_angle(expect_pose.w - pose_message_recieve.theta));
        pose_error_pub.publish(pose_error);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

/// \brief call the set_pen service of the turtlesim_node
///
/// \param r,g,b - color of the pen.
/// \param width - width of the pen.
/// \param off - lift or put down the pen.(lift:1,put down:0)
/// \return none
///  SetMyPen(255, 255, 255, 2, 1)
///http://docs.ros.org/melodic/api/roscpp/html/namespaceros_1_1service.html
void SetMyPen(int r, int g, int b, int width, bool off){
    turtlesim::SetPen setPen;
    setPen.request.r=r;
    setPen.request.g=g;
    setPen.request.b=b;
    setPen.request.width=width;
    setPen.request.off= off;

    ros::service::waitForService("/turtle1/set_pen");
    if (ros::service::call("/turtle1/set_pen",setPen)){
        if(off==1)
            ROS_INFO("Lift the pen.");
        else
            ROS_INFO("Put down the pen.");
    }
    else{
        ROS_ERROR("Failed to call service set_pen.");
        ros::Rate loop_rate(0.5);//wait for some time for the ROS_ERROR.
    }
}


/// \brief call the teleport_absolute service of the turtlesim_node
///
/// \param x - x of the goal pose.
/// \param y - y of the goal pose.
/// \param theta - theta of the goal pose
/// \return none
///  TeleportAbsolute(1, 3, 2)
void TeleportAbsolute(float x, float y, float theta){
    turtlesim::TeleportAbsolute teleportAbsolute;
    teleportAbsolute.request.x=x;
    teleportAbsolute.request.y=y;
    teleportAbsolute.request.theta=theta;

    ros::service::waitForService("/turtle1/teleport_absolute");

    if (ros::service::call("/turtle1/teleport_absolute",teleportAbsolute)){
        ROS_INFO("Teleport the turtle to the corner x: %f, y: %f.",x,y);
    }
    else{
        ROS_ERROR("Failed to call service teleport_absolute");//wait for some time for the ROS_ERROR.
    }
}

/// \brief callback function of the pose subscriber. Assign values to the message pose_message_recieve
void PoseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
    pose_message_recieve.x = pose_message->x;
    pose_message_recieve.y = pose_message->y;
    pose_message_recieve.theta = pose_message->theta;
}