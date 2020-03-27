/// \file: real_waypoint.cpp
/// \brief: create a new node called real_waypoint to make the turtlebot follow a trajectory of user-specified waypoints
///
/// PARAMETERS:
///    encoder_ticks_per_rev: the number of encoder ticks per revolution of the wheels.
///    max_trans_robot: maximum translational velocity of the robot in m/s
///    max_rot_robot: maximum rotational velocity of the robot, in rad/s
///    max_rot_motor: maximum rotational velocity of the motor, in rad/s
///    odometer/odom_frame_id: "odom"
///    frac_vel: [0,1], fraction of the max trans speed of robot
///    waypoint_x(private): x postion of the waypoints list
///    waypoint_y(private): y postion of the waypoints list
/// PUBLISHES:
///     /cmd_vel (geometry_msgs::Twist): publish the command velocity
///     /visualization_marker (visualization_msgs::Marker): publish the postion of the marker in rviz
/// SUBSCRIBES:
///     /odom (nav_msgs::Odometry): to get the actual pose of the turtlesim
/// SERVICES:
///     /start (nuturtle_robot::Start): start the robot and reset the odometry
///     /stop (std_srvs::Empty): stop the robot and reset the odometry

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nuturtle_robot/Start.h"
#include "rigid2d/SetPose.h"
#include <rigid2d/waypoints.hpp>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rigid2d/rigid2d.hpp> // include the header file
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Matrix3x3.h>

///callback functions
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_message);
bool startSrvCallback(nuturtle_robot::Start::Request &req,nuturtle_robot::Start::Response &res);
bool stopSrvCallback(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res);

///robot parameters
double max_trans_robot,max_rot_robot,max_rot_motor;
std::string odom_frame_id;
int encoder_ticks_per_rev;
double frac_vel;
int freq_cmd=120;//command frequency
int start_sign=-1;//start sign: give 1 to start
int mark_sign=-1;//mark sign: give 1 to publish the mark
bool reset_sign=1;//reset sign: a reset sign to reset the pose in waypoint class
rigid2d::Twist2D turtlebot_pose;//turtlebot pose in real world
std::vector<double> waypoint_x;//x position of waypoint
std::vector<double> waypoint_y;//y position of waypoint
std::vector<rigid2d::Vector2D> vecTraj;//2D trajectory of waypoint

///topics and services
ros::Publisher marker_pub;//for rviz marker
uint32_t shape;//for rviz marker
ros::ServiceServer start_srv;//start service
ros::ServiceServer stop_srv;//stop service
ros::Subscriber tbot_pose_sub;//get turtlebot pose in real world
ros::Publisher vel_pub;//velocity pub


int main(int argc, char **argv) {

    ros::init(argc, argv, "real_waypoint");
    ros::NodeHandle n;

    ///1. load robot parameters
    ros::NodeHandle params;
    params.getParam("encoder_ticks_per_rev",encoder_ticks_per_rev);
    params.getParam("max_trans_robot",max_trans_robot);
    params.getParam("max_rot_robot",max_rot_robot);
    params.getParam("max_rot_motor",max_rot_motor);
    params.getParam("odometer/odom_frame_id",odom_frame_id);
    params.getParam("frac_vel",frac_vel);
    ros::NodeHandle params_pri("~");
    params_pri.getParam("waypoint_x", waypoint_x);
    params_pri.getParam("waypoint_y", waypoint_y);

    ///2.generate waypoints set for waypoints, create waypoint class
    rigid2d::Vector2D vec_tmp;
    for(unsigned i=0;i < waypoint_x.size();i++){
        vec_tmp.x=waypoint_x[i];
        vec_tmp.y=waypoint_y[i];
        vecTraj.push_back(vec_tmp);
    }
    rigid2d::Waypoints myWaypoints(vecTraj,freq_cmd,max_trans_robot,max_rot_robot);

    /// 3. define nodes to pub or sub
    shape = visualization_msgs::Marker::CYLINDER;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1,true);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    tbot_pose_sub=n.subscribe("odom_pub" , 100 , odomCallback);
    stop_srv = n.advertiseService("stop", stopSrvCallback);
    start_srv = n.advertiseService("start", startSrvCallback);

    /// 4. Main part
    ros::Rate loop_rate(freq_cmd);
    rigid2d::Twist2D wpTwist;
    geometry_msgs::Twist vel;

    while(ros::ok()){
        myWaypoints.reset(reset_sign);//if call /start , reset_sign changes, then waypoints reset
        if (start_sign==1){//if motion enabled
            /// get command velocity in waypoints
            myWaypoints.getActualPose(turtlebot_pose);
            wpTwist=myWaypoints.nextWaypoint();
            /// publish command velocity to turtlebot
            vel.linear.x = wpTwist.vx*freq_cmd;
            vel.linear.y = 0;
            vel.linear.z = 0;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = wpTwist.w*freq_cmd;
            vel_pub.publish(vel);
        }
        else{
            /// publish command velocity to turtlebot
            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.linear.z = 0;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;
            vel_pub.publish(vel);
        }

        if (mark_sign==1){//if mark enabled
            ///marker
            for(unsigned i=0;i < waypoint_x.size();i++){
                visualization_msgs::Marker marker;
                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = odom_frame_id;
                marker.header.stamp = ros::Time::now();

                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                marker.ns = "basic_shapes";
                marker.id = i;

                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                marker.type = shape;

                // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                marker.pose.position.x = waypoint_x[i];
                marker.pose.position.y = waypoint_y[i];
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.8;

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration();
                marker_pub.publish(marker);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}

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

    ///start the motion, reverse the reset sign of waypoint
    if (req.positive){
        start_sign=1;
        reset_sign=!reset_sign;
        ROS_INFO("Start Moving.");
    }
    else{
        start_sign=0;
        ROS_INFO("Stop Moving.");
    }

    ///mark enabled
    mark_sign=1;
    return true;
}

/// \brief stop service callback function
/// call the stop service to
///     1.reset the odometers (both fake and actual)
///     2.disabled the motion
bool stopSrvCallback(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res){\

    /// reset the odometers (both fake and actual)
    rigid2d::SetPose setPose;
    setPose.request.w=0;
    setPose.request.vx=0;
    setPose.request.vy=0;
    ros::service::call("/set_pose",setPose);
    ros::service::call("/fake/set_pose",setPose);

    ///motion disabled.
    start_sign=0;
    return true;
}

/// \brief pose subscriber callback function
/// subscribe to the /odom topic
///     1.get the angle from quaternion
///     2.return the turtlebot pose in real world
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_message){
    ///get quaternion from odom_message, transfer it to euler angle
    tf2::Quaternion q(
            odom_message->pose.pose.orientation.x,
            odom_message->pose.pose.orientation.y,
            odom_message->pose.pose.orientation.z,
            odom_message->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ///return turtlebot pose in real world
    turtlebot_pose.w=yaw;
    turtlebot_pose.vx=odom_message->pose.pose.position.x;
    turtlebot_pose.vy=odom_message->pose.pose.position.y;
}

