/// \file: odometer.cpp
/// \brief: create a ros node caller odometer
///
/// PARAMETERS:
///    odom_frame_id: The name of the odometry tf frame
///    body_frame_id: The name of the body tf frame
///    left_wheel_joint: The name of the left wheel joint
///    right_wheel_joint: The name of the right wheel joint
/// PUBLISHES:
///     /odom_pub (nav_msgs::Odometry): publish the position and velocity of the robot
/// SUBSCRIBES:
///     /joint_states (sensor_msgs::JointState): subscribe the joint_states to get the wheel encoder value .
///                                             then publish the odom and the transform betweeen odom_frame and body_link(body_frame).

#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rigid2d/SetPose.h"
#include "rigid2d/diff_drive.hpp"
#include <nav_msgs/Path.h>

static std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;

class Odometer{
public:
    Odometer();
private:
    ///call back functions
    void odomSubCallback(const sensor_msgs::JointState::ConstPtr& jts);
    bool setPose(rigid2d::SetPose::Request &req,rigid2d::SetPose::Response &res);

    ///node handles
    ros::NodeHandle n;
    ros::NodeHandle odom_params;

    ///topics and services and odom
    ros::Publisher odom_pub;
    ros::Subscriber odom_sub;
    tf2::Quaternion odom_quat;
    tf2_ros::TransformBroadcaster odom_broadcaster;//broadcast transform
    ros::ServiceServer set_pose;
    nav_msgs::Odometry odom;

    nav_msgs::Path path;
    ros::Publisher path_pub;

    ///diff robot parameters
    rigid2d::DiffDrive intRobot;
    double wheel_base, wheel_radius;
    rigid2d::Encoder odom_encoder={0,0};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;
    ros::NodeHandle odom_params("~");
    odom_params.setParam("left_wheel_joint","left_wheel_axle");
    odom_params.setParam("right_wheel_joint","right_wheel_axle");
    odom_params.getParam("odom_frame_id",odom_frame_id);
    odom_params.getParam("body_frame_id",body_frame_id);
    odom_params.getParam("left_wheel_joint",left_wheel_joint);
    odom_params.getParam("right_wheel_joint",right_wheel_joint);

    Odometer odometer;
    ros::Rate my_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        my_rate.sleep();
    }
    //ros::spin();
    return 0;
};


/// \brief constructor function
/// initialize the parameters
Odometer::Odometer() {
    
    odom_params.getParam("/wheel_base", wheel_base);
    odom_params.getParam("/wheel_radius", wheel_radius);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom_pub", 300,true);
    odom_sub = n.subscribe("joint_states", 100, &Odometer::odomSubCallback,this);
    path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
    set_pose= n.advertiseService("set_pose", &Odometer::setPose,this);
};


/// \brief callback function
/// \param  jts - sensor_msgs::JointState
void Odometer::odomSubCallback(const sensor_msgs::JointState::ConstPtr& jts)
{

    /// 1.update the internal odometry state
    double wheel_left_diff = jts->position[0]-odom_encoder.left;
    double wheel_right_diff = jts->position[1]-odom_encoder.right;
    rigid2d::WheelVelocities  wheel_vel={wheel_left_diff,wheel_right_diff};
    rigid2d::Twist2D twist=intRobot.wheelsToTwist(wheel_vel);
    //ROS_INFO("twist %f, %f, %f",twist.w,twist.vx,twist.vy);
    intRobot.feedforward(twist);
    //update encoder
    odom_encoder.left=jts->position[0];
    odom_encoder.right=jts->position[1];
    //get the quaternion of the current orientation of the robot
    odom_quat.setRPY(0, 0, intRobot.pose().w);

    /// 2. publish odom
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;

    //set the position
    odom.pose.pose.position.x = intRobot.pose().vx;
    odom.pose.pose.position.y = intRobot.pose().vy;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();

    //set the velocity
    odom.child_frame_id = body_frame_id;
    odom.twist.twist.linear.x = twist.vx;
    odom.twist.twist.linear.y = twist.vy;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = twist.w;
    //publish the message
    odom_pub.publish(odom);

    /// 3. publish the transform
    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = body_frame_id;

    odom_trans.transform.translation.x = intRobot.pose().vx;
    odom_trans.transform.translation.y = intRobot.pose().vy;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    /// 4. Publish the Path
    path.header.stamp=ros::Time::now();
    path.header.frame_id=odom_frame_id;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = intRobot.pose().vx;
    pose_stamped.pose.position.y = intRobot.pose().vy;

    pose_stamped.pose.orientation.x = odom_quat.x();
    pose_stamped.pose.orientation.y = odom_quat.y();
    pose_stamped.pose.orientation.z = odom_quat.z();
    pose_stamped.pose.orientation.w = odom_quat.w();

    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id="odom";

    path.poses.push_back(pose_stamped);
    path_pub.publish(path);

};

/// \brief setpose function
/// \param  &req - rigid2d::SetPose set the pose of internal robot for some service like start and stop.
bool Odometer::setPose(rigid2d::SetPose::Request &req,rigid2d::SetPose::Response &res)
{
    rigid2d::Twist2D pose;
    pose.w=req.w;
    pose.vx=req.vx;
    pose.vy=req.vy;
    intRobot.reset(pose);
    return true;
}

