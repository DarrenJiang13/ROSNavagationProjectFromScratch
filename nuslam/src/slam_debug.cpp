/// \file: slam_debug.cpp
/// \brief: create a ros node caller slam_debug, this node runs only when in debug mode

/// PARAMETERS:
///    odom_frame_id: frame id of  the slam, which in this node is "map"
///    base_link_id: base_link
/// PUBLISHES:
///     /trajectory (nav_msgs::Path): publish the robot trajectory with ekf slam estimation
/// SUBSCRIBES:
///     /landmarks (nuslam::TurtleMap): publish the ground truth landmarks data in gazebo(added with some noise)
///     /gazebo/model_states(gazebo_msgs::ModelStates): get the landmarks and robot position data from gazebo
///     /cmd_vel(geometry_msgs::Twist): get the command velocity of the robot.

#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include "nuslam/ekf_slam.hpp"
#include "gazebo_msgs/ModelStates.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/Twist.h"

static std::string odom_frame_id, body_frame_id;
EKF_slam ekfSlam(0.01,0.01);//define the efk slam class here. two parameters are process noise and measure noise

class Odometer{
public:
    Odometer();
private:
    //call back functions
    void velSubCallback(const geometry_msgs::Twist::ConstPtr& ts);
    void landmarkSubCallback(const nuslam::TurtleMap::ConstPtr &tm);
    void gtPoseSubCallback(const gazebo_msgs::ModelStates::ConstPtr &ms);

    //node handles
    ros::NodeHandle n;
    ros::NodeHandle odom_params;

    //topics and services and odom
    ros::Publisher odom_pub;
    tf2::Quaternion odom_quat;
    ros::Subscriber landmarks_sub;
    ros::Subscriber gt_pose_sub;
    ros::Subscriber vel_sub;
    tf2_ros::TransformBroadcaster odom_broadcaster;//broadcast transform

    nav_msgs::Path path;
    ros::Publisher path_pub;

    //diff robot parameters
    std::vector<double>center_x;
    std::vector<double>center_y;
    std::vector<double>radius;
    std::vector<int>index;
    rigid2d::Twist2D gt_pose;

    //parameters for control input
    rigid2d::Twist2D current_pose;
    rigid2d::Twist2D input_twist;
    ros::Time current_time;
    ros::Time last_time;
    double delta_time;

    //double offset_x,offset_y,offset_theta;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_debug");
    ros::NodeHandle n;
    ros::NodeHandle odom_params("~");
    odom_params.getParam("odom_frame_id",odom_frame_id);
    odom_params.getParam("body_frame_id",body_frame_id);

    ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_vel");

    Odometer odometer;

    ros::Rate my_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        my_rate.sleep();
    }
//    ros::spin();
    return 0;
};


/// \brief constructor function
/// initialize the parameters
Odometer::Odometer() {
    vel_sub = n.subscribe("cmd_vel", 10, &Odometer::velSubCallback,this);
    landmarks_sub = n.subscribe("landmarks", 10, &Odometer::landmarkSubCallback,this);
    gt_pose_sub = n.subscribe("/gazebo/model_states", 10, &Odometer::gtPoseSubCallback,this);
    path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);

    current_time=ros::Time::now();
    last_time=ros::Time::now();
};

/// \brief callback function
/// \param  ls - nuslam::TurtleMap
void Odometer::landmarkSubCallback(const nuslam::TurtleMap::ConstPtr &tm)
{
    std::vector<double>center_x_temp=tm->center_x;
    std::vector<double>center_y_temp=tm->center_y;
    std::vector<double>radius_temp=tm->radius;
    std::vector<int>index_temp=tm->index;
    center_x=center_x_temp;
    center_y=center_y_temp;
    radius=radius_temp;
    index=index_temp;
};

/// \brief callback function
/// \param  ls - nuslam::TurtleMap
void Odometer::gtPoseSubCallback(const gazebo_msgs::ModelStates::ConstPtr &ms)
{
    tf2::Quaternion q(
            ms->pose[21].orientation.x,
            ms->pose[21].orientation.y,
            ms->pose[21].orientation.z,
            ms->pose[21].orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    gt_pose.w=yaw;
    gt_pose.vx=ms->pose[21].position.x;
    gt_pose.vy=ms->pose[21].position.y;
};


/// \brief callback function
/// \param  ls - nuslam::TurtleMap
void Odometer::velSubCallback(const geometry_msgs::Twist::ConstPtr &ts)
{
    current_time=ros::Time::now();
    delta_time=current_time.toSec()-last_time.toSec();
    rigid2d::Twist2D twist={delta_time*input_twist.w,delta_time*input_twist.vx,delta_time*input_twist.vy};

    /// 1.ekf slam
    ekfSlam.setLandmarks(center_x,center_y,index);
    ekfSlam.predictKnownAssociation(twist);
    rigid2d::Twist2D ekf_pose=ekfSlam.correctionKnownAssociation();
    //rigid2d::Twist2D ekf_pose=ekfSlam.getPose();
    //get the quaternion of the current orientation of the robot
    odom_quat.setRPY(0, 0, ekf_pose.w);

    /// 2. publish the transform
    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = body_frame_id;

    odom_trans.transform.translation.x = ekf_pose.vx;
    odom_trans.transform.translation.y = ekf_pose.vy;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    /// 3. Publish the Path
    path.header.stamp=ros::Time::now();
    path.header.frame_id=odom_frame_id;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = ekf_pose.vx;
    pose_stamped.pose.position.y = ekf_pose.vy;

    pose_stamped.pose.orientation.x = odom_quat.x();
    pose_stamped.pose.orientation.y = odom_quat.y();
    pose_stamped.pose.orientation.z = odom_quat.z();
    pose_stamped.pose.orientation.w = odom_quat.w();

    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id=odom_frame_id;

    path.poses.push_back(pose_stamped);
    path_pub.publish(path);

    //input_twist={ts->angular.z,ts->linear.x,ts->linear.y};
    //These are some magic numbers. 0.90,0.94
    //I found when cmd_vel is sent simultaneously to this node (without ekf) and odometer node,
    // the robot moved in different speed. maybe because the odometer node get the speed after several format change and some twists lost.
    input_twist={0.90*ts->angular.z,0.94*ts->linear.x,0};
    last_time=current_time;
};

