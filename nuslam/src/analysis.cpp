/// \file: analysis.cpp
/// \brief: create a ros node called analysis. This analysis node get ground truth landmarks data and robot pose in gaze and generate a stream of fake data.
///
/// PARAMETERS:
///    radius: fake scanning radius of the laser.You also set it to INFINITY
///    noise: noise level. random noise belong to noise*[-0.01,0.01].
/// PUBLISHES:
///     /landmarks (nuslam::TurtleMap): publish the ground truth landmarks data in gazebo(added with some noise)
///     /trajectory (nav_msgs::Path): publish the ground truth robot trajectory in gazebo
/// SUBSCRIBES:
///     /gazebo/model_states(gazebo_msgs::ModelStates): get the landmarks and robot position data from gazebo

#include "ros/ros.h"
#include <iostream>
#include "gazebo_msgs/ModelStates.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nuslam/TurtleMap.h"
#include <Eigen/Dense>
#include <random>

class Analysis{
public:
    Analysis(float radius, float noise);
    void modelStateSubCallback(const gazebo_msgs::ModelStates::ConstPtr& ms);

private:
    ros::NodeHandle n;
    ros::Subscriber model_state_sub;
    ros::Publisher landmark_pub;

    tf2_ros::TransformBroadcaster odom_broadcaster;//broadcast transform
    nav_msgs::Odometry odom;

    nav_msgs::Path path;
    ros::Publisher path_pub;

    double radius;//fake scan radius of the laser,you can even set it to INFINITY in launch file
    double noise;//noise in [-0.01,0.01] added to the fake landmark，

    std::mt19937 rng;//for generating random noise
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "analysis");

    float radius,noise;
    ros::NodeHandle params("~");
    params.getParam("radius",radius);
    params.getParam("noise",noise);

    Analysis analysis(radius,noise);

    ros::Rate my_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        my_rate.sleep();
    }
    return 0;
};

/// \brief constructor
/// \param  radius_ini - fake scanning radius
/// \param  noise_ini - noise level
Analysis::Analysis(float radius_ini, float noise_ini){
    model_state_sub=n.subscribe("/gazebo/model_states", 100, &Analysis::modelStateSubCallback,this);
    landmark_pub = n.advertise<nuslam::TurtleMap>("landmarks", 100,true);
    path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
    radius=radius_ini;
    noise=noise_ini;
}

/// \brief gazebo model states call back
/// \param  ms - ground truth states of the models in gazebo world
void Analysis::modelStateSubCallback(const gazebo_msgs::ModelStates::ConstPtr& ms){

    /// 1. publish the transform, which publish the ground truth robot pose in gazebo
    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "real_base_link";

    odom_trans.transform.translation.x = ms->pose[21].position.x;
    odom_trans.transform.translation.y = ms->pose[21].position.y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = ms->pose[21].orientation.x;
    odom_trans.transform.rotation.y = ms->pose[21].orientation.y;
    odom_trans.transform.rotation.z = ms->pose[21].orientation.z;
    odom_trans.transform.rotation.w = ms->pose[21].orientation.w;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    /// 2. publish the ground truth landmarks
    nuslam::TurtleMap turtle_map;

    rng.seed(std::random_device()());
    std::uniform_real_distribution<double> distribution(-0.01, 0.01);

    tf2::Quaternion q(
            ms->pose[21].orientation.x,
            ms->pose[21].orientation.y,
            ms->pose[21].orientation.z,
            ms->pose[21].orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    for(int i=9;i<21;i++){
        double scan_distance=pow(ms->pose[i].position.x-ms->pose[21].position.x,2)+pow(ms->pose[i].position.y-ms->pose[21].position.y,2);
        double delta_theta=atan2(ms->pose[i].position.y-ms->pose[21].position.y,ms->pose[i].position.x-ms->pose[21].position.x)-yaw;
        if (scan_distance<radius*radius){
//            turtle_map.center_x.push_back(ms->pose[i].position.x+noise*distribution(rng));
//            turtle_map.center_y.push_back(ms->pose[i].position.y+noise*distribution(rng));
            turtle_map.center_x.push_back(sqrt(scan_distance)*cos(delta_theta)+noise*distribution(rng));
            turtle_map.center_y.push_back(sqrt(scan_distance)*sin(delta_theta)+noise*distribution(rng));
            turtle_map.radius.push_back(0.03);
            turtle_map.index.push_back(i-9);
        }
    }
    landmark_pub.publish(turtle_map);


    /// 3. Publish the Path
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = ms->pose[21].position.x;
    pose_stamped.pose.position.y = ms->pose[21].position.y;

    pose_stamped.pose.orientation.x = ms->pose[21].orientation.x;
    pose_stamped.pose.orientation.y = ms->pose[21].orientation.y;
    pose_stamped.pose.orientation.z = ms->pose[21].orientation.z;
    pose_stamped.pose.orientation.w = ms->pose[21].orientation.w;

    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id="map";

    path.poses.push_back(pose_stamped);
    path_pub.publish(path);

}

