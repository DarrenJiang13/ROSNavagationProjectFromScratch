/// \file: turtle_rect.cpp
/// \brief: this is the .cpp file for the turtle_rect node
///
/// PARAMETERS:
///     x (float): The x coordinate of the lower left corner of a rectangle
///     y (float): The y coordinate of the lower left corner of a rectangle
///     width (float): The width of the rectangle
///     height (float): The height of the rectangle
///     trans_vel (float): The translational velocity of the robot
///     rot_vel (float): The rotational velocity of the robot
///     frequency (int): The frequency of the control loop
/// PUBLISHES:
///     vel_pub (geometry_msgs::Twist): publish the velocity to control the turtle
///     pose_error_pub (tsim::PoseError): publish the pose error of the turtle
/// SUBSCRIBES:
///     pose_sub (turtlesim::Pose): subscribe the current pose of the turtle
/// SERVICES:
///     traj_reset (std_srvs::Empty): moves the turtle to the lower-left corner of the rectangle and restarts the trajectory


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tsim/PoseError.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include <std_srvs/Empty.h>
#include <sstream>
#include <cstdlib>
#include <csignal>
using namespace std;


//method to move the robot straight
void SetMyPen(int r, int g, int b, int width, bool off);
void TeleportAbsolute(float x, float y, float theta);
void ReachGoal(double trans_speed, double rot_speed, double tolerence);
void TurtleMove(double trans_speed , double trans_distance , bool isForward);
void TurtleRot(double angular_speed , double angle , bool Clockwise);
void PoseCallback(const turtlesim::Pose::ConstPtr &pose_message);
bool TrajResetCallback(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res);
void MoveRectangle(double trans_speed,double rot_speed, double width,double height);
void press_stop(int param);

const double PI=3.1415926535;
bool stop_flag = true; //stop_flag to quit the while loop in ReachGoal function when you press "ctrl+c"
int turtle_state =0; //0: origin point; 1: 1st line of the rectangle; 2: 2nd line; 3: 3rd line; 4: 4th line.
static double x_left_corner_reset, y_left_corner_reset;

ros::Publisher vel_pub;
ros::Publisher pose_error_pub;
ros::Subscriber pose_sub;
turtlesim::Pose pose_message_recieve;
turtlesim::Pose goal_pose;


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "turtle_rect");

  	/** 1. get parameters from my .yaml files.
  	/*  In this part, the .launch file load parameters from .yaml file to the rosparam server.
  	/*  Then I use the node turtle_params to acquire the data.
  	/*  https://roboticsbackend.com/ros-param-yaml-format/
  	 */
    double x_left_corner, y_left_corner, width, height, trans_vel, rot_vel, frequency;
    ros::NodeHandle turtle_params;
    turtle_params.getParam("/x", x_left_corner);
    turtle_params.getParam("/y", y_left_corner);
    turtle_params.getParam("/width", width);
    turtle_params.getParam("/height", height);
    turtle_params.getParam("/trans_vel", trans_vel);
    turtle_params.getParam("/rot_vel", rot_vel);
    turtle_params.getParam("/frequency", frequency);
    ros::Rate loop_rate(0.5);
    x_left_corner_reset= x_left_corner;
    y_left_corner_reset= y_left_corner;

    ros::NodeHandle turtle_rect;
    //ros::NodeHandle turtle_ctrl;

    //reset service
    ros::ServiceServer traj_reset = turtle_rect.advertiseService("/traj_reset", TrajResetCallback);

		vel_pub = turtle_rect.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", frequency);
    pose_error_pub = turtle_rect.advertise<tsim::PoseError>("/pose_error", frequency);
    pose_sub = turtle_rect.subscribe("/turtle1/pose" , 1000 , PoseCallback);
    //pose_error =turtle_rect.advertise<geometry_msgs::Pose2D>("/pose_error", frequency);

    /**2. Lift the pen,  teleport the turtle to the left corner,  put the pen down.
     *  https://github.com/abmoRobotics/P1vdfdfgsdfg/blob/f4ba802a7cd5280abba9bcab1816a366164aaed3/src/communication.cpp
     */
    SetMyPen(255, 255, 255, 2, 1);
    goal_pose.x = x_left_corner;
    goal_pose.y = y_left_corner;
    goal_pose.theta = 0;
    ReachGoal(trans_vel, rot_vel, 0.1);

    SetMyPen(255, 255, 255, 2, 0);

    /**3. Move in Rectangle.
     *
     */
    MoveRectangle(trans_vel, rot_vel, width, height);

    ros::spin();
    return 0;
}

void SetMyPen(int r, int g, int b, int width, bool off){

    /// \brief call the set_pen service of the turtlesim_node
    ///
    /// \param r,g,b - color of the pen.
    /// \param width - width of the pen.
    /// \param off - lift or put down the pen.(lift:1,put down:0)
    /// \return none
    ///  SetMyPen(255, 255, 255, 2, 1)
    ///http://docs.ros.org/melodic/api/roscpp/html/namespaceros_1_1service.html
    //ros::service::waitForService("/turtle1/set_pen");
    //ros::ServiceClient turtle_ctrl_client = turtle_ctrl.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

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

void TeleportAbsolute(float x, float y, float theta){

    /// \brief call the teleport_absolute service of the turtlesim_node
    ///
    /// \param x - x of the goal pose.
    /// \param y - y of the goal pose.
    /// \param theta - theta of the goal pose
    /// \return none
    ///  TeleportAbsolute(1, 3, 2)

//    ros::service::waitForService("/turtle1/teleport_absolute");
//    ros::ServiceClient turtle_client_move = turtle_ctrl.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

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

void TurtleMove(double trans_speed, double trans_distance, bool isForward){

    /// \brief Translation the turtle in a relative distance.
    ///
    /// \param trans_speed - translation speed of the turtle.
    /// \param trans_distance - the distance to move.
    /// \param isForward - whether the rotation is forward or backward. (forward:1, backward:0)
    /// \return none
    ///  TurtleMove (2.0, 1.0, 1)

    geometry_msgs::Twist vel_msg;

    // set the linear velocity
    if (isForward)
        vel_msg.linear.x=std::abs(trans_speed);
    else
        vel_msg.linear.x=-std::abs(trans_speed);

    vel_msg.linear.y=0;
    vel_msg.linear.z=0;

    // set the angular velocity
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    vel_msg.angular.z=0;

    //t0: current time
    double t0=ros::Time::now().toSec();
    double current_distance=0.0;
    ros::Rate loop_rate(10);

    //loop if current distance of robot<=distance
    //publish the velocity
    //estimate the distance=speed*(t1-t0)
    while(current_distance < trans_distance){
        vel_pub.publish(vel_msg);
        double t1=ros::Time::now().toSec();
        current_distance=trans_speed*(t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //reset the velocity to 0 when loop ends.
    vel_msg.linear.x=0;
    vel_pub.publish(vel_msg);
}

void TurtleRot(double angular_speed , double angle , bool Clockwise)
{
    /// \brief Rotation the turtle in a relative angle
    ///
    /// \param angular_speed - angular speed of the turtle.
    /// \param angular - the angle to rotate.
    /// \param Clockwise - whether the rotation is clockwise or anticlockwise. (clockwise:1, anticlockwise:0)
    /// \return none
    /// TurtleRot (2.0, 1.0, 1)

    geometry_msgs::Twist ang_msg;

    ang_msg.linear.x = 0;
    ang_msg.linear.y = 0;
    ang_msg.linear.z = 0;

    ang_msg.angular.x = 0;
    ang_msg.angular.y = 0;

    if(Clockwise)
    {
        ang_msg.angular.z = -std::abs(angular_speed);
    }
    else
    {
        ang_msg.angular.z = std::abs(angular_speed);
    }

    double t0 = ros::Time::now().toSec();
    double current_angle = 0;
    ros::Rate loop_rate(1000);
    do
    {
        vel_pub.publish(ang_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = (t1 - t0) * angular_speed;
    }while(angle > current_angle);

    ang_msg.angular.z=0;
    vel_pub.publish(ang_msg);
}

void PoseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
    /// \brief callback function of the pose subscriber. Assign values to the message pose_message_recieve
    ///

    pose_message_recieve.x = pose_message->x;
    pose_message_recieve.y = pose_message->y;
    pose_message_recieve.theta = pose_message->theta;
    //printf("%lf , %lf , %lf \n",pose_message_recieve.x , pose_message_recieve.y , pose_message_recieve.theta );
}

void ReachGoal(double trans_speed, double rot_speed, double tolerence)
{
    /// \brief Go to the estimated destination.
    ///
    /// \param trans_speed - translation speed of the turtle.
    /// \param rot_speed - rotation speed of the turtle.
    /// \param tolerance - tolerance for the trajectory.
    /// \return empty
    /// ReachGoal (2.0, 1.0, 0.1)

    geometry_msgs::Twist vel;
    tsim::PoseError pose_error;
    ros::Rate loop_rate(100);
    signal(SIGINT, press_stop);
    ROS_INFO("Moving to x: %f, y: %f, theta: %f.",goal_pose.x, goal_pose.y, goal_pose.theta);

    int turtle_state_tmp=turtle_state;
    float ori_to_goal=1;
    while((ori_to_goal > 0.01) && (ori_to_goal < 2*PI+0.01) && stop_flag){
        if (turtle_state!=turtle_state_tmp) break;

        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;

        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = rot_speed;
        //vel.angular.z = 4* (atan2((goal_pose.y - pose_message_recieve.y),(goal_pose.x - pose_message_recieve.x)) - pose_message_recieve.theta);

        vel_pub.publish(vel);

        ori_to_goal=std::abs(atan2((goal_pose.y - pose_message_recieve.y),(goal_pose.x - pose_message_recieve.x)) - pose_message_recieve.theta);
        //ROS_INFO("pose_message_recieve.theta: %f, ori_to_goal: %f",pose_message_recieve.theta, ori_to_goal);

        pose_error.x_error=std::abs(goal_pose.x - pose_message_recieve.x);
        pose_error.y_error=std::abs(goal_pose.y - pose_message_recieve.y);
        pose_error.theta_error=std::abs(goal_pose.theta - pose_message_recieve.theta);
        pose_error_pub.publish(pose_error);

        ros::spinOnce();
        loop_rate.sleep();
    }


    float distance_to_goal=10000;
    while((distance_to_goal > tolerence) && stop_flag){
        if (turtle_state!=turtle_state_tmp) break;
        //vel.linear.x = 1 * sqrt(pow((pose_message_recieve.x - goal_pose.x) , 2 ) + pow((pose_message_recieve.y - goal_pose.y) , 2));
        vel.linear.x = trans_speed;
        vel.linear.y = 0;
        vel.linear.z = 0;

        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        //vel.angular.z = 4* (-pose_message_recieve.theta + goal_pose.theta);
        //printf("%lf , %lf \n",vel.linear.x ,vel.angular.z );
        vel_pub.publish(vel);

        distance_to_goal=sqrt(pow((pose_message_recieve.x - goal_pose.x) , 2 )
                 + pow((pose_message_recieve.y - goal_pose.y) , 2));

        pose_error.x_error=std::abs(goal_pose.x - pose_message_recieve.x);
        pose_error.y_error=std::abs(goal_pose.y - pose_message_recieve.y);
        pose_error.theta_error=std::abs(goal_pose.theta - pose_message_recieve.theta);
        pose_error_pub.publish(pose_error);

        //ROS_INFO("distance_to_goal: %f",distance_to_goal);
        ros::spinOnce();
        loop_rate.sleep();
    }

    vel.linear.x =0;
    vel.angular.z = 0;
    vel_pub.publish(vel);
    loop_rate.sleep();
}

void MoveRectangle(double trans_speed,double rot_speed, double width, double height){

    /// \brief Make the turtle move in a rectangular trajectory.
    ///
    /// \param trans_speed - translation speed of the turtle.
    /// \param rot_speed - rotation speed of the turtle.
    /// \param width - width of the rectangle.
    /// \param height - height of the rectangle.
    /// \return empty
    /// MoveRectangle (2.0, 1.0, 2, 8)

    double x_left_corner=goal_pose.x;
    double y_left_corner=goal_pose.y;
    signal(SIGINT, press_stop);

    while(stop_flag){
        if (turtle_state==0){
            turtle_state+=1;
            goal_pose.x = x_left_corner+width;
            goal_pose.y = y_left_corner;
            goal_pose.theta = 0;
            ReachGoal(trans_speed, rot_speed, 0.1);
        }
        else if (turtle_state==1){
            turtle_state+=1;
            goal_pose.y = y_left_corner+height;
            goal_pose.theta += PI/2;
            ReachGoal(trans_speed, rot_speed, 0.1);
        }

        else if (turtle_state==2){
            turtle_state+=1;
            goal_pose.x = x_left_corner;
            goal_pose.theta = goal_pose.theta+PI/2-2*PI;
            ReachGoal(trans_speed, rot_speed, 0.1);
        }

        else if (turtle_state==3){
            turtle_state+=1;
            goal_pose.y = y_left_corner;
            goal_pose.theta += PI/2;
            ReachGoal(trans_speed, rot_speed, 0.1);
        }

        else if (turtle_state==4){
            ROS_INFO("Destination reached, congratulations!");
            turtle_state=0;
            //break;
        }
        else{
            ROS_INFO("SOS! Wrong turtle_state: %d!",turtle_state);
            break;
        }
    }


//    TurtleMove(trans_speed, width, 1);
//    TurtleRot(rot_speed, PI/2, 0);
//    TurtleMove(trans_speed, height, 1);
//    TurtleRot(rot_speed, PI/2, 0);
//    TurtleMove(trans_speed, width, 1);
//    TurtleRot(rot_speed, PI/2, 0);
//    TurtleMove(trans_speed, height, 1);
//    TurtleRot(rot_speed, PI/2, 0);
}

void press_stop(int param) {

    /// \brief Stop the while loop in ReachGoal function when press "ctrl+c"
    ///
    /// \param param - do not bother.
    /// \return none

    stop_flag = false;
}

bool TrajResetCallback(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res){

    /// \brief callback function of the traj_reset service.
    ///  Moves the turtle to the lower-left corner of the rectangle and restarts the trajectory.
    ///

    turtlesim::TeleportAbsolute teleportAbsolute;
    teleportAbsolute.request.x=x_left_corner_reset;
    teleportAbsolute.request.y=y_left_corner_reset;
    teleportAbsolute.request.theta=0;

    ros::service::waitForService("/turtle1/teleport_absolute");

    SetMyPen(255, 255, 255, 2, 1);

    if (ros::service::call("/turtle1/teleport_absolute",teleportAbsolute)){
        ROS_INFO("Trajectory Reset");
        SetMyPen(255, 255, 255, 2, 0);
        turtle_state=0;
    }
    else{
        ROS_ERROR("Failed to call service traj_reset");
    }

    return true;
}
