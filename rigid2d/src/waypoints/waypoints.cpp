/// \file waypoints.cpp
/// \brief implementation of waypoints.hpp


#include <iosfwd> // contains forward definitions for iostream objects
#include <math.h>
#include <iostream>
#include <vector>
#include "rigid2d/waypoints.hpp" // include the header file

using namespace std;
using namespace rigid2d;

Waypoints::Waypoints(vector<Vector2D> vecTraj,double freq){
    trajectory=vecTraj;
    initial_robot_pose.w=0;
    initial_robot_pose.vx=trajectory[0].x;
    initial_robot_pose.vy=trajectory[0].y;
    delta_time=1/freq;
    diffRobot.reset(initial_robot_pose);
    max_trans_vel=0.5;
    max_rot_vel=0.5;
};

Waypoints::Waypoints(vector<Vector2D> vecTraj,double freq,double max_trans, double max_rot){
    trajectory=vecTraj;
    initial_robot_pose.w=0;
    initial_robot_pose.vx=trajectory[0].x;
    initial_robot_pose.vy=trajectory[0].y;
    delta_time=1/freq;
    diffRobot.reset(initial_robot_pose);
    max_trans_vel=max_trans;
    max_rot_vel=max_rot;
};

Waypoints::~Waypoints(){};

Twist2D Waypoints::nextWaypoint(){

    int traj_size=trajectory.size();
    Twist2D cmd_twist;
    Twist2D current_pose=diffRobot.pose();
    Vector2D current_point={current_pose.vx,current_pose.vy};
    Vector2D target_point=trajectory[target_index];
    ///set the desired twist with limit to 0.22m/s and 2.84rad/s
    double ori_goal=abs(normalize_angle(angle(target_point-current_point)-current_pose.w));

    if ((ori_goal > 0.05) & (ori_goal < 2*PI-0.05)){
        ///turn
        //magic number 0.001 to give the rotation a minimum value to prevent being stucked.
        cmd_twist.w=speed_rot_limit(normalize_angle(angle(target_point-current_point)-current_pose.w))*delta_time;
        cmd_twist.vx=0;
        cmd_twist.vy=0;
    }
    else{
        ///translate
        cmd_twist.w=0;
        cmd_twist.vx=speed_trans_limit(distance(current_point,target_point))*delta_time;
        cmd_twist.vy=0;
    }

    ///move the robot
    diffRobot.feedforward(cmd_twist);

    ///update the pose, point, and target waypoint index.
    Twist2D update_current_pose=diffRobot.pose();
    Vector2D update_current_point={update_current_pose.vx,update_current_pose.vy};
    if (distance(update_current_point,target_point)<0.012 && stop_sign==0){
        target_index+=1;
        if (target_index==traj_size){target_index=0;stop_sign=1;}//this line helps robot to move back to the first waypoint when it reaches the end
    }
    return cmd_twist;
};

double Waypoints::speed_trans_limit(double speed){
    double return_speed;
    if(abs(speed)>max_trans_vel){
        return_speed=speed/abs(speed)*max_trans_vel;
    }
    else{
        return_speed=speed;
    }

    return return_speed;
}

double Waypoints::speed_rot_limit(double speed){
    double return_speed;
    if(abs(speed)>max_rot_vel){
        return_speed=speed/abs(speed)*max_rot_vel;
    }
    else{
        return_speed=speed;
    }

    if(abs(speed)<0.01){
        return_speed=speed/abs(speed)*0.01;
    }
    else{
        return_speed=speed;
    }

    return return_speed;
}

void Waypoints::reset(bool sign){
    if (sign!=reset_sign){
        target_index=0;
        stop_sign=0;
        reset_sign=sign;
    }
};

void Waypoints::getActualPose(rigid2d::Twist2D ps){
    diffRobot.reset(ps);
};

