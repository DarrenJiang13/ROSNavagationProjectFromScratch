/// \file diff_drive.cpp
/// \brief implementation of diff_drive.hpp

#include <cmath>
#include <vector>
#include <iostream>
#include "rigid2d/diff_drive.hpp" // include the header file

using namespace rigid2d;

DiffDrive::DiffDrive(){
    //wheel_base 0.16, wheel_radius 0.033
    robot_pose.w=0;
    robot_pose.vx=0;
    robot_pose.vy=0;

    robot_wheel_base = 0.160;
    robot_wheel_radius = 0.0330;
}

DiffDrive::DiffDrive(Twist2D pose, double wheel_base, double wheel_radius){
    robot_pose.w=pose.w;
    robot_pose.vx=pose.vx;
    robot_pose.vy=pose.vy;

    robot_wheel_base = wheel_base;
    robot_wheel_radius = wheel_radius;
}

WheelVelocities DiffDrive::twistToWheels(Twist2D twist){
    WheelVelocities wheel_vel_tmp;

    try{
        if(robot_wheel_radius<0.0000001){
            throw robot_wheel_radius;
        }
        //body twist to wheels
        wheel_vel_tmp.left=0.5*(2*(twist.vx)-twist.w*robot_wheel_base)/robot_wheel_radius;
        wheel_vel_tmp.right=0.5*(2*(twist.vx)+twist.w*robot_wheel_base)/robot_wheel_radius;
    }
    catch(double bad_radius){
        std::cout<<"The wheel radius "<<bad_radius<<" is too small to compute. "<<std::endl;
    }
    return wheel_vel_tmp;
}

Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel){

    Twist2D returnTwist;
    //wheels to body twist
    returnTwist.w=(vel.right-vel.left)*robot_wheel_radius/robot_wheel_base;
    returnTwist.vx=0.5*(vel.right+vel.left)*robot_wheel_radius;
    returnTwist.vy=0;

    return returnTwist;
}

void DiffDrive::updateOdometry(double left, double right){

    ///wheel velocities update
    wheel_vel.left=left-encoder.left;
    wheel_vel.right=right-encoder.right;

    ///get the current pose of robot, make it a transform matrix
    Vector2D pose_vec;
    pose_vec.x=robot_pose.vx;
    pose_vec.y=robot_pose.vy;
    double pose_angle=robot_pose.w;
    Transform2D pose_transform(pose_vec,pose_angle);

    ///update the transform matrix of robot pose with a twist
    Twist2D cmdTwist= wheelsToTwist(wheel_vel);
    pose_transform*=integrateTwist(cmdTwist);

    ///update robot pose
    Twist2D pose_update=pose_transform.displacement();
    robot_pose=pose_update;
}

void DiffDrive::feedforward(Twist2D cmd){

    /// wheel velocities update
    WheelVelocities cmdVel=twistToWheels(cmd);

    /// get the target wheel encoders, update odometry
    double wheel_left=encoder.left+cmdVel.left;
    double wheel_right=encoder.right+cmdVel.right;
    updateOdometry(wheel_left,wheel_right);

    /// update the encoder of the robot
    encoder.left=encoder.left+wheel_vel.left;
    encoder.right=encoder.right+wheel_vel.right;
}

Twist2D DiffDrive::pose(){
    Twist2D returnTwist;
    returnTwist=robot_pose;
    return returnTwist;
}

WheelVelocities DiffDrive::wheelVelocities() const{
    WheelVelocities return_wheel_vel=wheel_vel;
    return return_wheel_vel;
}

Encoder DiffDrive::wheelEncoderState() {
    Encoder return_encoder=encoder;
    return return_encoder;
}

void DiffDrive::reset(Twist2D ps){
    robot_pose.w=ps.w;
    robot_pose.vx=ps.vx;
    robot_pose.vy=ps.vy;
}
