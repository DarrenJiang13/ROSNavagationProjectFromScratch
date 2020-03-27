/// \file: turtle_interface_test.cpp
/// \brief: create a ros node caller turtle_interface_test
///
/// PARAMETERS:
///    encoder_ticks_per_rev: the number of encoder ticks per revolution of the wheels.
///    max_trans_robot: maximum translational velocity of the robot in m/s
///    max_rot_robot: maximum rotational velocity of the robot, in rad/s
///    max_rot_motor: maximum rotational velocity of the motor, in rad/s
/// PUBLISHES:
///     /wheel_cmd (nuturtlebot::WheelCommands): transform the command twist to command velocity of wheels and publish.
///     /joint_states (sensor_msgs::JointState): transform the sensor data to joint states and publish.
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs::Twist): get the command velocity of the robot
///     /sensor_data (nuturtlebot::SensorData): get the sensor data of the robot

#include "ros/ros.h"
#include <iostream>
#include <gtest/gtest.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/diff_drive.hpp"

///helper functions
geometry_msgs::Twist createTwist(double vx, double wz);
nuturtlebot::SensorData createSensorData(int l_encoder, int r_encoder);

class Interface{
public:
    Interface();
private:
    ///functions
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& ts);
    void sensorDataCallback(const nuturtlebot::SensorData::ConstPtr& sd);
    geometry_msgs::Twist limitInput(const geometry_msgs::Twist::ConstPtr& ts);
    nuturtlebot::WheelCommands limitOutput(const rigid2d::WheelVelocities wheel_vel);

    ///node handles
    ros::NodeHandle n;
    ros::NodeHandle params;

    ///robot parameters
    double max_trans_robot,max_rot_robot,max_rot_motor;
    int encoder_ticks_per_rev;
    int left_encoder, right_encoder, left_offset, right_offset;
    ros::Time time_stamp_encoder=ros::Time::now();

    ///topics
    ros::Subscriber vel_sub;
    ros::Subscriber sensor_sub;
    ros::Publisher wheel_pub;
    ros::Publisher js_pub;
    sensor_msgs::JointState js;

    ///robot model
    rigid2d::DiffDrive intRobot;//only for using the function twisttowheels
};

class InterfaceFixture : public ::testing::Test {
    protected:
        geometry_msgs::Twist ts_trans, ts_rot, ts_trans_rot;
        nuturtlebot::SensorData sd;

    // Setup
    InterfaceFixture() {
        ts_trans = createTwist(1,0);
        ts_rot = createTwist(0,1);
        ts_trans_rot = createTwist(1,1);
        sd=createSensorData(2000,3000);
    };
};

/// \brief test translation of the robot
/// \param TestSuite, testTrans
/// \returns EXPECT_EQ
TEST_F(InterfaceFixture, TestTrans) {
    ros::NodeHandle nh;

    ros::Publisher twist_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, true);
    twist_pub.publish(ts_trans);
    Interface interface;
    ros::spinOnce();

    nuturtlebot::WheelCommandsConstPtr wheel_cmd=ros::topic::waitForMessage<nuturtlebot::WheelCommands>("/wheel_cmd");
    EXPECT_EQ(wheel_cmd->left_velocity, 265);
    EXPECT_EQ(wheel_cmd->right_velocity, 265);
}

/// \brief test rotation of the robot
/// \param TestSuite, testRot
/// \returns EXPECT_EQ
TEST_F(InterfaceFixture, TestRot) {
    ros::NodeHandle nh;

    ros::Publisher twist_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, true);
    twist_pub.publish(ts_rot);
    Interface interface;
    ros::spinOnce();

    nuturtlebot::WheelCommandsConstPtr wheel_cmd=ros::topic::waitForMessage<nuturtlebot::WheelCommands>("/wheel_cmd");
    EXPECT_EQ(wheel_cmd->left_velocity, -101);
    EXPECT_EQ(wheel_cmd->right_velocity, 101);
}

/// \brief test translation and rotation of the robot
/// \param TestSuite, testTransRot
/// \returns EXPECT_EQ
TEST_F(InterfaceFixture, TestTransRot) {
    ros::NodeHandle nh;

    ros::Publisher twist_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, true);
    twist_pub.publish(ts_trans_rot);
    Interface interface;
    ros::spinOnce();

    nuturtlebot::WheelCommandsConstPtr wheel_cmd=ros::topic::waitForMessage<nuturtlebot::WheelCommands>("/wheel_cmd");
    EXPECT_EQ(wheel_cmd->left_velocity, 176);
    EXPECT_EQ(wheel_cmd->right_velocity, 265);
}

/// \brief test sensor data of the robot
/// \param TestSuite, testSensorData
/// \returns EXPECT_EQ
TEST_F(InterfaceFixture, TestSensorData) {
    ros::NodeHandle nh;

    ros::Publisher sd_pub=nh.advertise<nuturtlebot::SensorData>("/sensor_data", 100, true);
    sd_pub.publish(sd);
    Interface interface;
    ros::spinOnce();

    sensor_msgs::JointStateConstPtr joint_state=ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    EXPECT_EQ(joint_state->position[0], 2000);
    EXPECT_EQ(joint_state->position[1], 3000);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");

    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
};

/// \brief constructor function
/// initialize the parameters
Interface::Interface() {
    vel_sub = n.subscribe("/cmd_vel", 100, &Interface::cmdVelCallback,this);
    sensor_sub = n.subscribe("/sensor_data", 100, &Interface::sensorDataCallback,this);
    wheel_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 100, true);
    js_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 100, true);

    params.getParam("encoder_ticks_per_rev",encoder_ticks_per_rev);
    params.getParam("max_trans_robot",max_trans_robot);
    params.getParam("max_rot_robot",max_rot_robot);
    params.getParam("max_rot_motor",max_rot_motor);
};

/// \brief callback function
/// \param  ts - geometry_msgs::Twist
void Interface::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& ts)
{
    /// 1. get twist from input, move the robot
    geometry_msgs::Twist limit_ts=limitInput(ts);
    rigid2d::Twist2D twist;
    twist.w=limit_ts.angular.z;
    twist.vx=limit_ts.linear.x;
    twist.vy=limit_ts.linear.y;
    rigid2d::WheelVelocities wheel_vel=intRobot.twistToWheels(twist);

    /// 2.publish joint states.
    nuturtlebot::WheelCommands limit_wheel_cmd=limitOutput(wheel_vel);
    wheel_pub.publish(limit_wheel_cmd);
};

/// \brief callback function
/// \param  sd - nuturtlebot::SensorData
void Interface::sensorDataCallback(const nuturtlebot::SensorData::ConstPtr& sd){

    /// 1.update velocity
    double time_interval=sd->stamp.toSec()-time_stamp_encoder.toSec();
    double vel_left=(sd->left_encoder-left_encoder)/(encoder_ticks_per_rev*time_interval);//in radians
    double vel_right=(sd->right_encoder-right_encoder)/(encoder_ticks_per_rev*time_interval);

    /// 2.update encoder and time stamp
    left_encoder=sd->left_encoder;
    right_encoder=sd->right_encoder;
    time_stamp_encoder = sd->stamp;

    /// 3.publish joint states.
    js.header.stamp = sd->stamp;
    js.name.push_back("left_wheel_axle");
    js.name.push_back("right_wheel_axle");
    js.position.push_back(left_encoder);
    js.position.push_back(right_encoder);
    js.velocity.push_back(vel_left);
    js.velocity.push_back(vel_right);
    js_pub.publish(js);
}

/// \brief limit input cmd_vel
/// \param  ts - geometry_msgs::Twist
geometry_msgs::Twist Interface::limitInput(const geometry_msgs::Twist::ConstPtr& ts){

    geometry_msgs::Twist new_ts;
    /// 1.translation limit
    if (abs(ts->linear.x)>max_trans_robot){
        new_ts.linear.x=ts->linear.x/abs(ts->linear.x)*max_trans_robot;
    }
    else{
        new_ts.linear.x=ts->linear.x;
    }
    new_ts.linear.y=0;
    new_ts.linear.z=0;

    /// 2.rotation limit
    new_ts.angular.x=0;
    new_ts.angular.y=0;
    if (abs(ts->angular.z)>max_rot_robot){
        new_ts.angular.z=ts->angular.z/abs(ts->angular.z)*max_trans_robot;
    }
    else{
        new_ts.angular.z=ts->angular.z;
    }

    return new_ts;
}

/// \brief limit output wheel velocities
/// \param  wheel_vel - rigid2d::WheelVelocities, wheel velocities
nuturtlebot::WheelCommands Interface::limitOutput(const rigid2d::WheelVelocities wheel_vel){
    nuturtlebot::WheelCommands limit_wheel_vel;

    /// 1.left velocity limit
    if (abs(wheel_vel.left)>max_rot_motor){limit_wheel_vel.left_velocity=wheel_vel.left/abs(wheel_vel.left)*265;}
    else{limit_wheel_vel.left_velocity=wheel_vel.left*265/max_rot_motor;}

    /// 2.right velocity limit
    if (abs(wheel_vel.right)>max_rot_motor){limit_wheel_vel.right_velocity=wheel_vel.right/abs(wheel_vel.right)*265;}
    else{limit_wheel_vel.right_velocity=wheel_vel.right*265/max_rot_motor;}

    return limit_wheel_vel;
}

/// \brief helper function to create a geometry_msg::Vector3
/// \param  vx,wz - double, linear velocity and rotating velocity
geometry_msgs::Twist createTwist(double vx, double wz) {
    geometry_msgs::Twist ts;
    /// 1.translation
    ts.linear.x = vx ;
    ts.linear.y = 0;
    ts.linear.z = 0;

    /// 2.rotation
    ts.angular.x=0;
    ts.angular.y=0;
    ts.angular.z=wz;
    return ts;
};

/// \brief helper function to create a geometry_msg::Vector3
/// \param  vx,wz - double, linear velocity and rotating velocity
nuturtlebot::SensorData createSensorData(int l_encoder, int r_encoder){
    nuturtlebot::SensorData sd;
    sd.stamp=ros::Time::now();
    sd.left_encoder=l_encoder;
    sd.right_encoder=r_encoder;
    return sd;
};
