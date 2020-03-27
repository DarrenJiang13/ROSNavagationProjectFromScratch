#include <ros/ros.h>
#include "nuturtle_gazebo/turtle_drive_plugin.hpp"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    DiffRobotGazebo::DiffRobotGazebo(){
    }

    void DiffRobotGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

        /// 0.Initialization
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL("A ROS node for Gazebo has not been initialized."
                      "Unable to load plugin. Load the Gazebo system plugin"
                      "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
            return;
        }

        // Store the pointer to the model
        this->model = _model;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DiffRobotGazebo::OnUpdate, this));

        /// 1. Parameters
        left_wheel_joint=_sdf->Get<std::string>("left_wheel_joint");
        right_wheel_joint=_sdf->Get<std::string>("right_wheel_joint");
        wheel_cmd_topic=_sdf->Get<std::string>("wheel_cmd_topic");
        sensor_data_topic=_sdf->Get<std::string>("sensor_data_topic");
        sensor_frequency=_sdf->Get<double>("sensor_frequency");
        encoder_ticks_per_rev=_sdf->Get<double>("encoder_ticks_per_rev");
        max_trans_robot=_sdf->Get<double>("max_trans_robot");
        max_rot_robot=_sdf->Get<double>("max_rot_robot");
        max_rot_motor=_sdf->Get<double>("max_rot_motor");
        //ROS_INFO("left_wheel_joint: %s. right_wheel_joint: %s wheel_cmd_topic: %s sensor_data_topic: %s",left_wheel_joint.c_str(),right_wheel_joint.c_str(),wheel_cmd_topic.c_str(),sensor_data_topic.c_str());
        //ROS_INFO("sensor_frequency: %f. encoder_ticks_per_rev: %f max_trans_robot: %f max_rot_robot: %f",sensor_frequency,encoder_ticks_per_rev,max_trans_robot,max_rot_robot);
        /// 2.topics
        wheel_sub = n.subscribe(wheel_cmd_topic,200,&DiffRobotGazebo::wheelCmdCallback, this);
        sensor_pub = n.advertise<nuturtlebot::SensorData>(sensor_data_topic,200,true);

        /// 3.joints
        this->joints.resize(2);

        this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("left_wheel_joint"));
        if (!(this->joints[0]))
        {
            gzerr <<"Unable to find joint: left_wheel_joint.\n";
            return;
        }

        this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("right_wheel_joint"));
        if (!this->joints[1])
        {
            gzerr <<"Unable to find joint: right_wheel_joint.\n";
            return;
        }

        ROS_INFO("turtle_drive_plugin loads successfully. ");
    }

    void DiffRobotGazebo::OnUpdate(){
        // Apply a small linear velocity to the model.
        //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));


        /// 1.send velocity to wheels in gazebo
        this->joints[0]->SetParam("fmax", 0, 10000.0);
        this->joints[0]->SetParam("vel", 0, wheel_left);

        this->joints[1]->SetParam("fmax", 0, 10000.0);
        this->joints[1]->SetParam("vel", 0, wheel_right);

        /// 2.publish wheel odometry sensor_data
//        this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0, 1000.0);
//        this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0, 1.0);
//
//        this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0, 1000.0);
//        this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0, 1.0);
        sensor_data.stamp=ros::Time::now();
        sensor_data.left_encoder=this->joints[0]->Position()*encoder_ticks_per_rev/(2.0*PI);
        sensor_data.right_encoder=this->joints[1]->Position()*encoder_ticks_per_rev/(2.0*PI);
        sensor_pub.publish(sensor_data);
    }

    void DiffRobotGazebo::wheelCmdCallback(const nuturtlebot::WheelCommands::ConstPtr& wc)
    {
        wheel_left=wc->left_velocity*1.0*max_rot_motor/265.0;
        wheel_right=wc->right_velocity*1.0*max_rot_motor/265.0;
    }
}