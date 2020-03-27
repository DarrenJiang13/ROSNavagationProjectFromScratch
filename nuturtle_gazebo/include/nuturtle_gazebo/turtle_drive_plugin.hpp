//Gazebo
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include <string>
#include <vector>

namespace gazebo
{
    class DiffRobotGazebo : public ModelPlugin
    {

    public:
        DiffRobotGazebo();
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:
        physics::ModelPtr model; // Pointer to the model
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        std::vector<physics::JointPtr> joints;
        std::string left_wheel_joint,right_wheel_joint;
        std::string wheel_cmd_topic,sensor_data_topic;

        const double PI=3.1415926535;
        double sensor_frequency;
        double encoder_ticks_per_rev;
        double max_trans_robot,max_rot_robot,max_rot_motor;
        double wheel_left,wheel_right;
        double encoder_left,encoder_right;

        ///node handles
        ros::NodeHandle n;
        ros::Subscriber wheel_sub;
        ros::Publisher sensor_pub;
        nuturtlebot::SensorData sensor_data;
        void wheelCmdCallback(const nuturtlebot::WheelCommands::ConstPtr& wc);
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DiffRobotGazebo)
}
