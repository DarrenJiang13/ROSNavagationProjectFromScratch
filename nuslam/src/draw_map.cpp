/// \file: draw_map.cpp
/// \brief: create a ros node caller draw_map
///
/// PARAMETERS:
///    base_frame_id: base frame id of landmarks
/// PUBLISHES:
///     /visualization_marker (visualization_msgs::MarkerArray): publish array of landmarks with respect to base_link frame
/// SUBSCRIBES:
///     /landmarks (nuslam::TurtleMap): publish the ground truth landmarks data in gazebo(added with some noise)

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include "nuslam/TurtleMap.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

const double PI=3.1415926535;
std::string base_frame;

ros::Publisher landmarks_pub;
ros::Subscriber landmarks_sub;
void landmarkSubCallback(const nuslam::TurtleMap::ConstPtr &tm);

int main(int argc, char **argv) {
    ros::init(argc, argv, "draw_map");
    ros::NodeHandle n;

    // 0. set base_frame_id from landmarks
    // when get TurtleMap from landmark node,set base_frame_id = base_scan in .launch file
    // else when from slam node, set base_fram_id = map;
    ros::NodeHandle get_params("~");
    get_params.getParam("base_frame_id",base_frame);

    /// 1. subscriber and publisher
    landmarks_pub=n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1,true);
    landmarks_sub=n.subscribe("/landmarks", 100, landmarkSubCallback);


    ros::spin();
    return 0;
};

/// \brief callback function
/// \param  ls - nuslam::TurtleMap
void landmarkSubCallback(const nuslam::TurtleMap::ConstPtr &tm)
{
    std::vector<double>center_x=tm->center_x;
    std::vector<double>center_y=tm->center_y;
    std::vector<double>radius=tm->radius;

    visualization_msgs::MarkerArray marker_array;

    ///marker
    for(unsigned i=0;i < center_x.size();i++){
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::CYLINDER;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = base_frame;
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
        marker.pose.position.x = center_x[i];
        marker.pose.position.y = center_y[i];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.06;
        marker.scale.y = 0.06;
        marker.scale.z = 0.6;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(marker);
    }

    landmarks_pub.publish(marker_array);
}