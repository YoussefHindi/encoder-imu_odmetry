// This script publishes readings to the robot links and frames and creates robot's odometry

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    // Set the frame IDs
    transformStamped.header.frame_id = "odom";  // Parent frame
    transformStamped.child_frame_id = "base_footprint";  // Child frame
    
    // Process the received Odometry message
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation = msg->pose.pose.orientation;
    
    transformStamped.header.stamp = ros::Time::now();
    
    br.sendTransform(transformStamped);
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "odometry_subscriber");
    ros::NodeHandle nh_sub;
    
    // TF Buffer Broadcaster
    tf2_ros::TransformBroadcaster br;
    
    ros::Subscriber odom_sub = nh_sub.subscribe("odom", 10, odomCallback);
    
    ros::spin();  


    return 0;
}
