#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

int getEncoderTicks(int ticks)
{
    ticks = ticks + 5;
    return ticks; // Example value
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_odom_publisher");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    // Constants for your robot setup
    double wheelDiameter = 0.02;  // 2 cm in meters
    int ticksPerMeter = 5000;     // Replace with your actual value
    
    ros::Rate rate(10);  // Publish data at 10 Hz
    
    int ticks = 0;

    while (ros::ok())
    {
        
        ticks = getEncoderTicks(ticks);
        
        // Calculate distance traveled in meters
        double distance = static_cast<double>(ticks) / ticksPerMeter;
        
        ROS_INFO("Encoder Ticks: %d, Distance: %.3f meters", ticks, distance);
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";


        //odom_msg.pose.pose.position.x = 0;
        //odom_msg.pose.pose.position.y = 0;
        //odom_msg.pose.pose.position.z = 0;
        odom_msg.twist.twist.linear.x = 0.1;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;

        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 1;

        // Publish the Odometry message
        odom_pub.publish(odom_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

