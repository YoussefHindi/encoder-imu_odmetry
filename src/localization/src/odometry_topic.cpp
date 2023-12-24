#include "ros/ros.h"
#include "nav_msgs/Odometry.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_odom_publisher");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Rate rate(10);  // Publish data at 10 Hz
    

    while (ros::ok())
    {   
        nav_msgs::Odometry odom_msg;
        //odom_msg.header.stamp = ros::Time::now();
        //odom_msg.header.frame_id = "odom";
        //odom_msg.child_frame_id = "base_footprint";

        // Publish the Odometry message
        odom_pub.publish(odom_msg);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

