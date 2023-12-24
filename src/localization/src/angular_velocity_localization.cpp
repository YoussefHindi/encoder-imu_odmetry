#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

// Constants for your robot setup
double wheelRadius = 0.02;           // Wheel radius in meters (1 cm)
int encoderTicksPerRevolution = 1000; // Replace with your actual value

// Function to calculate encoder ticks from angular velocity
int calculateEncoderTicks(double angularVelocity, double dt)
{
    return static_cast<int>((angularVelocity * encoderTicksPerRevolution * dt) / (2.0 * M_PI));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_localization");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    // Constants for your robot setup
    double angularVelocity = 100.0; // Example angular velocity in radians per second
    int ticksPerMeter = 500;

    ros::Rate rate(10);  // Loop rate
    
    ros::Time startTime = ros::Time::now();
    double total_distance = 0;

    while (ros::ok())
    {   
        ros::Time endTime = ros::Time::now();
        ros::Duration duration = endTime - startTime;
        double deltaTime = duration.toSec();  
        
        int encoderTicks = calculateEncoderTicks(angularVelocity, deltaTime);
        
        // Calculate distance traveled in meters
        double distance = static_cast<double>(encoderTicks) / ticksPerMeter;
        
        ROS_INFO("Encoder Ticks: %d, Distance: %.3f meters", encoderTicks, distance);
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";


        odom_msg.pose.pose.position.x = total_distance + distance;
        odom_msg.pose.pose.position.y = 0;
        odom_msg.pose.pose.position.z = 0;

        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 1;

        // Publish the Odometry message
        odom_pub.publish(odom_msg);
        ros::spinOnce();
        rate.sleep();
        
        total_distance = odom_msg.pose.pose.position.x;
        
        startTime = ros::Time::now();

    ROS_INFO("Angular Velocity: %.3f rad/s, Encoder Ticks: %d, Time Interval (Δt): %.6f seconds", angularVelocity, encoderTicks, deltaTime);
    }

    return 0;
}

