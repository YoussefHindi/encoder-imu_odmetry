#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ImuOdometrySimulator {
public:
    ImuOdometrySimulator() : linear_velocity_x(0.1), current_yaw(0.0) {
        imu_sub = nh.subscribe("imu", 10, &ImuOdometrySimulator::imuCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        double dt = (imu_msg->header.stamp - previous_stamp).toSec();

        // Integrate linear acceleration for velocity
        linear_velocity_x += imu_msg->linear_acceleration.x * dt;

        // Integrate angular velocity for orientation change
        double delta_yaw = imu_msg->angular_velocity.z * dt;
        current_yaw += delta_yaw;

        // Update pose based on velocity and orientation change
        double delta_x = linear_velocity_x * cos(current_yaw) * dt;
        double delta_y = linear_velocity_x * sin(current_yaw) * dt;

        robot_x += delta_x;
        robot_y += delta_y;

        // Publish odometry message
        publishOdometry(imu_msg->header.stamp);
    }

    void publishOdometry(const ros::Time& stamp) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        // Set the position
        odom_msg.pose.pose.position.x = robot_x;
        odom_msg.pose.pose.position.y = robot_y;
        odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, current_yaw));

        // Set the velocity
        odom_msg.twist.twist.linear.x = linear_velocity_x;

        odom_pub.publish(odom_msg);

        previous_stamp = stamp;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Publisher odom_pub;

    double linear_velocity_x;
    double current_yaw;
    double robot_x;
    double robot_y;
    ros::Time previous_stamp;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_odometry_simulator");
    ImuOdometrySimulator imu_odometry_simulator;
    ros::spin();
    return 0;
}

