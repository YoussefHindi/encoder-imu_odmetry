#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_grid_publisher");
    ros::NodeHandle nh;
    ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    
    tf2_ros::TransformBroadcaster br;


    // Create an occupancy grid message
    nav_msgs::OccupancyGrid map;
    
    // Set the map properties
    map.header.frame_id = "map"; // Replace with your frame ID
    map.info.resolution = 0.05; // 5 cm per cell
    map.info.width = 40; // 2 meters x 2 meters with 5 cm resolution
    map.info.height = 40;
    
    // Create a map data array and initialize it
    map.data.resize(map.info.width * map.info.height, 0);

    // Set the border of the map to occupied (e.g., value 100)
    for (int i = 0; i < map.info.width; i++) {
        map.data[i] = 100; // Occupied border on the top
        map.data[i + (map.info.height - 1) * map.info.width] = 100; // Occupied border on the bottom
    }
    
    for (int j = 0; j < map.info.height; j++) {
        map.data[j * map.info.width] = 100; // Occupied border on the left
        map.data[j * map.info.width + map.info.width - 1] = 100; // Occupied border on the right
    }
    
    // Create a transformation between "odom" and "map" frames
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map"; // Parent frame
    transformStamped.child_frame_id = "odom";   // Child frame
    transformStamped.transform.translation.x = 1.0;
    transformStamped.transform.translation.y = 1.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
        
    ros::Rate rate(1.0);  // Adjust the publishing rate as needed

    
    // Publish the updated transformation
    while (ros::ok()) {
        transformStamped.header.stamp = ros::Time::now();
        // Broadcast the transformation
        br.sendTransform(transformStamped);
        ros::spinOnce();
        // Publish the map
        map_publisher.publish(map);
        
        rate.sleep();
    }
  
    return 0;
}

