#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h> 
#include <laser_geometry/laser_geometry.h> 
#include <math.h>
#include <vector>
#include <iostream>

using std::cout, std::endl;
ros::Publisher pub;
geometry_msgs::Point xyu;

void subs(const sensor_msgs::PointCloud& cloud) {
   
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.ns = "huy";
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    marker.points.push_back(point);
    point.x = cloud.points.at(2).x;
    point.y = cloud.points.at(2).y;
    marker.points.push_back(point); 
    pub.publish(marker);
}
   


void check() {
    ros::NodeHandle n;
    
    pub = n.advertise<visualization_msgs::Marker>("Markers", 1);
    ros::Subscriber sub = n.subscribe("Points_for_drone", 1000, subs);
    
    ros::Rate loop_rate(5);
    
    while (ros::ok()) {
       
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sort");
   
    check();
    return 0;
}
