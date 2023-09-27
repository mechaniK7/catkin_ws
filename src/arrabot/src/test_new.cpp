#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 
#include <tf/transform_listener.h> 
#include <laser_geometry/laser_geometry.h> 
#include <math.h>
#include <vector>
#include <iostream>

bool inf_done_at_least_once = false;

sensor_msgs::LaserScan msg_first_lid; // Используется в колбэке роса для получения значений с лидара на прямую
sensor_msgs::PointCloud cloud;
geometry_msgs::Point32 log_point;

// Колбэк роса для передачи на прямую значений с лидара
void cd(const sensor_msgs::LaserScan& msg) {
   msg_first_lid = msg;
   if (!inf_done_at_least_once) inf_done_at_least_once = true;
}

void calc_to_cloud(const sensor_msgs::LaserScan& msg_log, float log_i) { 
    log_point.y = msg_log.ranges[log_i] * sin(msg_log.angle_increment * log_i);
    log_point.x = msg_log.ranges[log_i] * cos(msg_log.angle_increment * log_i);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_new");
    cloud.header.frame_id = "laser";
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, cd);
    ros::Publisher convert = n.advertise<sensor_msgs::PointCloud>("poins_from_laser", 100);

    ros::Rate loop_rate(5);
    
    while (ros::ok()) {

        using std::cout, std::endl;
        cloud.points.clear();

        if (inf_done_at_least_once) {
            cout << msg_first_lid.ranges.size() << endl;
            for (int i = 1; i <= msg_first_lid.ranges.size(); i++) {
                calc_to_cloud(msg_first_lid, i);
                cloud.points.push_back(log_point);
            }
            convert.publish(cloud); 

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}