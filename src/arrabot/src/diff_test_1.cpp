#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 
#include <tf/transform_listener.h> 
#include <laser_geometry/laser_geometry.h> 
#include <math.h>
#include <vector>
#include <iostream>

using std::cout, std::endl;
bool inf_done_at_least_once = false;
bool flag_to_check_door_start = false;
bool flag_to_check_door_stop = false;

sensor_msgs::LaserScan msg_first_lid; // Используется в колбэке роса для получения значений с лидара на прямую
sensor_msgs::LaserScan cut_laser; // Содержит лидарные точки с вырезанным сегментом по произвольным углам


// Колбэк роса для передачи на прямую значений с лидара
void cd(const sensor_msgs::LaserScan& msg) {
   msg_first_lid = msg;
   if (!inf_done_at_least_once) inf_done_at_least_once = true;
}

sensor_msgs::LaserScan cutout_scan_segment(sensor_msgs::LaserScan scan, double min_angle, double max_angle) {
    sensor_msgs::LaserScan segment_scan;
    segment_scan = scan;

    segment_scan.ranges.clear();
    int max_index = int (max_angle / scan.angle_increment);
    int min_index = int (min_angle / scan.angle_increment);

    int c = 0;

    for (int i = 0; i < scan.ranges.size(); ++i) {
        if (i < max_index and i > min_index) {
            segment_scan.ranges.push_back(scan.ranges.at(i));
            c++;
        }
        else {
            segment_scan.ranges.push_back(INFINITY);
        }
    }

    // ROS_INFO_STREAM("effective segment_scan.ranges.size(): " << c);

    return segment_scan;
}

float diff_laser(sensor_msgs::LaserScan scan, int log_i) {
    return (scan.ranges.at(log_i+1) - scan.ranges.at(log_i));
}




////////////////////////////////////////_______////_MAIN_////_______//////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    ros::init(argc, argv, "diff_test_1");
    cut_laser.header.frame_id = "laser";
    //cloud_search_vec.header.frame_id = "laser";
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, cd);
    ros::Publisher cut = n.advertise<sensor_msgs::LaserScan>("cut_laser_pub", 100);
    //ros::Publisher search_sys = n.advertise<sensor_msgs::PointCloud>("search_sys", 100);

    ros::Rate loop_rate(5); 

    while (ros::ok()) {

        int schet_door = 0;
        int right_dot_i_door = 0;
        int left_dot_i_door = 0;

        if (inf_done_at_least_once) {

            cut_laser = cutout_scan_segment(msg_first_lid, 160.0*M_PI/180, 260.0*M_PI/180);

            for (int i = 0; i < cut_laser.ranges.size()-1; i++) {
                if (std::isfinite(cut_laser.ranges.at(i))) {
                    float diff_znch = diff_laser(cut_laser, i);
                    if (diff_znch > 2 && flag_to_check_door_start == false) {
                        flag_to_check_door_start = true;
                        right_dot_i_door = i;
                    }
                    if (diff_znch < -2 && flag_to_check_door_stop == false && flag_to_check_door_start == true) {
                        flag_to_check_door_stop = true;
                        left_dot_i_door = i;
                    }
                }
            }

            if (flag_to_check_door_start == true && flag_to_check_door_stop == true) {
                flag_to_check_door_start = false;
                flag_to_check_door_stop = false;
                schet_door++;
                cout << "KOLICH DVEREI --> " << schet_door << endl;
            }


            cut.publish(cut_laser);
        }


        loop_rate.sleep();
        ros::spinOnce(); 
    }





    return 0;
}