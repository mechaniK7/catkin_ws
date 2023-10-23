#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <drone_msgs/Door.h>
#include <drone_msgs/Mission.h>
#include <math.h>
#include <vector>
#include <iostream>
#include "euler_angles.hpp"

using std::cout, std::endl;

bool mission_flag;
bool my_mission_stat = true;
drone_msgs::Door frame_info; // Содержит информацию о позициях и координатах при видимости рамки

// Колбэк роса для получения информации о позициях и координатах рамки
void cd(const drone_msgs::Door& msg) {
    frame_info = msg;
}

// Колбэк роса для получения информации о состоянии миссии
void cd_status(const std_msgs::Bool& msg) {
    mission_flag = msg.data;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "rotate_around_pillar");
    frame_info.header.frame_id = "laser";

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("detected_frames", 100, cd); // Данные по рамке
    ros::Subscriber sub2 = n.subscribe("autopilot/mission_status", 100, cd_status); // Данные по рамке
    ros::Publisher autopilot_mission = n.advertise<drone_msgs::Mission>("/autopilot/mission", 100); // Миссия

    ros::Rate loop_rate(5);

    while (ros::ok()) {


        if (frame_info.is_pillar.data == true && my_mission_stat == true) {

            float old_ang = frame_info.local_angle;

            drone_msgs::Mission mission;
            drone_msgs::MissionSegment segment;

            drone_msgs::Goal new_goal;
            new_goal.completion_policy = drone_msgs::Goal::POLICY_DISTANCE_REACHED;
            new_goal.policy_distance_threshold = 0.2;
            new_goal.pose.coordinates_type = drone_msgs::DronePose::LOCAL_XYC;

            new_goal.pose.point.x = 1;
            new_goal.pose.point.y = 2;
            new_goal.pose.point.z = 1;
            segment.trajectory.waypoints.push_back(new_goal);

            new_goal.pose.point.x = 1.5;
            new_goal.pose.point.y = 0;
            new_goal.pose.point.z = 1;
            segment.trajectory.waypoints.push_back(new_goal);

            new_goal.pose.point.x = 1;
            new_goal.pose.point.y = -2;
            new_goal.pose.point.z = 1;
            segment.trajectory.waypoints.push_back(new_goal);

            segment.completion_condition = drone_msgs::MissionSegment::LAST_POINT_ARRIVAL;
            segment.mode.mode = drone_msgs::FlightControlMode::GO_ALONG_TRAJECTORY;
            mission.segments.push_back(segment);

            autopilot_mission.publish(mission);

            my_mission_stat = false;
            /*float curr_ang = frame_info.local_angle;
            float err = curr_ang - old_ang;
            float p_koef = err * 0.5;
    
            new_goal.pose.course = p_koef;
            segment.trajectory.waypoints.push_back(new_goal);
            mission.segments.push_back(segment);
            autopilot_mission.publish(mission);*/

        }
        
        if (mission_flag == true) my_mission_stat = true;


        loop_rate.sleep();
        ros::spinOnce(); 
    }
    return 0;
}