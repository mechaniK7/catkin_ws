// Финальный вариант программы для 1 и 2 задания (ровная заливка дверных проёмов с выводом координат точек двери)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <drone_msgs/Door.h>
#include <math.h>
#include <vector>
#include <iostream>
#include "euler_angles.hpp"

using std::cout, std::endl;
bool inf_done_at_least_once = false; // Ожидание поступления значений лазер скана
bool flag_to_check_frame_right = false; // Флаг для определения правого края двери
bool flag_to_check_flame_left = false; // Флаг для определения левого края двери

EulerAngles curr_yaw; // Текущий курс
sensor_msgs::PointCloud cloud; // Содержит координаты: indx - 0 - Левый край двери; indx - 1 - Правый край двери; indx - 2 - Центр двери
sensor_msgs::LaserScan msg_first_lid; // Используется в колбэке роса для получения значений с лидара на прямую
sensor_msgs::LaserScan cut_laser; // Содержит лидарные точки с вырезанным сегментом по произвольным углам
drone_msgs::Door frame_info; // Содержит информацию о позициях и координатах при видимости рамки
geometry_msgs::PoseStamped curr_position; // Координаты текущей позиции


// Перевод лазера в облако точек
geometry_msgs::Point32 calc_to_cloud(const sensor_msgs::LaserScan& msg_log, float log_i) { 
    geometry_msgs::Point32 log_point;

    log_point.y = - msg_log.ranges[log_i] * sin(msg_log.angle_increment * log_i);
    log_point.x = - msg_log.ranges[log_i] * cos(msg_log.angle_increment * log_i);

    return log_point;
}

// Колбэк роса для передачи на прямую значений с лидара
void cd(const sensor_msgs::LaserScan& msg) {
   msg_first_lid = msg;

   if (!inf_done_at_least_once) inf_done_at_least_once = true;
}

// Колбэк роса для передачи текущей позиции дрона в пространстве
void pose_cd(const geometry_msgs::PoseStamped& msg) {
    curr_position = msg;
}

// Расчёт длины вектора
float math_vec(const sensor_msgs::PointCloud& cloud_log, int log1, int log2) { 
    return sqrt( pow(cloud_log.points.at(log1).x - cloud_log.points.at(log2).x, 2.0) + pow(cloud_log.points.at(log1).y - cloud_log.points.at(log2).y, 2.0) );
}

// Перевод из облака точек в лазер скан
float calc_to_laser(geometry_msgs::Point32 points_log) { 
    float length;

    length = sqrt(pow(points_log.x, 2) + pow(points_log.y, 2));

    return length;
}


// Расчёт координаты точки в середине точек взятых произвольно
geometry_msgs::Point32 calc_mid_by_2p(const sensor_msgs::PointCloud& cloud_log, int log_1_point, int log_2_point) {
    geometry_msgs::Point32 coord_pt;
    
    coord_pt.x = (cloud_log.points.at(log_1_point).x + cloud_log.points.at(log_2_point).x) / 2;
    coord_pt.y = (cloud_log.points.at(log_1_point).y + cloud_log.points.at(log_2_point).y) / 2;

    return coord_pt;
}

// Отделение произвольного сегмента от всех значений лазер скана
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
            //cout << i << " - dl_cut -- > " << scan.ranges.at(i) << endl;
        }
        else {
            segment_scan.ranges.push_back(INFINITY);
        }
    }

    // ROS_INFO_STREAM("effective segment_scan.ranges.size(): " << c);

    return segment_scan;
}

// Дифференцирование значений лазер скана
float diff_laser(sensor_msgs::LaserScan scan, int log_i) {
    return (scan.ranges.at(log_i+1) - scan.ranges.at(log_i));
}

// Инвертированное дифференцирование значений лазер скана
float diff_laser_inv(sensor_msgs::LaserScan scan, int log_i) {
    return (scan.ranges.at(log_i-1) - scan.ranges.at(log_i));
}



////////////////////////////////////////_______////_MAIN_////_______//////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

    ros::init(argc, argv, "test_2");

    cut_laser.header.frame_id = "laser";
    cloud.header.frame_id = "laser";
    frame_info.header.frame_id = "laser";


    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, cd); // Подписчик на лазер скан
    ros::Subscriber sub2 = n.subscribe("/mavros/local_position/pose", 1000, pose_cd);
    ros::Publisher cut = n.advertise<sensor_msgs::LaserScan>("cut_laser_pub", 100); // Сегмент лазер скана
    ros::Publisher frame_point = n.advertise<sensor_msgs::PointCloud>("frame_lrm_points", 100); // Координаты точек рамки
    ros::Publisher detected_frames = n.advertise<drone_msgs::Door>("detected_frames", 100); 

    ros::Rate loop_rate(5); 

    while (ros::ok()) {

        frame_info.header.stamp = ros::Time::now();
        int schet_frame = 0;
        int right_dot_i_frame = 0;
        int left_dot_i_frame = 0;
        cloud.points.clear();
        cout << endl;

        if (inf_done_at_least_once) {
            
            // Вырезаем сегмент лазер скана и сохраняем
            cut_laser = cutout_scan_segment(msg_first_lid, 100.0*M_PI/180, 260.0*M_PI/180);

            // Поиск правого края рамки
            for (int i = 0; i < cut_laser.ranges.size()-1; i++) {
                if (std::isfinite(cut_laser.ranges.at(i))) {
                    float diff_znch = diff_laser(cut_laser, i);
                    if (diff_znch > 1 && flag_to_check_frame_right == false && cut_laser.ranges.at(i) <= 2.5) {
                        flag_to_check_frame_right = true;
                        right_dot_i_frame = i;
                    }
                }
            }

            // Поиск левого края рамки
            for (int i = cut_laser.ranges.size()-1; i > 1; i--) {
                if (std::isfinite(cut_laser.ranges.at(i))) {
                    float diff_znch_inv = diff_laser_inv(cut_laser, i);
                    if (diff_znch_inv > 1 && flag_to_check_flame_left == false && flag_to_check_frame_right == true && cut_laser.ranges.at(i) <= 2.5) {
                        flag_to_check_flame_left = true;
                        left_dot_i_frame = i;
                    }
                }
            }

            // Обработка рамки
            if (flag_to_check_frame_right == true && flag_to_check_flame_left == true) {
                geometry_msgs::Point32 func_point; // Контейнер для координат точек рамки
                bool mid_pt_flag = false;

                flag_to_check_frame_right = false;
                flag_to_check_flame_left = false;

                schet_frame++;
                cout << "KOLICH RAMOK --> " << schet_frame << endl;

                // Левый край рамки
                func_point = calc_to_cloud(cut_laser, left_dot_i_frame);
                cloud.points.push_back(func_point);
                cout << "Levo i --> " << left_dot_i_frame << endl;
                cout << "LEVO_TOCH - x -- > " << func_point.x << endl;
                cout << "LEVO_TOCH - y -- > " << func_point.y << endl;

                // Правый край рамки
                func_point = calc_to_cloud(cut_laser, right_dot_i_frame);
                cloud.points.push_back(func_point);
                cout << "Pravo i --> " << right_dot_i_frame << endl;
                cout << "PRAVO_TOCH - x -- > " << func_point.x << endl;
                cout << "PRAVO_TOCH - y -- > " << func_point.y << endl;

                if (math_vec(cloud, 0, 1) >= 0.4 && math_vec(cloud, 0, 1) <= 0.85) { cout << "STOLB --> " << math_vec(cloud, 0, 1) << endl; mid_pt_flag = false; }
                if (math_vec(cloud, 0, 1) <= 0.4) { cout << "STOLB RAMKI --> " << math_vec(cloud, 0, 1) << endl; mid_pt_flag = false; }
                if (math_vec(cloud, 0, 1) >= 0.85) { cout << "RAMKA --> " << math_vec(cloud, 0, 1) << endl; mid_pt_flag = true; }

                if (mid_pt_flag == true) {
                    // Центр рамки
                    func_point = calc_mid_by_2p(cloud, 0, 1);
                    cloud.points.push_back(func_point);

                    // Сохраняем координаты центра рамки в контейнер
                    frame_info.position.x = func_point.x;
                    frame_info.position.y = func_point.y;

                    cout << "MID_TOCH - x -- > " << func_point.x << endl;
                    cout << "MID_TOCH - y -- > " << func_point.y << endl;

                    // Определяем текущий курс
                    curr_yaw.get_RPY_from_msg_quaternion(curr_position.pose.orientation);
                    cout << "KURSE --> " << curr_yaw.yaw() << endl;

                    // Сохраняем текущий курс в контейнер
                    frame_info.course = curr_yaw.yaw();

                    // Сохраняем текущую позицию дрона в пространстве в контейнер
                    frame_info.detection_position.x = curr_position.pose.position.x;
                    frame_info.detection_position.y = curr_position.pose.position.y;
                    frame_info.detection_position.z = curr_position.pose.position.z;

                }

            }

            detected_frames.publish(frame_info); // Отправляем контейнер в топик
            frame_point.publish(cloud);
            cut.publish(cut_laser);
        }
        loop_rate.sleep();
        ros::spinOnce(); 
    }
    return 0;
}