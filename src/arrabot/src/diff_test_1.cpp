#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h> 
#include <math.h>
#include <vector>
#include <iostream>

using std::cout, std::endl;
bool inf_done_at_least_once = false; // Ожидание поступления значений лазер скана
bool flag_to_check_door_right = false; // Флаг для определения правого края двери
bool flag_to_check_door_left = false; // Флаг для определения левого края двери
float koef_n; // Степень геометрической прогрессии
float koef_door; // Коэффициент геометрической прогресии

sensor_msgs::PointCloud cloud; // Содержит координаты: indx - 0 - Левый край двери; indx - 1 - Правый край двери; indx - 2 - Центр двери
sensor_msgs::LaserScan msg_first_lid; // Используется в колбэке роса для получения значений с лидара на прямую
sensor_msgs::LaserScan cut_laser; // Содержит лидарные точки с вырезанным сегментом по произвольным углам
sensor_msgs::LaserScan updated_scan; // Содержит весь лазер скан с заполненными дверьми 


// Перевод лазера в облако точек
geometry_msgs::Point32 calc_to_cloud(const sensor_msgs::LaserScan& msg_log, float log_i) { 
    geometry_msgs::Point32 log_point;

    log_point.y = - msg_log.ranges[log_i] * sin(msg_log.angle_increment * log_i);
    log_point.x = - msg_log.ranges[log_i] * cos(msg_log.angle_increment * log_i);

    return log_point;
}

// Перевод из облака точек в лазер скан
float calc_to_laser(geometry_msgs::Point32 points_log) { 
    float length;

    length = sqrt(pow(points_log.x, 2) + pow(points_log.y, 2));

    return length;
}

// Колбэк роса для передачи на прямую значений с лидара
void cd(const sensor_msgs::LaserScan& msg) {
   msg_first_lid = msg;

   if (!inf_done_at_least_once) inf_done_at_least_once = true;
}

// Заполнение пустот в двери
float get_door_point_range(sensor_msgs::LaserScan scan, int current_indx_laser, int l_door_indx, int r_door_indx) {
    geometry_msgs::Point32 log_coords_final;
    
    geometry_msgs::Point32 func_coords_l_pt1 = calc_to_cloud(scan, l_door_indx);
    geometry_msgs::Point32 func_coords_r_pt2 = calc_to_cloud(scan, r_door_indx);

    double current_angle = scan.angle_increment * current_indx_laser;

    log_coords_final.x = - (func_coords_l_pt1.x * func_coords_r_pt2.y - func_coords_r_pt2.x * func_coords_l_pt1.y) * cos(current_angle) / (cos(current_angle) * func_coords_l_pt1.y - cos(current_angle) * func_coords_r_pt2.y - sin(current_angle) * func_coords_l_pt1.x + sin(current_angle) * func_coords_r_pt2.x);
    log_coords_final.y = - (func_coords_l_pt1.x * func_coords_r_pt2.y - func_coords_r_pt2.x * func_coords_l_pt1.y) * sin(current_angle) / (cos(current_angle) * func_coords_l_pt1.y - cos(current_angle) * func_coords_r_pt2.y - sin(current_angle) * func_coords_l_pt1.x + sin(current_angle) * func_coords_r_pt2.x);

    cout << "X -- > " << log_coords_final.x << endl;
    cout << "Y -- > " << log_coords_final.y << endl;

    float length_laser = calc_to_laser(log_coords_final);
    cout << "DLOINA -- > " << length_laser << endl;

    return length_laser;
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

    ros::init(argc, argv, "diff_test_1");

    cut_laser.header.frame_id = "laser";
    cloud.header.frame_id = "laser";

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, cd); // Подписчик на лазер скан
    ros::Publisher cut = n.advertise<sensor_msgs::LaserScan>("cut_laser_pub", 100); // Сегмент лазер скана
    ros::Publisher fill_scan = n.advertise<sensor_msgs::LaserScan>("filling_scan", 100); // Лазер скан с зарисованной дверью
    ros::Publisher door_point = n.advertise<sensor_msgs::PointCloud>("door_lrm_points", 100); // Координаты точек двери

    ros::Rate loop_rate(5); 

    while (ros::ok()) {

        int schet_door = 0;
        int right_dot_i_door = 0;
        int left_dot_i_door = 0;
        cloud.points.clear();
        cout << endl;

        if (inf_done_at_least_once) {

            float ko = 0; // Геометрическая прогрессия
            
            // Вырезаем сегмент лазер скана и сохраняем
            cut_laser = cutout_scan_segment(msg_first_lid, 100.0*M_PI/180, 260.0*M_PI/180);

            // Поиск правого края двери
            for (int i = 0; i < cut_laser.ranges.size()-1; i++) {
                if (std::isfinite(cut_laser.ranges.at(i))) {
                    float diff_znch = diff_laser(cut_laser, i);
                    if (diff_znch > 1 && flag_to_check_door_right == false) {
                        flag_to_check_door_right = true;
                        right_dot_i_door = i;
                    }
                }
            }

            // Поиск левого края двери
            for (int i = cut_laser.ranges.size()-1; i > 1; i--) {
                if (std::isfinite(cut_laser.ranges.at(i))) {
                    float diff_znch_inv = diff_laser_inv(cut_laser, i);
                    if (diff_znch_inv > 1 && flag_to_check_door_left == false && flag_to_check_door_right == true) {
                        flag_to_check_door_left = true;
                        left_dot_i_door = i;
                    }
                }
            }

            // Обработка двери
            if (flag_to_check_door_right == true && flag_to_check_door_left == true) {
                geometry_msgs::Point32 func_point; // Контейнер для координат точек двери

                flag_to_check_door_right = false;
                flag_to_check_door_left = false;

                schet_door++;
                cout << "KOLICH DVEREI --> " << schet_door << endl;

                // Левый край двери
                func_point = calc_to_cloud(cut_laser, left_dot_i_door);
                cloud.points.push_back(func_point);
                cout << "Levo i --> " << left_dot_i_door << endl;
                cout << "LEVO_TOCH - x -- > " << func_point.x << endl;
                cout << "LEVO_TOCH - y -- > " << func_point.y << endl;

                // Правый край двери
                func_point = calc_to_cloud(cut_laser, right_dot_i_door);
                cloud.points.push_back(func_point);
                cout << "Pravo i --> " << right_dot_i_door << endl;
                cout << "PRAVO_TOCH - x -- > " << func_point.x << endl;
                cout << "PRAVO_TOCH - y -- > " << func_point.y << endl;

                // Центр двери
                func_point = calc_mid_by_2p(cloud, 0, 1);
                cloud.points.push_back(func_point);
                cout << "MID_TOCH - x -- > " << func_point.x << endl;
                cout << "MID_TOCH - y -- > " << func_point.y << endl;

                // // Считаем все коеффициенты по формулам
                // ko = updated_scan.ranges.at(right_dot_i_door);
                // koef_n = left_dot_i_door+1 - right_dot_i_door - 1;
                // koef_door = pow(updated_scan.ranges.at(left_dot_i_door+1) / updated_scan.ranges.at(right_dot_i_door), 1 / (koef_n));
                // cout << right_dot_i_door <<" - dlin --> " << updated_scan.ranges.at(right_dot_i_door) << endl;
                // cout << left_dot_i_door <<" - dlin --> " << updated_scan.ranges.at(left_dot_i_door) << endl;

                // // Зарисовываем дверь
                // for (int i = right_dot_i_door; i <= left_dot_i_door + 1; i++) {
                //     ko *= koef_door;
                //     cout << i <<" - ko --> " << ko << endl;
                //     updated_scan.ranges[i] = ko;
                // }


                updated_scan = msg_first_lid; // Копия всего лазер скана

                // Заполнение двери
                for (int i = right_dot_i_door; i <= left_dot_i_door; i++) {
                    updated_scan.ranges.at(i) = get_door_point_range(msg_first_lid, i, left_dot_i_door, right_dot_i_door);
                }
                ROS_INFO_STREAM(right_dot_i_door << left_dot_i_door);

            }
            fill_scan.publish(updated_scan);
            door_point.publish(cloud);
            cut.publish(cut_laser);
        }
        loop_rate.sleep();
        ros::spinOnce(); 
    }
    return 0;
}