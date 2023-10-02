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
std::vector <int> vec_for_index(9, 1); // Размер вектора строго больше/равен - 9
bool inf_done_at_least_once = false;
int i_global_for_vec = 1;
int iteration = 0;

sensor_msgs::LaserScan msg_first_lid; // Используется в колбэке роса для получения значений с лидара на прямую
sensor_msgs::PointCloud cloud; // Хранит полное облако точек полученное с лидара
sensor_msgs::PointCloud cloud_search_vec;
geometry_msgs::Point32 log_point;
geometry_msgs::Point32 mid_from_vec_by_2p_FIRST;
geometry_msgs::Point32 mid_from_vec_by_2p_SECOND;

// Колбэк роса для передачи на прямую значений с лидара
void cd(const sensor_msgs::LaserScan& msg) {
   msg_first_lid = msg;
   if (!inf_done_at_least_once) inf_done_at_least_once = true;
}

// Перевод лазера в облако точек
void calc_to_cloud(const sensor_msgs::LaserScan& msg_log, float log_i) { 
    log_point.y = msg_log.ranges[log_i] * sin(msg_log.angle_increment * log_i);
    log_point.x = msg_log.ranges[log_i] * cos(msg_log.angle_increment * log_i);
}

// Расчёт углового коэффикиента по координатам двух точек
float calc_k_koef(const sensor_msgs::PointCloud& cloud_log, int log1, int log2) {
    return ( (cloud_log.points.at(log2).y - cloud_log.points.at(log1).y) / (cloud_log.points.at(log2).x - cloud_log.points.at(log1).x) );
}

// Расчёт координаты точки в середине точек взятых из поискового вектора
void calc_mid_from_vec_by_2p(const sensor_msgs::PointCloud& cloud_log, int log_1_point, int log_2_point, int log_3_point, int log_4_point) {
    mid_from_vec_by_2p_FIRST.x = (cloud_log.points.at(log_1_point).x + cloud_log.points.at(log_2_point).x) / 2;
    mid_from_vec_by_2p_FIRST.y = (cloud_log.points.at(log_1_point).y + cloud_log.points.at(log_2_point).y) / 2;
    mid_from_vec_by_2p_SECOND.x = (cloud_log.points.at(log_3_point).x + cloud_log.points.at(log_4_point).x) / 2;
    mid_from_vec_by_2p_SECOND.y = (cloud_log.points.at(log_3_point).y + cloud_log.points.at(log_4_point).y) / 2;
}

float calc_k_from_vec_points() {
    return ( (mid_from_vec_by_2p_SECOND.y - mid_from_vec_by_2p_FIRST.y) / (mid_from_vec_by_2p_SECOND.x - mid_from_vec_by_2p_FIRST.x) );
}

// Сдвиг индексов лучей в поисковм векторе
void shift_vector(int log1) {
    for (int i = 0; i < vec_for_index.size() - 1; i++) {
        vec_for_index.at(i) = vec_for_index.at(i+1); 
    }
    vec_for_index.at(vec_for_index.size() - 1) = log1;
    /*for (int i = 0; i < vec_for_index.size(); i++) { // Вывод данных из вектора
        cout << vec_for_index[i] << ", ";
    }
    cout << endl;*/
}




////////////////////////////////////////_______////_MAIN_////_______//////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_new");
    cloud.header.frame_id = "laser";
    cloud_search_vec.header.frame_id = "laser";
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, cd);
    ros::Publisher convert = n.advertise<sensor_msgs::PointCloud>("poins_from_laser", 100);
    ros::Publisher search_sys = n.advertise<sensor_msgs::PointCloud>("search_sys", 100);

    ros::Rate loop_rate(5);
    
    while (ros::ok()) {

        cloud.points.clear();
        cloud_search_vec.points.clear();

        if (inf_done_at_least_once) {
            
            for (int i = 0; i < msg_first_lid.ranges.size(); i++) {
                calc_to_cloud(msg_first_lid, i);
                cloud.points.push_back(log_point);
            }
            
            shift_vector(i_global_for_vec);
            for (int i = 0; i < vec_for_index.size(); i++) {
                geometry_msgs::Point32 log_search_point = cloud.points.at(vec_for_index[i]);
                cloud_search_vec.points.push_back(log_search_point);
            }

            if (i_global_for_vec < cloud.points.size() - 1) i_global_for_vec++; else { i_global_for_vec = 1; cout << "OK  - 360" << endl; }

            calc_mid_from_vec_by_2p(cloud, 0, 3, vec_for_index.size()-4, vec_for_index.size()-1);




            convert.publish(cloud); 
            search_sys.publish(cloud_search_vec); 
        }






        if (iteration % 100 == 0) { // Было 100
            loop_rate.sleep();
            ros::spinOnce(); 
            iteration = 0;
        } 
        iteration++;
    }
    return 0;
}