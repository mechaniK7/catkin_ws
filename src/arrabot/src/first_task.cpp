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

float l_dr; // Дистанция до левого угла двери
float r_dr; // Дистанция до правого угла двери
float l_dr_i; // Индекс луча направленного на левый угол двери
float r_dr_i; // Индекс луча направленного на правый угол двери
float koef_door; // Коэффициент геометрической прогрессии
float koef_n; // Степень геометрической прогрессии
float dst_up; // Переменная для расчёта точек облёта
float k; // Коэффициент для расчёта облака точек

sensor_msgs::LaserScan msg_first_lid; // Используется в колбэке роса для получения значений с лидара на прямую
sensor_msgs::LaserScan msg_new; // Хранятся данные зарисовки двери
sensor_msgs::PointCloud cloud; // Облако точек. Содержит центры дверей
sensor_msgs::PointCloud cloud2; // Облако точек. Отвечает за углы комнаты, нахождение диагоналей. Необходим для расчёта точек облёта.
sensor_msgs::PointCloud cloud_filter;
sensor_msgs::PointCloud room_p; // Облако точек. Отвечает за хранение точек облёта и их отправку в отдельный топик
geometry_msgs::Point32 xyu; // Промежуточная переменная для конвертации центров дверей в облако
geometry_msgs::Point32 left_door_point;
geometry_msgs::Point32 right_door_point;
geometry_msgs::Point32 room; // Промежуточная переменная для перадачи координат произвольных точек в облако 
geometry_msgs::Point32 mid; // Промедуточная переменная для вычисления центра комнаты, хранения координат этой точки и отправки этих значений в облако

// Колбэк роса для передачи на прямую значений с лидара
void subs(const sensor_msgs::LaserScan& msg) {
   msg_first_lid = msg;
   //ROS_INFO_STREAM(msg_first_lid.ranges.size());
   if (!inf_done_at_least_once) inf_done_at_least_once = true;
}

// Заполнение данных LaserScan
void inf(const sensor_msgs::LaserScan& msg) {
   msg_new.angle_increment = msg.angle_increment;
   msg_new.angle_max = msg.angle_max;
   msg_new.angle_min = msg.angle_min;
   msg_new.header.stamp = msg.header.stamp;
   msg_new.range_min = msg.range_min;
   msg_new.range_max = msg.range_max;
   msg_new.ranges.resize(msg.ranges.size());
}

// Конвертируем точки краёв дверей в облако точек
void calculate_edge_door(const sensor_msgs::LaserScan& msg_new, float l_dr_i, float r_dr_i) { 
    left_door_point.y = - msg_new.ranges[l_dr_i] * sin(l_dr_i * M_PI / 180);
    left_door_point.x = - msg_new.ranges[l_dr_i] * cos(l_dr_i * M_PI / 180);

    right_door_point.y = - msg_new.ranges[r_dr_i] * sin(r_dr_i * M_PI / 180);
    right_door_point.x = - msg_new.ranges[r_dr_i] * cos(r_dr_i * M_PI / 180);
}

// Конвертируем точки центров дверей в облако точек
void raschet_door(const sensor_msgs::LaserScan& msg_new, float l_dr_i, float r_dr_i) { 
    xyu.y = - msg_new.ranges[int( (r_dr_i-1 + l_dr_i)/2 )] * sin((int( (r_dr_i-1 + l_dr_i)/2 )) * M_PI / 180);
    xyu.x = - msg_new.ranges[int( (r_dr_i-1 + l_dr_i)/2 )] * cos((int( (r_dr_i-1 + l_dr_i)/2 )) * M_PI / 180);
}

// Расчёт углов комнаты
void raschet_angles(const sensor_msgs::LaserScan& msg_new, float i) { 
    room.y = - msg_new.ranges[i] * sin(i * M_PI / 180);
    room.x = - msg_new.ranges[i] * cos(i * M_PI / 180);
}

// Расчёт точек облёта
void points_drone(const sensor_msgs::PointCloud& cloud2, const geometry_msgs::Point32& mid, int log1, int log2) { 
    ROS_INFO_STREAM("points_drone()\nRAZMER 1 --> " << cloud2.points.size() << "\nlog1 --> " << log1 << "\nlog2 -->" << log2 << std::endl);
    dst_up = sqrt( pow(cloud2.points.at(log1).x - mid.x, 2.0) + pow(cloud2.points.at(log1).y - mid.y, 2.0) );
    k = log2 * 1.5 / dst_up;
    room.x = mid.x + (cloud2.points.at(log1).x - mid.x) * k;
    room.y = mid.y + (cloud2.points.at(log1).y - mid.y) * k;
}

// Расчёт длины вектора
float math_vec(const sensor_msgs::PointCloud& cloud_log, int log1, int log2) { 
    return sqrt( pow(cloud_log.points.at(log1).x - cloud_log.points.at(log2).x, 2.0) + pow(cloud_log.points.at(log1).y - cloud_log.points.at(log2).y, 2.0) );
}

// Сортировка точек облёта в облако
void sort_vec(sensor_msgs::PointCloud& room_p, const geometry_msgs::Point32& log) { 
    room_p.points.push_back(log);
    room_p.header.stamp = ros::Time::now();
}

// Вычисление центра вектора по X
float math_midX(const sensor_msgs::PointCloud& cloud2, int log1, int log2) { 
    return (cloud2.points.at(log1).x + cloud2.points.at(log2).x) / 2;
}

// Вычисление центра вектора по Y
float math_midY(const sensor_msgs::PointCloud& cloud2, int log1, int log2) { 
    return (cloud2.points.at(log1).y + cloud2.points.at(log2).y) / 2;
}

// Удаление уже просканированных точек углов комнаты (удаление лишних)
void del_point (sensor_msgs::PointCloud& cloud2, int log) { 
    cloud2.points.erase(cloud2.points.begin() + log);
}

///////////////__-------MAIN-------__/////////////////////   

int main(int argc, char **argv) {
   ros::init(argc, argv, "first_task");
   msg_new.header.frame_id = "laser";
   cloud.header.frame_id = "laser";
   cloud2.header.frame_id = "laser";
   cloud_filter.header.frame_id = "laser";
   room_p.header.frame_id = "laser";
   ros::NodeHandle n;
    
   ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("banana", 1000);
   ros::Subscriber sub = n.subscribe("scan", 1000, subs);
   ros::Publisher convert = n.advertise<sensor_msgs::PointCloud>("Doors_mid", 100);
   ros::Publisher convert2 = n.advertise<sensor_msgs::PointCloud>("Ang_and_room_mid", 100); 
   ros::Publisher Room_points = n.advertise<sensor_msgs::PointCloud>("Points_for_drone", 100); 
    
   ros::Rate loop_rate(5);
    
while (ros::ok()) {

   using std::cout, std::endl;
   inf(msg_first_lid);
   cout << "//////////////////////////" << endl;
   bool flag = true; // Флажок. Используется для расчёта углов комнаты
   cloud.points.clear();
   cloud2.points.clear();
   cloud_filter.points.clear();
   room_p.points.clear();
   float ko = 0; // Дополнительный коэффициент для вычисления прогрессии и зарисовки дверей
    
   if (inf_done_at_least_once) {

      for (int i = 358; i >= 1; i--) {
         if (msg_first_lid.ranges[i] > 5) {
         msg_new.ranges[i] = 5;
         }
         else msg_new.ranges[i] = msg_first_lid.ranges[i]; 
         cout << "LOGS " << i << " --> " << msg_new.ranges[i] << endl;
         
         // Определяем левый край двери
         if ( (msg_new.ranges[i] != msg_new.ranges[i+1]) && (msg_new.ranges[i] == 5) ) {
         l_dr = msg_new.ranges[i+2];
         ko = l_dr;
         l_dr_i = i+2;
         cout << "    Leviy kriy №" << i+2 << " --> " << l_dr << endl;
         }
         /////////////////////////

         // Находим углы комнаты 	
               bool is_status = (msg_new.ranges[i+1] - msg_new.ranges[i] < 0) && (abs(msg_new.ranges[i+1] - msg_new.ranges[i]) >= 0.025); 
               if (is_status) { // Выставляем флажок
                  flag = false;
               }
               is_status = (flag == false) && (msg_new.ranges[i+1] - msg_new.ranges[i] > 0) && (abs(msg_new.ranges[i+1] - msg_new.ranges[i]) >= 0.025);
               if (is_status) { // Узнаём действительно ли точка яыляется углом комнаты. Если да, то сохраняем её
                  flag = true;
                  raschet_angles(msg_new, i);
                  cloud2.points.push_back(room);
               }
         ///////////////////////

         // Определяем правый край двери ------------------------
         if ( (msg_new.ranges[i] != msg_new.ranges[i+1]) && (msg_new.ranges[i+1] == 5) ) {
         r_dr = msg_new.ranges[i-1];
         r_dr_i = i-1;
         cout << "    Praviy kriy №" << i-1 << " --> " << r_dr << endl;
         
               // Считаем все коеффициенты по формулам +
               koef_n = l_dr_i - r_dr_i - 1;
               koef_door = pow(r_dr / l_dr, 1 / (koef_n - 1));
               ///////////////////// +
         
         calculate_edge_door(msg_new, l_dr_i, r_dr_i);
         cloud_filter.points.push_back(left_door_point);
         cloud_filter.points.push_back(right_door_point);

         cout << "DLINA DVERNOGO PROEMA SIZE --> " << cloud_filter.points.size() << endl;
         float length_of_door_vec = math_vec(cloud_filter, 0, 1);
         cout << "DLINA DVERNOGO PROEMA --> " << length_of_door_vec << endl;

         // Зарисовываем дверь /
         for (int i = l_dr_i+1; i >= r_dr_i; i--) {
            if (msg_new.ranges[i] == 5) {
            ko *= koef_door;
            msg_new.ranges[i] = ko;
            }
         }
         cloud_filter.points.clear();
         ///////////////////// /
               
      // Конвертируем значения в PointCloud
      raschet_door(msg_new, l_dr_i, r_dr_i);
      cloud.points.push_back(xyu);
      //////////////////////////////////////
         }

         /////////////////////////-----------------------
      }
         
         // Определяем центр комнаты по найденным ранее углам
         float max_cloud = 0; // Переменная для расчёта максимума
         float max_c2 = 0; // Переменная для расчёта максимума
         int m_i, m_j, m_i2, m_j2; // Переменная для сохранения индекса точки
         // Находим первую диагональ
         for (int i = 0; i < cloud2.points.size(); i++) {
            for (int j = i + 1; j < cloud2.points.size(); j++) {
                  if (math_vec(cloud2, i, j) > max_cloud) {
                     max_cloud = math_vec(cloud2, i, j);
                     m_i = i;
                     m_j = j;
                  }
            }
         }
        cout << "--------------------------------------------" << endl;


         // Вычисляем центр по первому вектору
         mid.x = math_midX(cloud2, m_i, m_j);
         mid.y = math_midY(cloud2, m_i, m_j);
         sort_vec(room_p, mid);

         points_drone(cloud2, mid, m_j, 1);
         sort_vec(room_p, room);

         points_drone(cloud2, mid, m_j, -1);
         sort_vec(room_p, room);

         if (m_j > m_i) { 
            del_point(cloud2, m_j); // 1 Удаление
            del_point(cloud2, m_i); // 2 Удаление
         }
         else { 
            del_point(cloud2, m_i); // 1 Удаление
            del_point(cloud2, m_j); // 2 Удаление
         }
         cout << "-----OK-----" << endl;



        // Находим вторую диагональ
         for (int i = 0; i < cloud2.points.size(); i++) {
            for (int j = i + 1; j < cloud2.points.size(); j++) {
                  if (math_vec(cloud2, i, j) > max_c2) {
                     max_c2 = math_vec(cloud2, i, j);
                     m_i2 = i;
                     m_j2 = j;
                     //cout << "I --> " << m_i2 << "\nJ --> " << m_j2 << endl;
                  }
            }
         }
         cout << "---ALL GOOD-----" << endl;
         points_drone(cloud2, mid, m_j2, 1);
         sort_vec(room_p, room);

         points_drone(cloud2, mid, m_j2, -1);
         sort_vec(room_p, room);
         ////////////////////////////////////////////////////


         pub.publish(msg_new);
         convert.publish(cloud); 
         convert2.publish(cloud2); 
         Room_points.publish(room_p);
   }

   ros::spinOnce();
   loop_rate.sleep();
}
   return 0;
}
