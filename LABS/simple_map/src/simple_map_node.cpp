#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Глобальная переменная - публикатор сообщения карты
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
// Глобальный указатель на tfListener, который будет проинициализирован в main
std::shared_ptr<tf2_ros::TransformListener> tfListener;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
// Имя для СК карты
std::string map_frame;
// Разрешение карты
double map_resolution = 0.1;
// Размер карты в клетках
int map_width = 100;
int map_height = 100;

void prepareMapMessage(nav_msgs::msg::OccupancyGrid &map_msg, const rclcpp::Time &stamp)
{
    map_msg.header.frame_id = map_frame;
    map_msg.header.stamp = stamp;
    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.info.resolution = map_resolution;
    // Изменяем размер вектора, который является хранилищем данных карты, и заполняем его значением (-1) - неизвестное значение
    map_msg.data.resize(map_height * map_width, -1);
}

bool determineScanTransform(geometry_msgs::msg::TransformStamped &scanTransform,
                          const rclcpp::Time &stamp,
                          const std::string &laser_frame)
{
    try
    {
        scanTransform = tfBuffer->lookupTransform(
            map_frame,
            laser_frame,
            stamp,
            rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException &e)
    {
        return false;
    }
    return true;
}

/**
* Функция, которая будет вызвана
* при получении данных от лазерного дальномера
*/
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    geometry_msgs::msg::TransformStamped scanTransform;
    const std::string &laser_frame = scan->header.frame_id;
    const rclcpp::Time &laser_stamp = scan->header.stamp;
    
    if (!determineScanTransform(scanTransform, laser_stamp, laser_frame))
    {
        return;
    }
    
    // Создаем сообщение карты
    auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    // Заполняем информацию о карте - готовим сообщение
    prepareMapMessage(*map_msg, laser_stamp);
    
    // Положение центра дальномера в СК дальномера
    geometry_msgs::msg::Point zero_pose;
    zero_pose.x = 0;
    zero_pose.y = 0;
    zero_pose.z = 0;
    
    // Положение дальномера в СК карты
    geometry_msgs::msg::Point scan_pose;
    tf2::doTransform(zero_pose, scan_pose, scanTransform);
    
    
    // Задаем начало карты так, чтобы сканнер находился в центре карты
    map_msg->info.origin.position.x = scan_pose.x - map_width * map_resolution / 2.0;
    map_msg->info.origin.position.y = scan_pose.y - map_height * map_resolution / 2.0;
    
    // Индексы карты, соответствующие положению центра лазера
    int center_y = (scan_pose.y - map_msg->info.origin.position.y) / map_resolution;
    int center_x = (scan_pose.x - map_msg->info.origin.position.x) / map_resolution;
    
    
    // В клетку карты соответствующую центру лазера - записываем значение 0
    map_msg->data[center_y * map_width + center_x] = 0;
    int map_idx_max = map_width * map_height;
    
    // Проходим по каждому измерению лидара
    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
        {
            continue;
        }
        
        // Угол в ПСК ЛД
        float angle = scan->angle_min + i * scan->angle_increment;
        
        // Вычисляем позицию препятствия в системе координат ЛД
        geometry_msgs::msg::Point obstacle_pose;
        obstacle_pose.x = scan->ranges[i] * cos(angle);
        obstacle_pose.y = scan->ranges[i] * sin(angle);
        obstacle_pose.z = 0.0;
        
        // Шаг для прохода по лучу
        double step = 0.1;
        
        // Идем по лучу от ЛД до препятствия
        for (double r = scan->range_min; r < scan->ranges[i] - step; r += step)
        {
            // Точка в ДСК ЛД
            geometry_msgs::msg::Point free_pos;
            free_pos.x = r * cos(angle);
            free_pos.y = r * sin(angle);
            free_pos.z = 0.0;
            
            // Точка в ДСК Карты
            geometry_msgs::msg::Point free_pos_map;
            tf2::doTransform(free_pos, free_pos_map, scanTransform);
            
            // Координаты точки карты
            int free_x = (free_pos_map.x - map_msg->info.origin.position.x) / map_resolution;
            int free_y = (free_pos_map.y - map_msg->info.origin.position.y) / map_resolution;
            
            // Индекс в массиве карты
            int map_free_idx = free_y * map_width + free_x;
            
            // Проверяем, что ячейка не находится за пределами карты
            if (map_free_idx > 0 && map_free_idx < map_idx_max)
            {
                map_msg->data[map_free_idx] = 0;
            }
        }
        
        // Вычисляем позицию препятствия в системе координат карты
        geometry_msgs::msg::Point obstacle_pose_map;
        tf2::doTransform(obstacle_pose, obstacle_pose_map, scanTransform);
        
        // Индексы ячейки, соответствующей позиции препятствия
        int obstacle_x = (obstacle_pose_map.x - map_msg->info.origin.position.x) / map_resolution;
        int obstacle_y = (obstacle_pose_map.y - map_msg->info.origin.position.y) / map_resolution;
        int map_idx = obstacle_y * map_width + obstacle_x;
        
        // Проверяем, что ячейка не находится за пределами карты
        if (map_idx > 0 && map_idx < map_idx_max)
        {
            map_msg->data[map_idx] = 100;
        }
    }
    
    // Публикуем сообщение с построенной картой
    mapPub->publish(std::move(map_msg));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("control_selector_node");
    
    // Читаем параметры
    node->declare_parameter<std::string>("map_frame", "odom");
    node->declare_parameter("map_resolution", map_resolution);
    node->declare_parameter("map_width", map_width);
    node->declare_parameter("map_height", map_height);
    
    map_frame = node->get_parameter("map_frame").as_string();
    map_resolution = node->get_parameter("map_resolution").as_double();
    map_width = node->get_parameter("map_width").as_int();
    map_height = node->get_parameter("map_height").as_int();
    
    // Создание объектов TF2
    tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    
    // Подписываемся на данные дальномера
    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/base_scan", 100, laserCallback);
    
    // Объявляем публикацию сообщений карты
    mapPub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}