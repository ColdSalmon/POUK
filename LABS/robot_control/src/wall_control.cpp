#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class WallFollower : public rclcpp::Node {
public:
  WallFollower() : Node("wall_control_node"), first_laser_msg(true), right_dist(0.0), front_right_dist(0.0), front_obstacle(false) {
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "base_scan", 10,
      std::bind(&WallFollower::laserCallback, this, std::placeholders::_1));
    
    timer_ = create_wall_timer(
      100ms, std::bind(&WallFollower::timerCallback, this));

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    desired_distance = 0.5;  // Желаемая дистанция от стены (метры)
    Kp = 1.0;               // Коэффициент пропорционального регулирования
    front_safe_distance = 0.7; // Безопасное расстояние для поворота
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (first_laser_msg) {
      angle_min = msg->angle_min;
      angle_increment = msg->angle_increment;
      int num_ranges = msg->ranges.size();
      
      // Рассчитываем индексы для лучей
      float right_angle = -M_PI/2;     // Правый борт (-90°)
      float front_right_angle = -M_PI/4; // Передний-правый (-45°)
      float front_start_angle = -M_PI/6; // Начало фронтального сектора (-30°)
      float front_end_angle = M_PI/6;   // Конец фронтального сектора (+30°)

      right_index = angleToIndex(right_angle, num_ranges);
      front_right_index = angleToIndex(front_right_angle, num_ranges);
      front_start_index = angleToIndex(front_start_angle, num_ranges);
      front_end_index = angleToIndex(front_end_angle, num_ranges);

      first_laser_msg = false;
    }

    // Получаем расстояния до препятствий
    right_dist = validateRange(msg->ranges[right_index]);
    front_right_dist = validateRange(msg->ranges[front_right_index]);

    // Проверка фронтальных препятствий
    front_obstacle = false;
    for (int i = front_start_index; i <= front_end_index; ++i) {
      if (validateRange(msg->ranges[i]) < front_safe_distance) {
        front_obstacle = true;
        break;
      }
    }
  }

  int angleToIndex(float angle, int num_ranges) {
    int index = static_cast<int>((angle - angle_min) / angle_increment);
    return std::max(0, std::min(index, num_ranges - 1));
  }

  float validateRange(float range) {
    return std::isfinite(range) ? range : 10.0f;
  }

  void timerCallback() {
    auto cmd = geometry_msgs::msg::Twist();

    if (front_obstacle) {
      // Обход препятствия: поворот налево
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.5;
    } else {
      // Пропорциональное регулирование для движения вдоль стены
      double error = desired_distance - right_dist;
      double angular_z = Kp * error;

      // Коррекция для обхода углов
      if (front_right_dist < front_safe_distance) {
        angular_z += 0.3;
      }

      // Ограничение скорости вращения
      angular_z = std::clamp(angular_z, -1.0, 1.0);

      cmd.linear.x = 0.3;
      cmd.angular.z = angular_z;
    }

    publisher_->publish(cmd);
  }

  // Параметры и состояние
  bool first_laser_msg;
  float angle_min, angle_increment;
  int right_index, front_right_index, front_start_index, front_end_index;
  float right_dist, front_right_dist;
  bool front_obstacle;
  float desired_distance;
  double Kp, front_safe_distance;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}