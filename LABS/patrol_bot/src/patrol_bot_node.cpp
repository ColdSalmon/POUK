#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class PatrolController : public rclcpp::Node {
public:
  PatrolController() : Node("patrol_controller"), current_goal_index_(-1) {
    // Инициализация паблишера для отправки целей
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10);

    // Подписка на точки из интерфейса
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&PatrolController::point_callback, this, std::placeholders::_1));

    // Таймер для циклической отправки целей
    timer_ = create_wall_timer(10000ms, [this]() {
      if (!goals_.empty()) {
        publish_next_goal();
      }
    });
  }

private:
  // Константы
  static constexpr size_t MAX_GOALS = 5;
  static constexpr double TARGET_YAW = M_PI/2; // 90 градусов

  // Члены класса
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  int current_goal_index_;

  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (goals_.size() >= MAX_GOALS) {
      RCLCPP_WARN(get_logger(), "Maximum number of goals (%d) reached!", MAX_GOALS);
      return;
    }

    // Создание новой цели
    geometry_msgs::msg::PoseStamped new_goal;
    new_goal.header = msg->header;
    new_goal.pose.position = msg->point;
    
    // Задание ориентации (фиксированный поворот на 90 градусов)
    new_goal.pose.orientation.z = sin(TARGET_YAW/2);
    new_goal.pose.orientation.w = cos(TARGET_YAW/2);

    goals_.push_back(new_goal);
    RCLCPP_INFO(get_logger(), "New goal added: (%.2f, %.2f)", 
               msg->point.x, msg->point.y);
  }

  void publish_next_goal() {
    current_goal_index_ = (current_goal_index_ + 1) % goals_.size();
    auto goal = goals_[current_goal_index_];
    
    // Обновление временной метки
    goal.header.stamp = now();
    
    // Публикация цели
    goal_publisher_->publish(goal);
    RCLCPP_INFO(get_logger(), "Published goal #%d: (%.2f, %.2f)",
               current_goal_index_ + 1,
               goal.pose.position.x,
               goal.pose.position.y);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}