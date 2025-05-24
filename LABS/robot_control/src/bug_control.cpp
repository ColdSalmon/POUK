#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node {
public:
  ControlNode() : Node("bug_control_node"), obstacle(false), is_close(false) {
    // Инициализация подписчиков и таймера
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "base_scan", 10,
      std::bind(&ControlNode::laserCallback, this, std::placeholders::_1));
    
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "base_pose_ground_truth", 10,
      std::bind(&ControlNode::poseCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      100ms, std::bind(&ControlNode::timerCallback, this));

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    //RCLCPP_INFO(get_logger(), "Laser msg: %f", msg->scan_time);
    const double kMinRange = 0.2;
	const double kObhRange = 1.5;
    float start_angle = msg->angle_min;
    float end_angle = msg->angle_max;
    float step_angle = msg->angle_increment;
	
	float start_obst_angle = -20 * M_PI/180;
	float end_obst_angle = 20 * M_PI/180;
    // Пример использования
	/*
    RCLCPP_INFO(this->get_logger(), 
        "Scan sector: %.2f to %.2f radians", 
        start_angle, 
        end_angle);
	*/
	float scan_angle = start_angle;
	int ranges_count = msg->ranges.size();
	is_close = false;
	for(int i = 0; i < ranges_count; i++)
	{
		if (msg->ranges[i] < kMinRange) {
			obstacle = true;
			RCLCPP_WARN(get_logger(), "OBSTACLE!!!");
		}
		if((scan_angle >= start_obst_angle) and (scan_angle <= end_obst_angle))
		{
			if(msg->ranges[i] < kObhRange)
			{
				is_close = true;
				RCLCPP_INFO(get_logger(), "Laser msg: %f", msg->scan_time);
			}
		}
		if(scan_angle < end_angle) scan_angle += step_angle;
	}
	/*
    for (const auto& range : msg->ranges) {
      if (range < kMinRange) {
        obstacle = true;
		is_close = true;
        RCLCPP_WARN(get_logger(), "OBSTACLE!!!");
      }
	  else{
		is_close = false;
	  }
    }
	*/
  }

  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), 
      "Pose msg: x = %f y = %f theta = %f",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      2 * atan2(msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w));
  }

  void timerCallback() {
    //static int counter = 0;
    //counter++;
    //RCLCPP_DEBUG(get_logger(), "on timer %d", counter);
    
    auto cmd = geometry_msgs::msg::Twist();
    
	
    if (!is_close) {
		cmd.linear.x = 0.5;
		cmd.angular.z = 0.0;
	  /*
	  if (counter % 30 > 15) {
        RCLCPP_INFO(get_logger(), "go left");
        cmd.linear.x = 0.5;
        cmd.angular.z = 0.5;
      } else {
        RCLCPP_INFO(get_logger(), "go right");
        cmd.linear.x = 0.5;
        cmd.angular.z = -0.5;
      }
	  */
    }
	else
	{
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.5;
	}
    publisher_->publish(cmd);
  }

  // Члены класса
  bool obstacle;
  bool is_close;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
