#include "pid_node.h"
#include <cmath>
#include <functional>
#include <algorithm>

using namespace std::chrono_literals;

PidControl::PidControl() : 
    Node("pid_node"),
    int_error_(0.0),
    prev_error_(0.0),
    obstacle_detected_(false),
    x_(0.0), y_(0.0), yaw_(0.0)
{
    // Parameter declaration
    declare_parameter<std::string>("task", "line");
    declare_parameter<double>("line_y", -7.0);
    declare_parameter<double>("oval_center_x", -6.0);
    declare_parameter<double>("oval_center_y", 0.0);
    declare_parameter<double>("oval_radius", 6.0);
    declare_parameter<double>("oval_straight_len", 12.0);
    declare_parameter<double>("task_vel", 1.0);
    declare_parameter<double>("kp", 0.0);
    declare_parameter<double>("ki", 0.0);
    declare_parameter<double>("kd", 0.0);
    declare_parameter<double>("min_obstacle_range", 1.0);
    declare_parameter<double>("control_interval", 0.1);

    // Parameter loading
    get_parameter("task", task_);
    get_parameter("line_y", line_y_);
    oval_.center_x = get_parameter("oval_center_x").as_double();
    oval_.center_y = get_parameter("oval_center_y").as_double();
    oval_.radius = get_parameter("oval_radius").as_double();
    oval_.straight_len = get_parameter("oval_straight_len").as_double();
    get_parameter("task_vel", task_vel_);
    get_parameter("kp", kp_);
    get_parameter("ki", ki_);
    get_parameter("kd", kd_);
    get_parameter("min_obstacle_range", min_obstacle_range_);
    get_parameter("control_interval", control_interval_);

    // ROS interfaces setup
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    error_pub_ = create_publisher<std_msgs::msg::Float64>("/error", 10);
    /*
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PidControl::laserCallback, this, std::placeholders::_1));
    */
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth", 10,
        std::bind(&PidControl::poseCallback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_interval_),
        std::bind(&PidControl::controlTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Controller initialized with task: %s", task_.c_str());
}

double PidControl::crossTrackErrorLine() const
{
	return line_y_ - y_;
}


double PidControl::crossTrackErrorCircle() const
{
	double dx = oval_.center_x - x_;
    double dy = oval_.center_y - y_;
    return sqrt(dx*dx + dy*dy) - oval_.radius;
}

void PidControl::publishError(double error) const
{
    auto msg = std_msgs::msg::Float64();
    msg.data = error;
    error_pub_->publish(msg);
}

void PidControl::poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
}

double PidControl::crossTrackErrorOval() const
{
    const double right_circle_x = oval_.center_x + oval_.straight_len;
    const double upper_y = oval_.center_y + oval_.radius;
    const double lower_y = oval_.center_y - oval_.radius;

    // Determine current track section
    if(x_ < oval_.center_x) {
        // Left circle section
        const double dx = oval_.center_x - x_;
        const double dy = oval_.center_y - y_;
        return std::hypot(dx, dy) - oval_.radius;
    }
    else if(x_ > right_circle_x) {
        // Right circle section
        const double dx = x_ - right_circle_x;
        const double dy = oval_.center_y - y_;
        return std::hypot(dx, dy) - oval_.radius;
    }
    else {
        // Straight section - calculate vertical error
        const double target_y = (y_ > oval_.center_y) ? upper_y : lower_y;
        return (target_y < oval_.center_y) ? target_y - y_ : -(target_y - y_);
    }
}


void PidControl::controlTimerCallback()
{
    geometry_msgs::msg::Twist cmd;
    if(!obstacle_detected_) {
        double error = 0.0;
        
        if(task_ == "line") {
            error = crossTrackErrorLine();
        }
        else if(task_ == "circle") {
            error = crossTrackErrorCircle();
        }
        else if(task_ == "oval") {
            error = crossTrackErrorOval();
        }
        else {
            RCLCPP_ERROR(get_logger(), "Unknown task type: %s", task_.c_str());
            return;
        }

        // PID calculations
        const double dt = control_interval_;
        const double integral = int_error_ + error * dt;
        const double derivative = (error - prev_error_) / dt;
        
        cmd.linear.x = task_vel_;
        cmd.angular.z = kp_ * error + ki_ * integral + kd_ * derivative;

        // Update state
        int_error_ = integral;
        prev_error_ = error;
        
        publishError(error);
    }
    
    cmd_vel_pub_->publish(cmd);
    obstacle_detected_ = false;  // Reset obstacle flag
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}