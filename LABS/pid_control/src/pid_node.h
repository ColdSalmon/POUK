#ifndef PID_NODE_H
#define PID_NODE_H

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class PidControl : public rclcpp::Node
{
public:
    explicit PidControl();
    
private:
    // Error calculation methods
    double crossTrackErrorLine() const;
    double crossTrackErrorCircle() const;
    double crossTrackErrorOval() const;
    
    // ROS callbacks
    //void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void controlTimerCallback();
    void publishError(double error) const;

    // Parameters
    struct OvalParams {
        double center_x;      // Left circle center X
        double center_y;      // Circles center Y
        double radius;        // Circle radius
        double straight_len;  // Length of straight section
    };
    
    // Configuration
    std::string task_;
    double line_y_;
    OvalParams oval_;
    double task_vel_;
    double kp_, ki_, kd_;
    double min_obstacle_range_;
    double control_interval_;
    
    // State
    double int_error_;
    double prev_error_;
    bool obstacle_detected_;
    double x_, y_, yaw_;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif // CONTROL_NODE_H