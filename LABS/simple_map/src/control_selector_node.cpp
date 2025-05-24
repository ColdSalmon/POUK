#include "control.h"
#include "voyager_control.h"
#include "dummy_control.h"
#include "wall_follower_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
    enum ControlEnum { DUMMY, VOYAGER, WALL_FOLLOWER, nControls };

    ControlNode() : Node("control_selector_node")
    {
        declare_parameters();
        init_publishers();
        init_subscriptions();
        init_controls();
        init_timer();
    }

    ~ControlNode()
    {
        for(int i = 0; i < nControls; ++i) delete controls[i];
    }

private:
    void declare_parameters()
    {
        this->declare_parameter<double>("min_range", 1.0);
        this->declare_parameter<double>("max_vel", 0.5);
        this->declare_parameter<double>("max_omega", 0.5);
		
		this->declare_parameter<double>("wall_target_dist", 0.5);
		this->declare_parameter<double>("wall_max_vel", 0.4);
		this->declare_parameter<double>("wall_kP", 1.5);
		this->declare_parameter<double>("wall_front_dist", 0.7);
    }

    void init_publishers()
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(100));
    }

    void init_subscriptions()
    {
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/base_scan", rclcpp::QoS(1),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                if(control_ptr_) control_ptr_->setLaserData(msg->ranges);
            });

        pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/base_pose_ground_truth", rclcpp::QoS(1),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                const auto& p = msg->pose.pose.position;
                const auto& q = msg->pose.pose.orientation;
                double theta = 2.0 * atan2(q.z, q.w);
                if(control_ptr_) control_ptr_->setRobotPose(p.x, p.y, theta);
            });

        select_sub_ = create_subscription<std_msgs::msg::UInt16>(
            "/selector", rclcpp::QoS(100),
            [this](const std_msgs::msg::UInt16::SharedPtr msg) {
                if(msg->data >= nControls) {
                    RCLCPP_ERROR(get_logger(), "Invalid control index: %d", msg->data);
                    control_ptr_ = nullptr;
                } else {
                    control_ptr_ = controls[msg->data];
                    RCLCPP_INFO(get_logger(), "Selected controller: %s", 
                        control_ptr_->getName().c_str());
                }
            });
    }

    void init_controls()
    {
        controls[DUMMY] = new DummyControl();
        controls[VOYAGER] = new VoyagerControl(
            get_parameter("min_range").as_double(),
            get_parameter("max_vel").as_double(),
            get_parameter("max_omega").as_double()
        );
		controls[WALL_FOLLOWER] = new WallFollowerControl(
			get_parameter("wall_target_dist").as_double(),
			get_parameter("wall_max_vel").as_double(),
			get_parameter("wall_kP").as_double(),
			get_parameter("wall_front_dist").as_double()
		);
        control_ptr_ = controls[VOYAGER];
    }

    void init_timer()
    {
        timer_ = create_wall_timer(100ms, [this]() {
            geometry_msgs::msg::Twist cmd;
            if(control_ptr_) {
                double v, w;
                control_ptr_->getControl(v, w);
                cmd.linear.x = v;
                cmd.angular.z = w;
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1, "No active controller");
            }
            cmd_pub_->publish(cmd);
        });
    }

    // Члены класса
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr select_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    Control* controls[nControls] = {nullptr};
    Control* control_ptr_ = nullptr;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}