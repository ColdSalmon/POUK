#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <iostream>
#include <thread>
#include <atomic>

class InteractiveSelector : public rclcpp::Node
{
public:
    InteractiveSelector() : Node("interactive_selector_node")
    {
        // Инициализация публикатора
        selector_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
            "/selector", rclcpp::QoS(10));
            
        // Запуск потока для чтения ввода
        input_thread_ = std::thread(&InteractiveSelector::input_loop, this);
        
        // Вывод инструкции
        print_instructions();
    }

    ~InteractiveSelector()
    {
        running_.store(false);
        if(input_thread_.joinable())
            input_thread_.join();
    }

private:
    void print_instructions()
    {
        std::cout << "\n=== Control Mode Selector ===\n";
        std::cout << "Available modes:\n";
        std::cout << "0 - Dummy control\n";
        std::cout << "1 - Voyager control\n";
        std::cout << "2 - Wall follower\n";
        std::cout << "Enter mode number: ";
        std::cout.flush();
    }

    void input_loop()
    {
        while(running_.load())
        {
            std::string input;
            std::getline(std::cin, input);
            
            if(!running_.load()) break;
            
            try {
                uint16_t mode = static_cast<uint16_t>(std::stoi(input));
                publish_mode(mode);
            }
            catch(const std::exception& e) {
                std::cout << "Invalid input! Please enter a number.\n";
                print_instructions();
            }
        }
    }

    void publish_mode(uint16_t mode)
    {
        auto msg = std_msgs::msg::UInt16();
        msg.data = mode;
        selector_pub_->publish(msg);
        std::cout << "Selected mode: " << mode << std::endl;
    }

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr selector_pub_;
    std::thread input_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveSelector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}