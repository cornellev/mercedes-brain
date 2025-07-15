#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <cmath>
#include <vector>
#include <pigpiod_if2.h>

class GPIONode : public rclcpp::Node{
public: 
    GPIONode() : Node("gpio_node") {
        pi_ = pigpio_start(NULL, NULL);
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
            rclcpp::shutdown();
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "pigpio initialized");
        } 

        set_mode(pi_, 17, PI_OUTPUT);
        set_mode(pi_, 27, PI_OUTPUT);
        set_mode(pi_, 22, PI_OUTPUT);

        rc_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("rc_msg", 1, std::bind(&GPIONode::rc_callback, this, std::placeholders::_1));
    }

    ~GPIONode() {
        if (pi_ >= 0) {
            pigpio_stop(pi_);
        }
    }

private:
    int pi_{-1};
    float velocity_ = 0;
    float steer_ = 0;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_sub_;

    void rc_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        velocity_ = msg->speed;
        steer_ = msg->steering_angle;

        // drive
        int v_pwm = static_cast<int>(std::abs(velocity_) * 255);
        bool v_dir = (velocity_ > 0);
        gpio_write(pi_, 6, !v_dir);
        gpio_write(pi_, 13, v_dir);
        set_PWM_dutycycle(pi_, 12, v_pwm);
        set_PWM_dutycycle(pi_, 16, v_pwm);

        // steer
        int s_pwm = static_cast<int>(0.5 * 255);
        if (std::abs(steer_) < 0.1) {
            s_pwm = 0;
        }
        bool s_dir = (steer_ < 0);
        gpio_write(pi_, 19, true);
        gpio_write(pi_, 26, true);
        gpio_write(pi_, 20, s_dir);
        set_PWM_dutycycle(pi_, 21, s_pwm);
        set_PWM_frequency(pi_, 21, 4000);
        
        RCLCPP_INFO(this->get_logger(), "Velocity: %f\tSteer: %f", velocity_, steer_);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}