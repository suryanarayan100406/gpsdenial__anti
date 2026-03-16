/**
 * @file motor_mixer.cpp
 * @brief Translates roll, pitch, yaw, and thrust commands to motor speeds
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;

class MotorMixer : public rclcpp::Node {
public:
    MotorMixer() : Node("motor_mixer") {
        control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/control_cmd", 10, std::bind(&MotorMixer::control_callback, this, _1));
        
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_speeds", 10);
        RCLCPP_INFO(this->get_logger(), "Motor Mixer Node Initialized.");
    }

private:
    void control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        
        float roll = msg->data[0];
        float pitch = msg->data[1];
        float yaw = msg->data[2];
        float thrust = msg->data[3];

        // Standard quadcopter X-configuration mixing
        float m1 = thrust - roll + pitch + yaw; // Front Right
        float m2 = thrust - roll - pitch - yaw; // Back Right
        float m3 = thrust + roll + pitch - yaw; // Front Left
        float m4 = thrust + roll - pitch + yaw; // Back Left

        // Clamp to physical limits (assuming 0 to 1000 range)
        auto clamp_motor = [](float val) { return std::max(0.0f, std::min(val, 1000.0f)); };

        auto motor_msg = std_msgs::msg::Float32MultiArray();
        motor_msg.data = {clamp_motor(m1), clamp_motor(m2), clamp_motor(m3), clamp_motor(m4)};
        motor_pub_->publish(motor_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Motor Mixer Node...");
    auto node = std::make_shared<MotorMixer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
