#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <memory>

class GazeboKeyboardController : public rclcpp::Node
{
public:
    GazeboKeyboardController() : Node("gazebo_keyboard_controller")
    {
        // 创建力矩发布器
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Rwheel_joint/cmd_force", 10);
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Lwheel_joint/cmd_force", 10);
        
        // 订阅Gazebo键盘输入
        keyboard_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/keyboard/keypress", 10,
            std::bind(&GazeboKeyboardController::keyboard_callback, this, std::placeholders::_1));
        
        // 参数设置
        this->declare_parameter("max_force", 8.0);
        max_force_ = this->get_parameter("max_force").as_double();
        
        // 创建定时器来停止轮子（如果没有新的按键输入）
        stop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),  // 200ms后自动停止
            std::bind(&GazeboKeyboardController::auto_stop_check, this));
        
        last_key_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "=== Gazebo Keyboard Controller Started! ===");
        RCLCPP_INFO(this->get_logger(), "Listening for Gazebo keyboard input...");
        RCLCPP_INFO(this->get_logger(), "Controls (focus on Gazebo window):");
        RCLCPP_INFO(this->get_logger(), "  W(87) - Forward");
        RCLCPP_INFO(this->get_logger(), "  S(83) - Backward");
        RCLCPP_INFO(this->get_logger(), "  A(65) - Turn Left");
        RCLCPP_INFO(this->get_logger(), "  D(68) - Turn Right");
        RCLCPP_INFO(this->get_logger(), "Max force: %.2f N", max_force_);
    }
    
    ~GazeboKeyboardController()
    {
        // 停止轮子
        stopWheels();
        RCLCPP_INFO(this->get_logger(), "Gazebo Keyboard Controller shutdown complete.");
    }

private:
    void keyboard_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int key_code = msg->data;
        
        // 调试输出：显示接收到的按键码
        RCLCPP_INFO(this->get_logger(), "Received key code: %d", key_code);
        
        // 更新最后按键时间
        last_key_time_ = this->now();
        
        double right_force = 0.0;
        double left_force = 0.0;
        std::string action = "";
        
        switch(key_code) {
            case 87:  // W - Forward
                right_force = -max_force_;
                left_force = max_force_;
                action = "Forward";
                break;
                
            case 83:  // S - Backward
                right_force = max_force_;
                left_force = -max_force_;
                action = "Backward";
                break;
                
            case 65:  // A - Turn Left
                right_force = -max_force_ * 0.7;
                left_force = -max_force_ * 0.7;
                action = "Turn Left";
                break;
                
            case 68:  // D - Turn Right
                right_force = max_force_ * 0.7;
                left_force = max_force_ * 0.7;
                action = "Turn Right";
                break;
                
            default:
                // 未知按键，停止
                right_force = 0.0;
                left_force = 0.0;
                action = "Unknown key - Stop";
                break;
        }
        
        // 发布力矩指令
        publishForces(right_force, left_force);
        
        // 输出动作信息
        RCLCPP_INFO(this->get_logger(), "%s - Right: %.2f, Left: %.2f", 
            action.c_str(), right_force, left_force);
    }
    
    void auto_stop_check()
    {
        // 如果超过一定时间没有按键输入，自动停止
        auto now = this->now();
        auto time_diff = (now - last_key_time_).seconds();
        
        if (time_diff > 0.5) {  // 500ms后自动停止
            if (current_right_force_ != 0.0 || current_left_force_ != 0.0) {
                stopWheels();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Auto-stop: No key input for %.1fs", time_diff);
            }
        }
    }
    
    void publishForces(double right_force, double left_force)
    {
        current_right_force_ = right_force;
        current_left_force_ = left_force;
        
        auto right_msg = std_msgs::msg::Float64();
        auto left_msg = std_msgs::msg::Float64();
        
        right_msg.data = right_force;
        left_msg.data = left_force;
        
        right_wheel_pub_->publish(right_msg);
        left_wheel_pub_->publish(left_msg);
    }
    
    void stopWheels()
    {
        publishForces(0.0, 0.0);
    }

private:
    // ROS2 组件
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keyboard_sub_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    
    // 参数和状态
    double max_force_;
    double current_right_force_ = 0.0;
    double current_left_force_ = 0.0;
    rclcpp::Time last_key_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GazeboKeyboardController>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
