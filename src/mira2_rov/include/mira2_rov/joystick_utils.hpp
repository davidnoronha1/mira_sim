#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class JoystickController : public rclcpp::Node {
public:
    JoystickController() : Node("joystick_controller") {
        // Initialize publishers and subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickController::joyCallback, this, std::placeholders::_1));
        thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bluerov2/thrusters", 10);

        // Initialize default PWM values for 8 thrusters
        pwm_values_ = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

        sensitivity_all_axes_ = 0.6;
        sensitivity_yaw_ = 0.5;
        sensitivity_pitch_ = 0.5;

        prev_msg_ = 0;
        arm_disarm_ = 0;
        prev_conversion_toggle_ = 0;
        armed_ = false;
        apply_pwm_conversion_ = false; // Toggle for PWM conversion
        current_mode_ = "STABILIZE";
        
        RCLCPP_INFO(this->get_logger(), "Joystick Controller initialized");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        prev_msg_ = arm_disarm_;
        arm_disarm_ = msg->buttons[0];
        int mode_manual = msg->buttons[1];
        int mode_depth_hold = msg->buttons[2];
        int mode_stabilise = msg->buttons[3];

        int roll_left_button = msg->buttons[4];
        int roll_right_button = msg->buttons[5];
        
        // Toggle for PWM conversion (using button 6)
        if (msg->buttons[6] == 1 && prev_conversion_toggle_ == 0) {
            apply_pwm_conversion_ = !apply_pwm_conversion_;
            RCLCPP_INFO(this->get_logger(), "PWM conversion %s", apply_pwm_conversion_ ? "ENABLED" : "DISABLED");
        }
        prev_conversion_toggle_ = msg->buttons[6];

        // Calculate control values
        float pitch = 1500 + ((msg->axes[4]) * 400) * sensitivity_pitch_;
        float thrust = 1500 + ((((msg->axes[5]) + 1) * -200) + (((msg->axes[2]) + 1) * 200)) * sensitivity_all_axes_;
        float forward = 1500 + ((msg->axes[1]) * 400) * sensitivity_all_axes_;
        float lateral = 1500 + ((msg->axes[0]) * -400) * sensitivity_all_axes_;
        float yaw = 1500 + (((msg->axes[3]) * -400)) * sensitivity_yaw_;
        float roll = 1500;
        
        if (roll_left_button == 1) {
            roll = 1400;
        } else if (roll_right_button == 1) {
            roll = 1600;
        }

        // Arm/Disarm logic
        if (arm_disarm_ == 1 && prev_msg_ == 0) {
            armed_ = !armed_;
            RCLCPP_WARN(this->get_logger(), "VEHICLE %s", armed_ ? "ARMED" : "DISARMED");
        }

        // Mode changes
        if (mode_stabilise == 1 && current_mode_ != "STABILIZE") {
            current_mode_ = "STABILIZE";
            RCLCPP_INFO(this->get_logger(), "Mode changed to STABILIZE");
        }
        if (mode_depth_hold == 1 && current_mode_ != "ALT_HOLD") {
            current_mode_ = "ALT_HOLD";
            RCLCPP_INFO(this->get_logger(), "Mode changed to ALT_HOLD");
        }
        if (mode_manual == 1 && current_mode_ != "MANUAL") {
            current_mode_ = "MANUAL";
            RCLCPP_INFO(this->get_logger(), "Mode changed to MANUAL");
        }

        // Only update thruster values if armed
        if (armed_) {
            // Map control values to thrusters
            // BlueROV2 has 8 thrusters in a configuration that allows for 6DOF movement
            pwm_values_[0] = forward + yaw;      // Front right
            pwm_values_[1] = forward - yaw;      // Front left
            pwm_values_[2] = -forward + yaw;     // Back right
            pwm_values_[3] = -forward - yaw;     // Back left
            pwm_values_[4] = thrust + pitch;     // Vertical right
            pwm_values_[5] = thrust - pitch;     // Vertical left
            pwm_values_[6] = lateral + roll;     // Lateral right
            pwm_values_[7] = lateral - roll;     // Lateral left
            
            // Clamp values to valid PWM range
            for (auto& pwm : pwm_values_) {
                pwm = std::max(1100.0, std::min(1900.0, pwm));
            }
        } else {
            // Set neutral values when disarmed
            std::fill(pwm_values_.begin(), pwm_values_.end(), 1500);
        }

        // Prepare the message
        std_msgs::msg::Float64MultiArray thruster_msg;
        thruster_msg.data.resize(pwm_values_.size());
        
        // Apply conversion if enabled
        if (apply_pwm_conversion_) {
            // Apply the math transformation: pwm_setpoint = [(x-1500)/400 for x in pwm_thrusters]
            for (size_t i = 0; i < pwm_values_.size(); ++i) {
                thruster_msg.data[i] = (pwm_values_[i] - 1500) / 400;
            }
        } else {
            // Send raw PWM values
            for (size_t i = 0; i < pwm_values_.size(); ++i) {
                thruster_msg.data[i] = pwm_values_[i];
            }
        }

        // Publish the message
        thruster_pub_->publish(thruster_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
    std::vector<double> pwm_values_;
    float sensitivity_all_axes_, sensitivity_yaw_, sensitivity_pitch_;
    int prev_msg_, arm_disarm_, prev_conversion_toggle_;
    bool armed_, apply_pwm_conversion_;
    std::string current_mode_;
};