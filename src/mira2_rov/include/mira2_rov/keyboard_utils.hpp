#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

class KeyboardController : public rclcpp::Node {
public:
    KeyboardController() : Node("keyboard_controller") {
        // Initialize publisher
        thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bluerov2/thrusters", 10);

        // Initialize default PWM values for 8 thrusters
        pwm_values_ = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

        sensitivity_all_axes_ = 0.6;
        sensitivity_yaw_ = 0.5;
        sensitivity_pitch_ = 0.5;

        armed_ = false;
        apply_pwm_conversion_ = false;
        current_mode_ = "STABILIZE";
        
        // Control states
        forward_ = 0.0;
        lateral_ = 0.0;
        thrust_ = 0.0;
        yaw_ = 0.0;
        pitch_ = 0.0;
        roll_ = 0.0;

        // Setup non-blocking keyboard input
        setupTerminal();
        
        // Start keyboard reading thread
        running_ = true;
        keyboard_thread_ = std::thread(&KeyboardController::keyboardLoop, this);
        
        // Create timer for publishing at 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&KeyboardController::publishThrusters, this));

        RCLCPP_INFO(this->get_logger(), "Keyboard Controller initialized");
        printInstructions();
    }

    ~KeyboardController() {
        running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        restoreTerminal();
    }

private:
    void setupTerminal() {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        
        // Disable canonical mode and echo
        new_tio_.c_lflag &= ~(ICANON | ECHO);
        new_tio_.c_cc[VMIN] = 0;
        new_tio_.c_cc[VTIME] = 0;
        
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
        
        // Set non-blocking mode
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    void printInstructions() {
        RCLCPP_INFO(this->get_logger(), "\n=== KEYBOARD CONTROLS ===");
        RCLCPP_INFO(this->get_logger(), "W/S: Forward/Backward");
        RCLCPP_INFO(this->get_logger(), "A/D: Strafe Left/Right");
        RCLCPP_INFO(this->get_logger(), "Q/E: Yaw Left/Right");
        RCLCPP_INFO(this->get_logger(), "R/F: Up/Down (Thrust)");
        RCLCPP_INFO(this->get_logger(), "T/G: Pitch Up/Down");
        RCLCPP_INFO(this->get_logger(), "Z/C: Roll Left/Right");
        RCLCPP_INFO(this->get_logger(), "SPACE: Arm/Disarm");
        RCLCPP_INFO(this->get_logger(), "1: MANUAL mode");
        RCLCPP_INFO(this->get_logger(), "2: ALT_HOLD mode");
        RCLCPP_INFO(this->get_logger(), "3: STABILIZE mode");
        RCLCPP_INFO(this->get_logger(), "P: Toggle PWM conversion");
        RCLCPP_INFO(this->get_logger(), "X: Exit");
        RCLCPP_INFO(this->get_logger(), "========================\n");
    }

    void keyboardLoop() {
        while (running_) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                processKey(c);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void processKey(char key) {
        // Convert to lowercase for easier handling
        key = std::tolower(key);

        switch(key) {
            // Movement controls
            case 'w':
                forward_ = 400 * sensitivity_all_axes_;
                break;
            case 's':
                forward_ = -400 * sensitivity_all_axes_;
                break;
            case 'a':
                lateral_ = -400 * sensitivity_all_axes_;
                break;
            case 'd':
                lateral_ = 400 * sensitivity_all_axes_;
                break;
            case 'q':
                yaw_ = -400 * sensitivity_yaw_;
                break;
            case 'e':
                yaw_ = 400 * sensitivity_yaw_;
                break;
            case 'r':
                thrust_ = 400 * sensitivity_all_axes_;
                break;
            case 'f':
                thrust_ = -400 * sensitivity_all_axes_;
                break;
            case 't':
                pitch_ = 400 * sensitivity_pitch_;
                break;
            case 'g':
                pitch_ = -400 * sensitivity_pitch_;
                break;
            case 'z':
                roll_ = -100;
                break;
            case 'c':
                roll_ = 100;
                break;
            
            // Arm/Disarm
            case ' ':
                armed_ = !armed_;
                RCLCPP_WARN(this->get_logger(), "VEHICLE %s", armed_ ? "ARMED" : "DISARMED");
                break;
            
            // Mode changes
            case '1':
                current_mode_ = "MANUAL";
                RCLCPP_INFO(this->get_logger(), "Mode changed to MANUAL");
                break;
            case '2':
                current_mode_ = "ALT_HOLD";
                RCLCPP_INFO(this->get_logger(), "Mode changed to ALT_HOLD");
                break;
            case '3':
                current_mode_ = "STABILIZE";
                RCLCPP_INFO(this->get_logger(), "Mode changed to STABILIZE");
                break;
            
            // PWM conversion toggle
            case 'p':
                apply_pwm_conversion_ = !apply_pwm_conversion_;
                RCLCPP_INFO(this->get_logger(), "PWM conversion %s", apply_pwm_conversion_ ? "ENABLED" : "DISABLED");
                break;
            
            // Exit
            case 'x':
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                rclcpp::shutdown();
                break;
        }
        
        // Reset controls when key is released (simple decay)
        // In a more sophisticated implementation, you'd track key press/release
    }

    void publishThrusters() {
        // Decay control values (simulate key release)
        forward_ *= 0.8;
        lateral_ *= 0.8;
        thrust_ *= 0.8;
        yaw_ *= 0.8;
        pitch_ *= 0.8;
        roll_ *= 0.8;

        // Only update thruster values if armed
        if (armed_) {
            // Map control values to thrusters
            pwm_values_[0] = 1500 + forward_ + yaw_;      // Front right
            pwm_values_[1] = 1500 + forward_ - yaw_;      // Front left
            pwm_values_[2] = 1500 - forward_ + yaw_;      // Back right
            pwm_values_[3] = 1500 - forward_ - yaw_;      // Back left
            pwm_values_[4] = 1500 + thrust_ + pitch_;     // Vertical right
            pwm_values_[5] = 1500 + thrust_ - pitch_;     // Vertical left
            pwm_values_[6] = 1500 + lateral_ + roll_;     // Lateral right
            pwm_values_[7] = 1500 + lateral_ - roll_;     // Lateral left
            
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
            for (size_t i = 0; i < pwm_values_.size(); ++i) {
                thruster_msg.data[i] = (pwm_values_[i] - 1500) / 400;
            }
        } else {
            for (size_t i = 0; i < pwm_values_.size(); ++i) {
                thruster_msg.data[i] = pwm_values_[i];
            }
        }

        // Publish the message
        thruster_pub_->publish(thruster_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> pwm_values_;
    float sensitivity_all_axes_, sensitivity_yaw_, sensitivity_pitch_;
    bool armed_, apply_pwm_conversion_;
    std::string current_mode_;
    
    // Control states
    double forward_, lateral_, thrust_, yaw_, pitch_, roll_;
    
    // Terminal handling
    struct termios old_tio_, new_tio_;
    std::thread keyboard_thread_;
    bool running_;
};
