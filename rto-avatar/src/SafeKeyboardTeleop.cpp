#include "SafeKeyboardTeleop.h"
#include <unistd.h>
#include <cstring>
#include <thread>

SafeKeyboardTeleop::SafeKeyboardTeleop(struct termios &cooked, struct termios &raw, int &kfd)
    : Node("safe_keyboard_teleop"), cooked_(cooked), raw_(raw), kfd_(kfd)
{
    safe_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rto3/safe_cmd_vel", 1);
    readParams();
    first_publish_ = this->get_clock()->now();
    last_publish_ = this->get_clock()->now();
}

/* Restore terminal setting when object is destroyed */
SafeKeyboardTeleop::~SafeKeyboardTeleop()
{
    tcsetattr(kfd_, TCSANOW, &cooked_);
}

void SafeKeyboardTeleop::readParams()
{
    this->declare_parameter<double>("scale_linear", 1.0);
    this->declare_parameter<double>("scale_angular", 1.0);
    this->get_parameter("scale_linear", scale_linear_);
    this->get_parameter("scale_angular", scale_angular_);
} 

void SafeKeyboardTeleop::publish(double vel_x, double vel_y, double vel_omega)
{
    safe_cmd_vel_msg_.linear.x = scale_linear_ * vel_x;
    safe_cmd_vel_msg_.linear.y = scale_linear_ * vel_y;
    safe_cmd_vel_msg_.angular.z = scale_angular_ * vel_omega;

    safe_cmd_vel_pub_->publish(safe_cmd_vel_msg_);
}

void SafeKeyboardTeleop::spin()
{
    char c;
    double vel_x, vel_y, vel_omega;

    // Get the console in raw mode
    tcgetattr(kfd_, &cooked_);
    memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &= ~(ICANON | ECHO);

    // Setting a new line, then end of file
    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    tcsetattr(kfd_, TCSANOW, &raw_);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'WASD' for translation");
    puts("Use 'QE' for rotation");
    puts("Press 'Space' to STOP");

    while (rclcpp::ok())
    {
        vel_x = 0.0;
        vel_y = 0.0;
        vel_omega = 0.0;

        // get the next event from the keyboard
        if (read(kfd_, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch (c)
        {
        // Walking
        case KEYCODE_W:
            vel_x = 0.05;
            break;
        case KEYCODE_S:
            vel_x = -0.05;
            break;
        case KEYCODE_A:
            vel_y = 0.05;
            break;
        case KEYCODE_D:
            vel_y = -0.05;
            break;
        case KEYCODE_Q:
            vel_omega = 0.5;
            break;
        case KEYCODE_E:
            vel_omega = -0.5;
            break;
        case KEYCODE_SPACE:
            vel_x = vel_y = vel_omega = 0.0;
            break;
        default:
            break;
        }

        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (this->get_clock()->now() > last_publish_ + rclcpp::Duration(1, 0))
        {
            first_publish_ = this->get_clock()->now();
        }
        last_publish_ = this->get_clock()->now();
        publish(vel_x, vel_y, vel_omega);
    }
}

void SafeKeyboardTeleop::watchdog()
{
    std::lock_guard<std::mutex> lock(publish_mutex_);
    if ((this->get_clock()->now() > last_publish_ + rclcpp::Duration(0, 150000000)) &&
        (this->get_clock()->now() > first_publish_ + rclcpp::Duration(0, 500000000)))
    {
        publish(0.0, 0.0, 0.0);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    struct termios cooked;
    struct termios raw;
    int kfd = 0;

    auto node = std::make_shared<SafeKeyboardTeleop>(cooked, raw, kfd);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() { executor.spin();});

    node->spin();

    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}