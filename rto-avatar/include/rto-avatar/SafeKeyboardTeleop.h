/* These prevent the header file from being included multiple times, which can cause errors. */
#ifndef SAFEKEYBOARDTELEOP_H_
#define SAFEKEYBOARDTELEOP_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
/* For handling the keyboard input*/
#include <termios.h>
/* To prevent data races when accessing shared resources */
#include <mutex>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_SPACE 0x20

class SafeKeyboardTeleop : public rclcpp::Node 
{
    public:
        SafeKeyboardTeleop(termios &cooked, termios &raw, int &kfd);
        ~SafeKeyboardTeleop();

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
        rclcpp::Time first_publish_, last_publish_;

        geometry_msgs::msg::Twist safe_cmd_vel_msg_;

        double scale_linear_, scale_angular_;
        struct termios cooked_, raw_;
        int kfd_;

        std::mutex publish_mutex_;

        void readParams();
        void publish(double vel_x, double vel_y, double vel_omega);

    public:
        void spin();
        void watchdog();

};

#endif /* SAFEKEYBOARDTELEOP_H_ */