/*
 * KeyboardTeleop.h
 *
 *  Created on: 11.01.2012
 *      Author: indorewala@servicerobotics.eu
 */

/*
 * Functionality borrowed from turtlebot_teleop
 */

#ifndef KEYBOARDTELEOP_H_
#define KEYBOARDTELEOP_H_

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <mutex>

// ROS1
// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <termios.h>
// #include "boost/thread/mutex.hpp"

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_SPACE 0x20

// ROS2
class KeyboardTeleop : public rclcpp::Node
{
	public:
        KeyboardTeleop(termios &cooked, termios &raw, int &kfd);
        ~KeyboardTeleop();

    private:
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp::Time first_publish_, last_publish_;

		geometry_msgs::msg::Twist cmd_vel_msg_;

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

// ROS1
// class KeyboardTeleop
// {
// public:
// 	KeyboardTeleop( struct termios &cooked, struct termios &raw, int &kfd );
// 	~KeyboardTeleop();

// private:
// 	ros::NodeHandle nh_;
// 	ros::Publisher cmd_vel_pub_;
// 	ros::Time first_publish_, last_publish_;

// 	geometry_msgs::Twist cmd_vel_msg_;

// 	double scale_linear_, scale_angular_;
// 	struct termios cooked_, raw_;
// 	int kfd_;

// 	boost::mutex publish_mutex_;

// 	void readParams( ros::NodeHandle n );
// 	void publish( double vel_x, double vel_y, double vel_omega );

// public:
// 	void spin();
// 	void watchdog();
// };


#endif /* KEYBOARDTELEOP_H_ */
