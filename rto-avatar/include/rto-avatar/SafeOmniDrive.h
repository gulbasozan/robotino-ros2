#ifndef SAFEOMNIDRIVE_H_
#define SAFEOMNIDRIVE_H_

#include "rec/robotino/api2/OmniDrive.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <mutex>

using namespace std;

class SafeOmniDrive: public rec::robotino::api2::OmniDrive
{
    public:
        SafeOmniDrive(rclcpp::Node* parent_node);
        ~SafeOmniDrive();

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr distance_sensors_sub_;

        std::string parent_node_name_;
        sensor_msgs::msg::PointCloud distance_sensor_readings_;
        std::mutex distance_sensor_mutex_;

        double max_linear_vel_,
             min_linear_vel_,
             max_angular_vel_,
             min_angular_vel_,
             safety_radius_;

        void safeCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void distanceSensorsCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
        bool isInsideSafetyBubble(double point_x, double point_y, double safety_radius);
        string boolToString(bool x);

    public:
        void setMaxMin( double max_linear_vel, double min_linear_vel,
                    double max_angular_vel, double min_angular_vel );

};

#endif /* SAFEOMNIDRIVE_H_ */