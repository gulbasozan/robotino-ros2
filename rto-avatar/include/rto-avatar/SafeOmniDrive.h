#ifndef SAFEOMNIDRIVE_H_
#define SAFEOMNIDRIVE_H_

#include "rec/robotino/api2/OmniDrive.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class SafeOmniDrive: public rec::robotino::api2::OmniDrive
{
    public:
        SafeOmniDrive(rclcpp::Node* parent_node);
        ~SafeOmniDrive();

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_sub_;

        std::string parent_node_name_;

        double max_linear_vel_;
        double min_linear_vel_;
        double max_angular_vel_;
        double min_angular_vel_;

        void safeCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    public:
        void setMaxMin( double max_linear_vel, double min_linear_vel,
                    double max_angular_vel, double min_angular_vel );

};

#endif /* SAFEOMNIDRIVE_H_ */