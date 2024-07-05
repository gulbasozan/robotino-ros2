#ifndef AVATARNODE_H_
#define AVATARNODE_H_

#include "ComROS.h"
#include "SafeOmniDrive.h"

#include "rclcpp/rclcpp.hpp"

class SafeOmniDriveNode : public rclcpp::Node
{
    public:
        SafeOmniDriveNode();
        ~SafeOmniDriveNode();
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::string hostname_;
        double max_linear_vel_, min_linear_vel_, max_angular_vel_, min_angular_vel_;

        ComROS com_;
        SafeOmniDrive safe_omni_drive_;

        void initModules();

    public:
        void spin();

};

#endif /* AVATARNODE_H_ */