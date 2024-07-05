#include "SafeOmniDriveNode.h"

using namespace std::chrono_literals;

SafeOmniDriveNode::SafeOmniDriveNode():
    Node("safe_omni_drive_node"),
    com_(this),
    safe_omni_drive_(this)
{
    this->declare_parameter("hostname","172.31.1.145");
	this->declare_parameter("max_linear_vel", 0.02);
	this->declare_parameter("min_linear_vel", 0.05);
	this->declare_parameter("max_angular_vel", 1.0);
	this->declare_parameter("min_angular_vel", 0.1);

    hostname_ = this->get_parameter("hostname").as_string();
	
	max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
	min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
	max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
	min_angular_vel_ = this->get_parameter("min_angular_vel").as_double();

    com_.setName("SafeOmniDrive");   

    initModules();
    timer_ = this->create_wall_timer(200ms, std::bind(&SafeOmniDriveNode::spin, this));

}

SafeOmniDriveNode::~SafeOmniDriveNode()
{}

void SafeOmniDriveNode::initModules()
{
    com_.setAddress(hostname_.c_str());

    safe_omni_drive_.setComId(com_.id());
    com_.connectToServer(false);
}

void SafeOmniDriveNode::spin()
{
    // rclcpp::Time curr_time = rclcpp::Clock().now();
    // safe_omni_drive_.setTimeStamp(curr_time);

    com_.processEvents();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeOmniDriveNode>());

    return 0;
}

