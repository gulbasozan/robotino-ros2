#include "SafeOmniDrive.h"

using std::placeholders::_1;

SafeOmniDrive::SafeOmniDrive(rclcpp::Node* parent_node) 
{
	parent_node_name_ = std::string(parent_node->get_name());
    safe_cmd_vel_sub_ = parent_node->create_subscription<geometry_msgs::msg::Twist>("rto3/safe_cmd_vel", 10, std::bind(&SafeOmniDrive::safeCmdVelCallback, this, _1));
}

SafeOmniDrive::~SafeOmniDrive()
{}


void SafeOmniDrive::safeCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_x = msg->linear.x;
	double linear_y = msg->linear.y;
	double angular = msg->angular.z;

	if ( fabs( linear_x ) > max_linear_vel_ )
	{
		if( linear_x > 0.0 )
			linear_x = max_linear_vel_;
		else
			linear_x = -max_linear_vel_;
	}
	else if( fabs( linear_x ) <  min_linear_vel_ && fabs( linear_x ) > 0.0 )
	{
		if( linear_x > 0.0 )
			linear_x = min_linear_vel_;
		else
			linear_x = -min_linear_vel_;
	}

	if ( fabs( linear_y ) > max_linear_vel_ )
	{
		if( linear_y > 0.0 )
			linear_y = max_linear_vel_;
		else
			linear_y = -max_linear_vel_;
	}
	else if( fabs( linear_y ) <  min_linear_vel_ && fabs( linear_y ) > 0.0 )
	{
		if( linear_y > 0.0 )
			linear_y = min_linear_vel_;
		else
			linear_y = -min_linear_vel_;
	}

	if ( fabs( angular ) > max_angular_vel_ )
	{
		if( angular > 0.0 )
			angular = max_angular_vel_;
		else
			angular = -max_angular_vel_;
	}
	else if( fabs( angular ) <  min_angular_vel_ && fabs( angular ) > 0.0 )
	{
		if( angular > 0.0 )
			angular = min_angular_vel_;
		else
			angular = -min_angular_vel_;
	}

	RCLCPP_INFO(rclcpp::get_logger(parent_node_name_), "Callback Function Triggered");
	setVelocity( linear_x, linear_y, angular);
}

void SafeOmniDrive::setMaxMin( double max_linear_vel, double min_linear_vel,
		double max_angular_vel, double min_angular_vel )
{
	max_linear_vel_ = max_linear_vel;
	min_linear_vel_ = min_linear_vel;
	max_angular_vel_ = max_angular_vel;
	min_angular_vel_ = min_angular_vel;
}
