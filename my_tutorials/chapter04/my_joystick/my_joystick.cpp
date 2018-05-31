#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopTurtle
{
  public:
	TeleopTurtle();

  private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh;
	int vel_linear, vel_angular;
	double l_scale_, a_scale_;
	ros::Publisher  vel_pub_;
	ros::Subscriber joy_sub_;
};

TeleopTurtle::TeleopTurtle(): vel_linear(3), vel_angular(0), a_scale_(3.2),l_scale_(2.0)
{
	nh.param("axis_linear"  , vel_linear , vel_linear);
	nh.param("axis_angular" , vel_angular, vel_angular);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear" , l_scale_, l_scale_);

	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
	vel_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.linear.x = l_scale_*joy->axes[vel_linear];
	twist.angular.z = a_scale_*joy->axes[vel_angular];
	ROS_INFO_STREAM("(" << joy->axes[vel_linear] << " " << joy->axes[vel_angular] << ")");
	vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_turtle");
	TeleopTurtle teleop_turtle;

	ros::spin();
	return 0;
}

