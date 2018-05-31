//#include <Leap.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <chapter04/leap_motion.h>

class LeapTurtle
{
    public: 
	LeapTurtle();

    private:
	void leapCallback(const chapter04::leap_motion::ConstPtr& msg);
	ros::NodeHandle nh;
	ros::Publisher  pub;
	ros::Subscriber sub;
	int vel_linear, vel_angular;
};

LeapTurtle::LeapTurtle(): vel_linear(3), vel_angular(0)
{
	sub = nh.subscribe<chapter04::leap_motion>(
//		"/my_tutorials/hands_motion", 10, &LeapTurtle::leapCallback, this);
		"/hands_motion", 10, &LeapTurtle::leapCallback, this);
	pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void LeapTurtle::leapCallback(const chapter04::leap_motion::ConstPtr& msg)
{
	std::cout << "frame id: " << msg->header.frame_id << std::endl;
	std::cout << "tiemstamp: " << msg->header.stamp << std::endl;
	std::cout << "seq: " << msg->header.seq << std::endl;
	std::cout << "hand id: " << msg->hand_id << std::endl;
	std::cout << "direction: \n" << msg->direction << std::endl;
	std::cout << "normal: \n" << msg->normal << std::endl;
	std::cout << "velocity: \n" << msg->velocity << std::endl;
	std::cout << "palmpos: \n" << msg->palmpos << std::endl;
	std::cout << "ypr: \n" << msg->ypr << std::endl;
	std::cout << "--------------" << std::endl;

	geometry_msgs::Twist twist;
	twist.linear.x  = -0.15*msg->ypr.x;
	twist.angular.z = 0.15*msg->ypr.y;
	ROS_INFO_STREAM("(" << msg->ypr.x << " " << msg->ypr.y << ")");

	pub.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leap_turtle_teleop");
	LeapTurtle leap_turtle;

	ros::spin();
	return 0;
}

