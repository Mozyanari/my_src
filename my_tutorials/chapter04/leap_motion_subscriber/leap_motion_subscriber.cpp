//#include <Leap.h>
#include <ros/ros.h>
#include <chapter04/leap_motion.h>

void callback(const chapter04::leap_motion::ConstPtr& msg)
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
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "leap_motion_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<chapter04::leap_motion>(
//		"/my_tutorials/hands_motion", 10, callback);
		"/hands_motion", 10, callback);

    ros::spin();
    return 0;
}
