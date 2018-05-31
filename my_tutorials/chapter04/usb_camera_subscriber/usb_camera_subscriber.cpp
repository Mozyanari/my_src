#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		cv::imshow("camera_Subscriber", cv_bridge::toCvShare(msg, "bgr8")->image);
		if(cv::waitKey(30) >= 0)
			ros::shutdown();
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_subscriber");
	ros::NodeHandle nh;
	cv::namedWindow("camera_Subscriber");
	cv::startWindowThread();

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

	ros::spin();
	cv::destroyWindow("camera_Subscriber");

	return EXIT_SUCCESS;
}

