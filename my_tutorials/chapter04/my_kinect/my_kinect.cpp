#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

class camera_proc
{
    public: 
	camera_proc(ros::NodeHandle node,ros::NodeHandle private_nh);   
	camera_proc(){};
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
    private:
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_; 
};

camera_proc::camera_proc (ros::NodeHandle node, ros::NodeHandle private_nh)
{
	image_transport::ImageTransport it_(node);
	image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 
			1, &camera_proc::image_callback,this) ;      
} //camera_proc's constructor

namespace enc = sensor_msgs::image_encodings;
void camera_proc::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::imshow("in image", cv_ptr->image);
	cv::waitKey(3);

	image_pub_.publish(cv_ptr->toImageMsg());
}

int main (int argc, char** argv)
{
	ros::init(argc, argv,"my_kinect_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");
	camera_proc class_object(nh, priv_nh);
	ros::spin();

	return 0;
}
