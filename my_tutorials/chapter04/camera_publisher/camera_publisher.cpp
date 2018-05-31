#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main( int argc, char **argv )
{
	ros::init( argc, argv, "image_Camerapublisher" );
	ros::NodeHandle n;

	// Open camera with CAMERA_INDEX (webcam is typically #0).
	const int CAMERA_INDEX = 0;
	cv::VideoCapture capture( CAMERA_INDEX );
	if( not capture.isOpened() ) {
		ROS_INFO_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
		ros::shutdown();
	}

	// Set the parameters using the results of executing the following command:
	//   v4l2-ctl -d /dev/video0 --list-formats-ext
 	capture.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    	capture.set(CV_CAP_PROP_FPS, 20);

	image_transport::ImageTransport it( n );
	image_transport::CameraPublisher pub_image = it.advertiseCamera( "camera_raw", 1 );
	sensor_msgs::CameraInfo cam_info;

	// for debug
	//cv::namedWindow("Image_CameraPublisher", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

	cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
	frame->encoding = sensor_msgs::image_encodings::BGR8;
	while( ros::ok() ) {
		// for debug
		//cv::Mat image;
		//capture >> image;
		//cv::imshow("Image_CameraPublisher", image);
		//

		capture >> frame->image;
		if( frame->image.empty() ) {
			ROS_ERROR_STREAM( "Failed to capture frame!" );
			ros::shutdown();
		}
		frame->header.stamp = ros::Time::now();
		cam_info.header.stamp = frame->header.stamp;
		pub_image.publish( frame->toImageMsg() , 
			sensor_msgs::CameraInfoConstPtr(new sensor_msgs::CameraInfo(cam_info)));

		if(cv::waitKey(30) >= 0) break;
		ros::spinOnce();
	}

	capture.release();
	return EXIT_SUCCESS;
}

