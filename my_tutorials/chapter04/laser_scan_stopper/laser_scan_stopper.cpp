#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class ScanStopper {
    public:
	// Tunable parameters
	const static double FORWARD_SPEED_MPS = 0.5;
	const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;

	// Should be smaller than sensor_msgs::LaserScan::range_max
	const static float  MIN_PROXIMITY_RANGE_M = 0.1; 

	ScanStopper();
	void startMoving();

    private:
	ros::NodeHandle nh;
	// Publisher to the robot's velocity command topic
	ros::Publisher cmd_pub; 
	// Subscriber to the robot's laser scan topic
	ros::Subscriber laser_sub; 
	// Indicates whether the robot should continue moving
	bool keepMoving; 

	void moveForward();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

ScanStopper::ScanStopper()
{
	keepMoving = true;

	// Advertise a new publisher for the simulated robot's velocity command topic
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// Subscribe to the simulated robot's laser scan topic
	laser_sub = nh.subscribe("base_scan", 1, &ScanStopper::scanCallback, this);
}

// Send a velocity command
void 
ScanStopper::moveForward() {
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg; 
	msg.linear.x = FORWARD_SPEED_MPS;
	cmd_pub.publish(msg);
}

// Process the incoming laser scan message
void 
ScanStopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex =  ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M) {
		ROS_INFO("Stop!");
		keepMoving = false;
	}
}

void 
ScanStopper::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok() && keepMoving) {
		moveForward();

		// Need to call this function often to allow ROS to process incoming messages
		ros::spinOnce(); 
		rate.sleep();
	}
}

int 
main(int argc, char **argv) 
{
	// Initiate new ROS node named "stopper"
	ros::init(argc, argv, "stopper");

	// Create new stopper object
	ScanStopper stopper;

	// Start the movement
	stopper.startMoving();

	return 0;
}

