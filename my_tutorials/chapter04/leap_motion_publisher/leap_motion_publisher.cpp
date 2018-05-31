#include <Leap.h>
#include <ros/ros.h>
#include <chapter04/leap_motion.h>

using namespace Leap;

class HandsListener : public Listener {
    public:
	ros::NodeHandle nh;
	ros::Publisher  pub;

	/*
	 * Called once, when this Listener object is newly added to a Controller.
	 */
	virtual void onInit(const Controller&);

	/* 
	 * Called when the Controller object connects to the Leap Motion software 
 	 * and the Leap Motion hardware device is plugged in, or when this Listener 
	 * object is added to a Controller that is already connected.
	 */
	virtual void onConnect(const Controller&);

	/*
	 * Called when the Controller object disconnects from the Leap Motion software 
	 * or the Leap Motion hardware is unplugged.
	 */
	virtual void onDisconnect(const Controller&){ROS_DEBUG("Disconnected");};

	/*
	 * Called when this Listener object is removed from the Controller or the 
	 * Controller instance is destroyed.
	 */
	virtual void onExit(const Controller&){ROS_DEBUG("Exited");};

	/*
	 * Called when a new frame of hand and finger tracking data is available.
	 */
	virtual void onFrame(const Controller&);

	/*
	 * Called when this application becomes the foreground application.
	 */
	virtual void onFocusGained(const Controller&) {ROS_DEBUG("Focus Gained");};

	/*
	 * Called when this application loses the foreground focus.
	 */
	virtual void onFocusLost(const Controller&){ROS_DEBUG("Focus Lost");};

	/*
	 * Called when a Leap Motion controller plugged in, unplugged, or the device changes state.
	 */
	virtual void onDeviceChange(const Controller&){ROS_DEBUG("Device Changed");};

	/*
	 * Called when the Leap Motion daemon/service connects to your application Controller.
	 */
	virtual void onServiceConnect(const Controller&){ROS_DEBUG("Service Connected");};

	/*
	 * Called if the Leap Motion daemon/service disconnects from your application Controller.
	 */
	virtual void onServiceDisconnect(const Controller&){ROS_DEBUG("Service Disconnected");};
    private:
};

void HandsListener::onInit(const Controller& controller) 
{
	// topic name: /hands_motion
	// message type: chapter04::leap_motion
	std::cout << "Initialized" << std::endl;
	//pub = nh.advertise<chapter04::leap_motion>("my_tutorials/hands_motion", 1);
	pub = nh.advertise<chapter04::leap_motion>("hands_motion", 1);
}

void HandsListener::onConnect(const Controller& controller) 
{
	std::cout << "Connected" << std::endl;
	controller.enableGesture(Gesture::TYPE_CIRCLE);
	controller.enableGesture(Gesture::TYPE_KEY_TAP);
	controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
	controller.enableGesture(Gesture::TYPE_SWIPE);
}

void HandsListener::onFrame(const Controller& controller) 
{
	// Get the most recent frame and report some basic information
	const Frame frame = controller.frame();
	chapter04::leap_motion msg;

	msg.header.frame_id = "leap_motion_pub";
	msg.header.stamp = ros::Time::now();

	HandList hands = frame.hands();
	const Hand hand = hands[0];

	Vector normal = hand.palmNormal();
	Vector direction = hand.direction();
	Vector velocity = hand.palmVelocity();
	Vector position = hand.palmPosition();

	msg.hand_id = hand.id();

	msg.direction.x = direction[0];
	msg.direction.y = direction[1];
	msg.direction.z = direction[2];

	msg.normal.x = normal[0];
	msg.normal.y = normal[1];
	msg.normal.z = normal[2];

	msg.velocity.x = velocity[0];
	msg.velocity.y = velocity[1];
	msg.velocity.z = velocity[2];

	msg.palmpos.x = position[0];
	msg.palmpos.y = position[1];
	msg.palmpos.z = position[2];

	msg.ypr.x = direction.pitch() * RAD_TO_DEG ;
	msg.ypr.y = normal.roll() * RAD_TO_DEG;
	msg.ypr.z = direction.yaw() * RAD_TO_DEG;
	
	std::cout << "      Hand ID:" << hand.id() << std::endl;
	std::cout << " PalmPosition:" << hand.palmPosition() << std::endl;
	std::cout << " PalmVelocity:" << hand.palmVelocity() << std::endl;
	std::cout << "   PalmNormal:" << hand.palmNormal() << std::endl;
	std::cout << "PalmDirection:" << hand.direction() << std::endl;
	std::cout << " pitch:" << msg.ypr.x << std::endl;
	std::cout << "  roll:" << msg.ypr.y << std::endl;
	std::cout << "   yaw:" << msg.ypr.z << std::endl;
	std::cout << "--------------" << std::endl;

	pub.publish(msg);
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "leap_motion_publisher");

	// Create a sample listener and controller
	HandsListener listener;
	Controller controller;
  
	// Have the sample listener receive events from the controller
	controller.addListener(listener);
	controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));

	ros::spin();

	// Remove the sample listener when done
	controller.removeListener(listener);
	return 0;
}
