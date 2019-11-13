#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class joy_to_twist{
    public:
    joy_to_twist();
    private:
    void cb_joy_to_twist(const sensor_msgs::Joy::ConstPtr &data);

    ros::NodeHandle nh;

    ros::Subscriber sub_joy;
    ros::Publisher pub_twist;
};

joy_to_twist::joy_to_twist(){
    sub_joy = nh.subscribe("/joy",5,&joy_to_twist::cb_joy_to_twist,this);
    pub_twist = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel",1000);
}

void joy_to_twist::cb_joy_to_twist(const sensor_msgs::Joy::ConstPtr &data){
    geometry_msgs::Twist vel;
    vel.linear.x = data->axes[1]*0.1;
    vel.angular.z = data->axes[3]*0.3;
    pub_twist.publish(vel);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_to_twist");
	joy_to_twist joy_to_twist;
	ros::spin();
	return 0;
}