#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

class hmd_rotation{
    public:
    hmd_rotation();
    private:
    void cb_hmd_rotation(const geometry_msgs::PoseStamped::ConstPtr &data);

    ros::NodeHandle nh;

    ros::Subscriber sub_hmd_rotation;
    ros::Publisher pub_pan_rotation;
    ros::Publisher pub_tilt_rotation;
};

hmd_rotation::hmd_rotation(){
    sub_hmd_rotation = nh.subscribe("/head_pose",5,&hmd_rotation::cb_hmd_rotation,this);
    pub_pan_rotation = nh.advertise<std_msgs::Float64>("/icart_depth/pan_joint_controller/command",1000);
    pub_tilt_rotation = nh.advertise<std_msgs::Float64>("/icart_depth/tilt_joint_controller/command",1000);
}

void hmd_rotation::cb_hmd_rotation(const geometry_msgs::PoseStamped::ConstPtr &data){
    std_msgs::Float64 rad_roll;
    std_msgs::Float64 rad_pitch;
    std_msgs::Float64 rad_yaw;
    //クオータニオンをroll,pitch,yawに分解する
    tf::Quaternion quat(data->pose.orientation.x, data->pose.orientation.y, data->pose.orientation.z, data->pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(rad_roll.data,rad_pitch.data,rad_yaw.data);
    ROS_INFO("roll=%f pitch=%f yaw=%f",rad_roll.data,rad_pitch.data,rad_yaw.data);
    //ROSとUnityの座標のとり方が異なるため，panにはyaw，tiltにはrollのデータを送信
    //rollは上下が反対のため負にする
    rad_roll.data = -rad_roll.data;
    pub_pan_rotation.publish(rad_yaw);
    pub_tilt_rotation.publish(rad_roll);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "hmd_rotation");
	hmd_rotation hmd_rotation;
	ros::spin();
	return 0;
}