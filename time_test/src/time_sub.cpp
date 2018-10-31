#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#include <std_msgs/Float32.h>


class time_sub{
public:
  time_sub();
private:
  //コールバック関数
  void sub_future_time(const geometry_msgs::PoseStamped::ConstPtr &msg);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  //pub,sub定義
  ros::Subscriber sub_time;
  ros::Publisher pub_time;


  geometry_msgs::PoseStamped target_point;
};

//コンストラクタ
time_sub::time_sub(){
  sub_time = nh.subscribe("/future_posi", 5, &time_sub::sub_future_time,this);
  pub_time = nh.advertise<std_msgs::Float32>("time",1,true);

}

//関数定義-----------------------------------------------------------------------
void time_sub::sub_future_time(const geometry_msgs::PoseStamped::ConstPtr &msg){
  target_point = *msg;
  double future_time = (target_point.header.stamp - ros::Time::now()).toSec();
  //デバック出力
  std_msgs::Float32 time;
  time.data = future_time;

  pub_time.publish(time);

  ROS_INFO("time=%f",future_time);
  //ros::Time future_time = ros::Time::now();

}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "time_sub");
	time_sub time_sub;
	ros::spin();
	return 0;
}
