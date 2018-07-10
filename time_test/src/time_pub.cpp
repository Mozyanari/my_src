#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


class time_pub{
public:
  time_pub();
private:
  //コールバック関数
  void send_future_time(const ros::TimerEvent&);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  //pub,sub定義
  ros::Publisher pub_time;
  //時間の関数作成
  ros::Timer timer;

  geometry_msgs::PoseStamped target_point;
};

//コンストラクタ
time_pub::time_pub(){
  pub_time = nh.advertise<geometry_msgs::PoseStamped>("/future_posi",1000);

  //timer定義
  timer = nh.createTimer(ros::Duration(0.001), &time_pub::send_future_time,this);
}

//関数定義-----------------------------------------------------------------------
void time_pub::send_future_time(const ros::TimerEvent&){
  target_point.pose.position.x = 1;
  target_point.pose.position.y = 2;

  target_point.header.stamp = ros::Time::now() + ros::Duration(10.0);

  //ros::Time future_time = ros::Time::now();
  pub_time.publish(target_point);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "time_pub");
	time_pub time_pub;
	ros::spin();
	return 0;
}
