#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

class offset_tf_broadcast{
public:
  offset_tf_broadcast();


private:
  //コールバック関数定義
  void cb_offset_odom(const nav_msgs::Odometry::ConstPtr &msg);

  //ノードハンドラ定義
  ros::NodeHandle nh;

  //配布するトピックの定義
  ros::Subscriber sub_odom;

  //機体パラメータ
  //オフセット距離
  double s;

  //std::string base_link;
  //std::string offset_base_link;


};

offset_tf_broadcast::offset_tf_broadcast(){
  //オフセット距離[m]160mm
  s=0.16;

  //パラメータ設定
  //ros::param::set("mother_frame","base_link");
  //ros::param::set("chile_frame", "offset_base_link");

  nh.setParam("mother_frame","base_link");
  nh.setParam("chile_frame","offset_base_link");


  //subscribe_define
  sub_odom = nh.subscribe("/ypspur_ros/odom", 5, &offset_tf_broadcast::cb_offset_odom,this);
}

//コールバック関数
void offset_tf_broadcast::cb_offset_odom(const nav_msgs::Odometry::ConstPtr &msg){
  std::string mother_frame;
  std::string chile_frame;

  //ブロードキャスター生成
  static tf::TransformBroadcaster br;
  //送信データ生成
  tf::Transform transform;
  //位置情報をセット
  transform.setOrigin(tf::Vector3(-s, 0.0, 0.0) );
  //クオータニオンをセット
  transform.setRotation(tf::Quaternion(0.0,0.0,0.0));
  //
  ROS_INFO("OK");
  ros::param::get("mother_frame",mother_frame);
  ros::param::get("chile_frame",chile_frame);

  //データを送信
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),mother_frame,chile_frame));
}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "offset_tf_broadcast");
	offset_tf_broadcast offset_tf_broadcast;



	ros::spin();
	return 0;
}
