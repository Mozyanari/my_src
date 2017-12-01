#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class control_tf_broadcast{
public:
  control_tf_broadcast();
  //Listener定義
  tf::TransformListener listener;

private:
  //コールバック関数定義
  void control_pub(const ros::TimerEvent&);

  //ノードハンドラ定義
  ros::NodeHandle nh;

  //配布するトピックの定義
  ros::Subscriber sub_odom_first;
  ros::Subscriber sub_odom_second;

  //時間の関数作成
  ros::Timer timer;

};

control_tf_broadcast::control_tf_broadcast(){
  //timer_define
  timer = nh.createTimer(ros::Duration(0.001), &control_tf_broadcast::control_pub,this);
}

//コールバック関数
void control_tf_broadcast::control_pub(const ros::TimerEvent&){
  tf::StampedTransform transform_first;
  tf::StampedTransform transform_second;

  //worldから見たbase_link_first_offsetの位置取得
  //wait_until_trans
  try{
    listener.waitForTransform("/world","/base_link_first_offset",ros::Time::now(),ros::Duration(5.0));
    //trans_to_offset_cloud
    listener.lookupTransform("/world","/base_link_first_offset",ros::Time(0),transform_first);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  //worldから見たbase_link_second_offsetの位置取得
  //wait_until_trans
  try{
    listener.waitForTransform("/world","/base_link_second_offset",ros::Time::now(),ros::Duration(5.0));
    //trans_to_offset_cloud
    listener.lookupTransform("/world","/base_link_second_offset",ros::Time(0),transform_second);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }




  //firstとsecondの中間点をcontrol_pointとして計算
  double world_control_point_x=0;
  double world_control_point_y=0;
  double world_control_point_rad=0;

  world_control_point_x = (transform_first.getOrigin().x() + transform_second.getOrigin().x())/2;
  world_control_point_y = (transform_first.getOrigin().y() + transform_second.getOrigin().y())/2;

  world_control_point_rad = atan2(transform_first.getOrigin().y()-transform_second.getOrigin().y(),transform_first.getOrigin().x() - transform_second.getOrigin().x());


  //tfとしてcontrol_pointをpublish
  //ブロードキャスター生成
  static tf::TransformBroadcaster br;
  //送信データ生成
  tf::Transform transform_control;
  //位置情報をセット
  transform_control.setOrigin(tf::Vector3(world_control_point_x,world_control_point_y, 0.0) );
  //クオータニオンをセット
  tf::Quaternion q;
  q.setRPY(0,0,world_control_point_rad);
  transform_control.setRotation(q);

  //データを送信
  br.sendTransform(tf::StampedTransform(transform_control, ros::Time::now(),"world","control_point"));


  //ROS_INFO("world_control_point_x %f",world_control_point_x);
  //ROS_INFO("world_control_point_y %f",world_control_point_y);
  //ROS_INFO("world_control_point_rad %f",world_control_point_rad);


}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_tf_broadcast");
	control_tf_broadcast control_tf_broadcast;
	ros::spin();
	return 0;
}
