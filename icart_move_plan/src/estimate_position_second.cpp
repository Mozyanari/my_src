#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



class estimate_position_second{
public:
  estimate_position_second();

  //Listener定義
  tf::TransformListener listener;

private:
  //コールバック定義
  //一秒ごとにデバックするための関数
  void calc_estimate_position_second(const ros::TimerEvent&);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  ros::Publisher pub_estimate_position_second;

  //時間の関数作成
  ros::Timer timer;
};

estimate_position_second::estimate_position_second(){

  timer = nh.createTimer(ros::Duration(0.5), &estimate_position_second::calc_estimate_position_second,this);

  pub_estimate_position_second=nh.advertise<nav_msgs::Odometry>("/estimate_position_second", 1000);
}

void estimate_position_second::calc_estimate_position_second(const ros::TimerEvent&){
  nav_msgs::Odometry estimate_position_second;
  tf::StampedTransform transform;

  std::string mother_tf_frame = "map";
  std::string chile_tf_frame = "base_footprint_second_offset";

  try{
    listener.waitForTransform(mother_tf_frame,chile_tf_frame,ros::Time::now(),ros::Duration(1.0));
    listener.lookupTransform(mother_tf_frame,chile_tf_frame,ros::Time(0),transform);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  //transformからestimate_position_secondに変換
  estimate_position_second.pose.pose.position.x = transform.getOrigin().x();
  estimate_position_second.pose.pose.position.y = transform.getOrigin().y();
  estimate_position_second.pose.pose.position.z = tf::getYaw(transform.getRotation());
  //estimate_position_second.data=sqrt( (pow(transform.getOrigin().x(),2)) + (pow(transform.getOrigin().y(),2)) );
  //ROS_INFO("estimate_position_second.data %f",estimate_position_second);

  pub_estimate_position_second.publish(estimate_position_second);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "estimate_position_second");
	estimate_position_second estimate_position_second;
	ros::spin();
	return 0;
}
