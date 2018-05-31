#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



class estimate_position_first{
public:
  estimate_position_first();

  //Listener定義
  tf::TransformListener listener;

private:
  //コールバック定義
  //一秒ごとにデバックするための関数
  void calc_estimate_position_first(const ros::TimerEvent&);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  ros::Publisher pub_estimate_position_first;

  //時間の関数作成
  ros::Timer timer;
};

estimate_position_first::estimate_position_first(){

  timer = nh.createTimer(ros::Duration(0.5), &estimate_position_first::calc_estimate_position_first,this);

  pub_estimate_position_first=nh.advertise<nav_msgs::Odometry>("/estimate_position_first", 1000);
}

void estimate_position_first::calc_estimate_position_first(const ros::TimerEvent&){
  nav_msgs::Odometry estimate_position_first;
  tf::StampedTransform transform;

  std::string mother_tf_frame = "map";
  std::string chile_tf_frame = "base_footprint_first_offset";

  try{
    listener.waitForTransform(mother_tf_frame,chile_tf_frame,ros::Time::now(),ros::Duration(1.0));
    listener.lookupTransform(mother_tf_frame,chile_tf_frame,ros::Time(0),transform);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  //transformからestimate_position_firstに変換
  estimate_position_first.pose.pose.position.x = transform.getOrigin().x();
  estimate_position_first.pose.pose.position.y = transform.getOrigin().y();
  estimate_position_first.pose.pose.position.z = tf::getYaw(transform.getRotation());
  //estimate_position_first.data=sqrt( (pow(transform.getOrigin().x(),2)) + (pow(transform.getOrigin().y(),2)) );
  //ROS_INFO("estimate_position_first.data %f",estimate_position_first);

  pub_estimate_position_first.publish(estimate_position_first);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "estimate_position_first");
	estimate_position_first estimate_position_first;
	ros::spin();
	return 0;
}
