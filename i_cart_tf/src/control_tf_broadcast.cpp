#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class control_tf_broadcast{
public:
  control_tf_broadcast();
  //Listener定義
  tf::TransformListener listener_first;

private:
  //コールバック関数定義
  void cb_offset_first_odom(const tf2_msgs::TFMessage &msg);
  //void cb_offset_second_odom(const nav_msgs::Odometry::ConstPtr &msg);

  //ノードハンドラ定義
  ros::NodeHandle nh;




  //配布するトピックの定義
  ros::Subscriber sub_odom_first;
  ros::Subscriber sub_odom_second;

};

control_tf_broadcast::control_tf_broadcast(){
  //subscribe_define
  sub_odom_first = nh.subscribe("/tf", 5, &control_tf_broadcast::cb_offset_first_odom,this);
}

//コールバック関数
void control_tf_broadcast::cb_offset_first_odom(const tf2_msgs::TFMessage &msg){

  tf::StampedTransform transform;
  try{
    listener_first.lookupTransform("/base_link","/odom",ros::Time(0),transform);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("%f",double(transform.getOrigin().y()));
}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_tf_broadcast");
	control_tf_broadcast control_tf_broadcast;
	ros::spin();
	return 0;
}
