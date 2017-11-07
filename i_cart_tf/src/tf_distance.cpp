#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>



class tf_distance{
public:
  tf_distance();

  //Listener定義
  tf::TransformListener listener;

private:
  //コールバック定義
  //一秒ごとにデバックするための関数
  void calc_tf_distance(const ros::TimerEvent&);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  ros::Publisher pub_tf_distance;

  //時間の関数作成
  ros::Timer timer;
};

tf_distance::tf_distance(){

  timer = nh.createTimer(ros::Duration(1.0), &tf_distance::calc_tf_distance,this);

  pub_tf_distance=nh.advertise<std_msgs::Float32>("/tf_distance", 1000);

  nh.setParam("mother_tf_frame","base_link_first_offset");
  nh.setParam("chile_tf_frame","base_link_second_offset");
}

void tf_distance::calc_tf_distance(const ros::TimerEvent&){
  std_msgs::Float32 tf_distance;
  tf::StampedTransform transform;

  std::string mother_tf_frame;
  std::string chile_tf_frame;

  ros::param::get("mother_tf_frame",mother_tf_frame);
  ros::param::get("chile_tf_frame",chile_tf_frame);

  try{
    listener.waitForTransform(mother_tf_frame,chile_tf_frame,ros::Time::now(),ros::Duration(5.0));
    listener.lookupTransform(mother_tf_frame,chile_tf_frame,ros::Time(0),transform);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf_distance.data=sqrt( (pow(transform.getOrigin().x(),2)) + (pow(transform.getOrigin().y(),2)) );
  ROS_INFO("tf_distance.data %f",tf_distance);

  pub_tf_distance.publish(tf_distance);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_distance");
	tf_distance tf_distance;
	ros::spin();
	return 0;
}
