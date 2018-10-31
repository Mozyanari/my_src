#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



class offset_position{
public:
  offset_position();
private:
  //コールバック定義
  void calc_offset_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  ros::Publisher pub_offset_position;
  ros::Subscriber sub_amcl_pose;

  //オフセット距離[m]
  double s;
};

//コンストラクタ定義
offset_position::offset_position(){
  //Pub,Sub定義
  sub_amcl_pose = nh.subscribe("amcl_pose", 5, &offset_position::calc_offset_position,this);
  pub_offset_position = nh.advertise<geometry_msgs::Pose2D>("offset_position", 1000, true);

  //オドメトリ位置からオフセットまでの距離[m]
  s = 0.16;
}

//オフセット位置計算
void offset_position::calc_offset_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data){
  //オフセット位置
  geometry_msgs::Pose2D offset_position;

  //amcl_poseからoffset_positionに変換
  offset_position.theta = tf::getYaw(data->pose.pose.orientation);
  offset_position.x = data->pose.pose.position.x - (s * cos(offset_position.theta));
  offset_position.y = data->pose.pose.position.y - (s * sin(offset_position.theta));
  //offset_position.pose.pose.position.z = tf::getYaw(data.pose.pose.orientation);
  //offset_position.data=sqrt( (pow(transform.getOrigin().x(),2)) + (pow(transform.getOrigin().y(),2)) );
  //ROS_INFO("offset_position.data %f",offset_position);

  pub_offset_position.publish(offset_position);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "offset_position");
	offset_position offset_position;
	ros::spin();
	return 0;
}
