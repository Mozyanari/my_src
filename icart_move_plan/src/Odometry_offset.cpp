#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



class Odometry_offset{
public:
  Odometry_offset();
private:
  //コールバック定義
  void calc_Odometry_offset(const nav_msgs::Odometry::ConstPtr& data);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  ros::Publisher pub_Odometry_offset;
  ros::Subscriber sub_amcl_pose;

  //オフセット距離[m]
  double s;
};

//コンストラクタ定義
Odometry_offset::Odometry_offset(){
  //Pub,Sub定義
  sub_amcl_pose = nh.subscribe("pose_ground_truth", 5, &Odometry_offset::calc_Odometry_offset,this);
  pub_Odometry_offset = nh.advertise<geometry_msgs::Pose2D>("pose_offset_truth", 1000, true);

  //オドメトリ位置からオフセットまでの距離[m]
  s = 0.16;
}

//オフセット位置計算
void Odometry_offset::calc_Odometry_offset(const nav_msgs::Odometry::ConstPtr& data){
  //オフセット位置
  geometry_msgs::Pose2D Odometry_offset;

  //amcl_poseからOdometry_offsetに変換
  Odometry_offset.theta = tf::getYaw(data->pose.pose.orientation);
  Odometry_offset.x = data->pose.pose.position.x - (s * cos(Odometry_offset.theta));
  Odometry_offset.y = data->pose.pose.position.y - (s * sin(Odometry_offset.theta));
  //Odometry_offset.pose.pose.position.z = tf::getYaw(data.pose.pose.orientation);
  //Odometry_offset.data=sqrt( (pow(transform.getOrigin().x(),2)) + (pow(transform.getOrigin().y(),2)) );
  //ROS_INFO("Odometry_offset.data %f",Odometry_offset);

  pub_Odometry_offset.publish(Odometry_offset);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Odometry_offset");
	Odometry_offset Odometry_offset;
	ros::spin();
	return 0;
}
