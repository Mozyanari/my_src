#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>

class data_create{
public:
  data_create();
private:
  //機体からのオドメトリ取得
  void cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg);
  void cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg);
  //距離取得
  void cb_tf_distance(const std_msgs::Float32 &distance);
  //目標位置を取得
  void cb_target_point(const nav_msgs::Odometry::ConstPtr &position);

  //一秒ごとにデバックするための関数
  void pub_send_data(const ros::TimerEvent&);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  ros::Subscriber sub_odom_first;
  ros::Subscriber sub_odom_second;
  ros::Subscriber sub_tf_distance;
  ros::Subscriber sub_target_point;

  ros::Publisher pub_data;

  //時間の関数作成
  ros::Timer timer;

  //first
  nav_msgs::Odometry odom_first;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_first;
  double world_offset_position_y_first;

  //second
  nav_msgs::Odometry odom_second;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_second;
  double world_offset_position_y_second;

  //contorl
  nav_msgs::Odometry odom_control;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_control;
  double world_offset_position_y_control;


  //tf_distance
  double tf_distance;

  //target_point
  double target_point_x;
  double target_point_y;
  double target_point_yaw;

  //オフセット距離
  double s;
  //機体間距離[m]
  double distance_multi;

};

data_create::data_create(){
  //購読するトピックの定義
  sub_odom_first = nh.subscribe("/ypspur_ros_first/odom", 5, &data_create::cb_odom_first,this);
  sub_odom_second = nh.subscribe("/ypspur_ros_second/odom", 5, &data_create::cb_odom_second,this);
  sub_tf_distance = nh.subscribe("/tf_distance", 5, &data_create::cb_tf_distance,this);
  sub_target_point = nh.subscribe("/target_point", 5, &data_create::cb_target_point,this);


  //配布するトピックの定義
  pub_data = nh.advertise<nav_msgs::Odometry>("/send_data", 1000);

  //timer定義
  timer = nh.createTimer(ros::Duration(0.1), &data_create::pub_send_data,this);

  //オフセット距離[m]160mm
  s = 0.16;
  //機体間距離[m]570mm
  distance_multi = 0.57;
}

//関数定義-----------------------------------------------------------------------
//cb_odom_first関数定義
//機体のオドメトリデータの取得
void data_create::cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg){
  //first機体の状態受信
  odom_first = *msg;
  //first機体に関する計算----------------------------------------------------------
  //first機体の位置を代入
  double position_x_first = odom_first.pose.pose.position.x;
  double position_y_first = odom_first.pose.pose.position.y;
  //first機体の角度を代入
  double rad_first = tf::getYaw(odom_first.pose.pose.orientation);

  //first機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_first = (position_x_first + s + distance_multi) - (s * cos(rad_first));
  world_offset_position_y_first = position_y_first - (s * sin(rad_first));
}

//cb_odom_second関数定義
//機体のオドメトリデータの取得
void data_create::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg){
  //second機体の状態受信
  odom_second = *msg;
  //second機体に関する計算----------------------------------------------------------
  //second機体の位置を代入
  double position_x_second = odom_second.pose.pose.position.x;
  double position_y_second = odom_second.pose.pose.position.y;
  //second機体の角度を代入
  double rad_second = tf::getYaw(odom_second.pose.pose.orientation);

  //second機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_second = (position_x_second + s) - (s * cos(rad_second));
  world_offset_position_y_second = position_y_second - (s * sin(rad_second));
}

//cb_tf_distance
void data_create::cb_tf_distance(const std_msgs::Float32 &distance){
  tf_distance = distance.data;
}

void data_create::cb_target_point(const nav_msgs::Odometry::ConstPtr &position){
  nav_msgs::Odometry point = *position;
  geometry_msgs::Quaternion data_quaternion;

  target_point_x = point.pose.pose.position.x;
  target_point_y = point.pose.pose.position.y;

  data_quaternion.x = point.pose.pose.orientation.x;
  data_quaternion.y = point.pose.pose.orientation.y;
  data_quaternion.z = point.pose.pose.orientation.z;
  data_quaternion.w = point.pose.pose.orientation.w;
  target_point_yaw = tf::getYaw(data_quaternion);
}

void data_create::pub_send_data(const ros::TimerEvent&){
  world_offset_position_x_control = (world_offset_position_x_first + world_offset_position_x_second)/2.0;
  world_offset_position_y_control = (world_offset_position_y_first + world_offset_position_y_second)/2.0;

  odom_control.pose.pose.position.x = world_offset_position_x_control;
  odom_control.pose.pose.position.y = world_offset_position_y_control;
  odom_control.pose.pose.position.z = tf_distance;

  odom_control.twist.twist.linear.x = target_point_x;
  odom_control.twist.twist.linear.y = target_point_y;
  odom_control.twist.twist.linear.z = target_point_yaw;


  pub_data.publish(odom_control);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_create");
	data_create data_create;
	ros::spin();
	return 0;
}
