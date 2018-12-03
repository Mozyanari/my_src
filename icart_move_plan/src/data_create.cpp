#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>


class data_create{
public:
  data_create();
private:
  //推定した機体のオフセット位置
  void cb_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg);
  void cb_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg);
  //シミュレーション上の真値のオフセット位置
  void cb_gazebo_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg);
  void cb_gazebo_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg);
  //目標位置を取得
  void cb_target_point(const geometry_msgs::Pose2D::ConstPtr &position);
  //機体の目標位置
  void cb_target_point_first(const nav_msgs::Odometry::ConstPtr &position);
  void cb_target_point_second(const nav_msgs::Odometry::ConstPtr &position);
  //機体の速度
  void cb_vel_first(const geometry_msgs::Twist::ConstPtr &speed);
  void cb_vel_second(const geometry_msgs::Twist::ConstPtr &speed);
  //連結リンクの真値
  void cb_block(const nav_msgs::Odometry::ConstPtr &position);


  //一秒ごとにデバックするための関数
  void pub_send_data(const ros::TimerEvent&);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  ros::Subscriber sub_odom_first;
  ros::Subscriber sub_odom_second;
  ros::Subscriber sub_gazebo_odom_first;
  ros::Subscriber sub_gazebo_odom_second;
  ros::Subscriber sub_tf_distance;
  ros::Subscriber sub_target_point;
  ros::Subscriber sub_target_point_first;
  ros::Subscriber sub_target_point_second;
  ros::Subscriber sub_speed_first;
  ros::Subscriber sub_speed_second;
  ros::Subscriber sub_block;


  ros::Publisher pub_data;

  //時間の関数作成
  ros::Timer timer;

  //first
  geometry_msgs::Pose2D odom_first;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_first;
  double world_offset_position_y_first;
  double world_offset_position_theta_first;

  //second
  geometry_msgs::Pose2D odom_second;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_second;
  double world_offset_position_y_second;
  double world_offset_position_theta_second;

  //contorl
  nav_msgs::Odometry odom_control;
  //現在の機体の世界座標における位置
  double world_offset_position_x_control;
  double world_offset_position_y_control;
  double world_offset_position_theta_control;

  //gazebo上の真値
  geometry_msgs::Pose2D gazebo_first;

  geometry_msgs::Pose2D gazebo_second;

  geometry_msgs::Pose2D gazebo_control;

  //blockの真値
  nav_msgs::Odometry block;

  //送信データ型
  geometry_msgs::PoseArray senddata;

  //tf_distance
  double tf_distance;

  //target_point
  double target_point_x;
  double target_point_y;
  double target_point_yaw;

  //target_point_first
  double target_point_first_x;
  double target_point_first_y;
  double target_point_first_yaw;
  double target_time_first;

  //target_point_second
  double target_point_second_x;
  double target_point_second_y;
  double target_point_second_yaw;
  double target_time_second;

  //first_speed
  double x_vel_first;
  double y_vel_first;
  double omega_r_first;
  double omega_l_first;
  double machine_x_first;
  double machine_z_first;

  //second_speed
  double x_vel_second;
  double y_vel_second;
  double omega_r_second;
  double omega_l_second;
  double machine_x_second;
  double machine_z_second;


  //オフセット距離
  double s;
  //機体間距離[m]
  double distance_multi;

};

data_create::data_create(){
  //購読するトピックの定義
  sub_odom_first = nh.subscribe("/first/offset_odom_true", 5, &data_create::cb_odom_first,this);
  sub_odom_second = nh.subscribe("/second/offset_odom_true", 5, &data_create::cb_odom_second,this);
  sub_gazebo_odom_first = nh.subscribe("/first/pose_offset_truth", 5, &data_create::cb_gazebo_odom_first,this);
  sub_gazebo_odom_second = nh.subscribe("/second/pose_offset_truth", 5, &data_create::cb_gazebo_odom_second,this);
  sub_target_point = nh.subscribe("/target_point", 5, &data_create::cb_target_point,this);
  sub_target_point_first = nh.subscribe("/first/target_point", 5, &data_create::cb_target_point_first,this);
  sub_target_point_second = nh.subscribe("/second/target_point", 5, &data_create::cb_target_point_second,this);
  sub_speed_first = nh.subscribe("/icart_first/diff_drive_controller/cmd_vel", 5, &data_create::cb_vel_first,this);
  sub_speed_second = nh.subscribe("/icart_second/diff_drive_controller/cmd_vel", 5, &data_create::cb_vel_second,this);
  sub_block = nh.subscribe("/block/pose_ground_truth", 5, &data_create::cb_block,this);



  //配布するトピックの定義
  pub_data = nh.advertise<geometry_msgs::PoseArray>("/send_data", 1000);

  //timer定義
  timer = nh.createTimer(ros::Duration(1.0), &data_create::pub_send_data,this);

  //オフセット距離[m]160mm
  s = 0.16;
  //機体間距離[m]570mm
  distance_multi = 0.57;

  //配列の初期化
  senddata.poses.clear();
}

//関数定義-----------------------------------------------------------------------
//cb_odom_first関数定義
//機体のオドメトリデータの取得
void data_create::cb_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg){
  //first機体の状態受信
  odom_first = *msg;
  //first機体に関する計算----------------------------------------------------------
  //first機体の位置を代入
  double position_x_first = odom_first.x;
  double position_y_first = odom_first.y;
  //first機体の角度を代入
  double rad_first = odom_first.theta;

  //first機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_first = position_x_first;
  world_offset_position_y_first = position_y_first;
  world_offset_position_theta_first = rad_first;
}

//cb_odom_second関数定義
//機体のオドメトリデータの取得
void data_create::cb_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg){
  //second機体の状態受信
  odom_second = *msg;
  //second機体に関する計算----------------------------------------------------------
  //second機体の位置を代入
  double position_x_second = odom_second.x;
  double position_y_second = odom_second.y;
  //second機体の角度を代入
  double rad_second = odom_second.theta;

  //second機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_second = position_x_second;
  world_offset_position_y_second = position_y_second;
  world_offset_position_theta_second = rad_second;
}

//gazebo_true_position
//cb_gazebo_odom_first関数定義
void data_create::cb_gazebo_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg){
  gazebo_first = *msg;
}

//cb_gazebo_odom_second関数定義
//機体のオドメトリデータの取得
void data_create::cb_gazebo_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg){
  gazebo_second = *msg;
}

void data_create::cb_target_point(const geometry_msgs::Pose2D::ConstPtr &position){
  geometry_msgs::Pose2D point = *position;

  target_point_x = point.x;
  target_point_y = point.y;
  target_point_yaw = point.theta;
}

void data_create::cb_target_point_first(const nav_msgs::Odometry::ConstPtr &position){
  target_point_first_x = position->pose.pose.position.x;
  target_point_first_y = position->pose.pose.position.y;
  target_time_first = position->pose.pose.orientation.w;
}

void data_create::cb_target_point_second(const nav_msgs::Odometry::ConstPtr &position){
  target_point_second_x = position->pose.pose.position.x;
  target_point_second_y = position->pose.pose.position.y;
  target_time_second = position->pose.pose.orientation.w;
}

void data_create::cb_vel_first(const geometry_msgs::Twist::ConstPtr &speed){
  x_vel_first=speed->angular.x;
  y_vel_first=speed->angular.y;
  omega_r_first=speed->linear.z;
  omega_l_first=speed->linear.y;
  machine_x_first=speed->linear.x;
  machine_z_first=speed->angular.z;
}

void data_create::cb_vel_second(const geometry_msgs::Twist::ConstPtr &speed){
  x_vel_second=speed->angular.x;
  y_vel_second=speed->angular.y;
  omega_r_second=speed->linear.z;
  omega_l_second=speed->linear.y;
  machine_x_second=speed->linear.x;
  machine_z_second=speed->angular.z;
}

void data_create::cb_block(const nav_msgs::Odometry::ConstPtr &position){
  block = *position;
}
void data_create::pub_send_data(const ros::TimerEvent&){
  //搬送物の制御点の位置と姿勢
  world_offset_position_x_control = (world_offset_position_x_first + world_offset_position_x_second)/2.0;
  world_offset_position_y_control = (world_offset_position_y_first + world_offset_position_y_second)/2.0;
  world_offset_position_theta_control = atan2(world_offset_position_y_first - world_offset_position_y_second, world_offset_position_x_first - world_offset_position_x_second);

  //制御点間の計算
  tf_distance = sqrt( (pow((world_offset_position_x_first - world_offset_position_x_second),2)) + (pow((world_offset_position_y_first - world_offset_position_y_second),2)) );
  ROS_INFO("distance_%f",tf_distance);
  
  //gazebo上の搬送物の制御点の位置と姿勢
  gazebo_control.x = (gazebo_first.x + gazebo_second.x)/2.0;
  gazebo_control.y = (gazebo_first.y + gazebo_second.y)/2.0;
  gazebo_control.theta = atan2(gazebo_first.y - gazebo_second.y, gazebo_first.x - gazebo_second.x);

  //gazebo上の制御点間の計算
  double gazebo_tf_distance = sqrt( (pow((gazebo_first.x - gazebo_second.x),2)) + (pow((gazebo_first.y - gazebo_second.y),2)) );

  geometry_msgs::Pose data;
  //送信のために値を代入
  senddata.poses.clear();
  //位置
  //0(搬送物)
  data.position.x = world_offset_position_x_control;
  data.position.y = world_offset_position_y_control;
  data.position.z = world_offset_position_theta_control;
  data.orientation.x = tf_distance;
  senddata.poses.push_back(data);

  //1(first)
  data.position.x = world_offset_position_x_first;
  data.position.y = world_offset_position_y_first;
  data.position.z = world_offset_position_theta_first;
  senddata.poses.push_back(data);

  //2(second)
  data.position.x = world_offset_position_x_second;
  data.position.y = world_offset_position_y_second;
  data.position.z = world_offset_position_theta_second;
  senddata.poses.push_back(data);

  //gazebo上の真値
  //3(搬送物)
  data.position.x = gazebo_control.x;
  data.position.y = gazebo_control.y;
  data.position.z = gazebo_control.theta;
  data.orientation.x = gazebo_tf_distance;
  senddata.poses.push_back(data);

  //4(first)
  data.position.x = gazebo_first.x;
  data.position.y = gazebo_first.y;
  data.position.z = gazebo_first.theta;
  data.orientation.x = gazebo_first.x - world_offset_position_x_first;
  data.orientation.y = gazebo_first.y - world_offset_position_y_first;
  data.orientation.z = gazebo_first.theta - world_offset_position_theta_first;
  data.orientation.w = sqrt( (pow((gazebo_first.x - world_offset_position_x_first),2)) + (pow((gazebo_first.y - world_offset_position_y_first),2)) );
  senddata.poses.push_back(data);

  //5(second)
  data.position.x = gazebo_second.x;
  data.position.y = gazebo_second.y;
  data.position.z = gazebo_second.theta;
  data.orientation.x = gazebo_second.x - world_offset_position_x_second;
  data.orientation.y = gazebo_second.y - world_offset_position_y_second;
  data.orientation.z = gazebo_second.theta - world_offset_position_theta_second;
  data.orientation.w = sqrt( (pow((gazebo_second.x - world_offset_position_x_second),2)) + (pow((gazebo_second.y - world_offset_position_y_second),2)) );
  senddata.poses.push_back(data);

  //目標位置
  //6(搬送物)
  data.position.x = target_point_x;
  data.position.y = target_point_y;
  data.position.z = target_point_yaw;
  senddata.poses.push_back(data);

  //7(first)
  data.position.x = target_point_first_x;
  data.position.y = target_point_first_y;
  data.position.z = 0;
  senddata.poses.push_back(data);

  //8(second)
  data.position.x = target_point_second_x;
  data.position.y = target_point_second_y;
  data.position.z = 0;
  senddata.poses.push_back(data);

  //速度
  //9(first)
  data.position.x=x_vel_first;
  data.position.y=y_vel_first;
  data.orientation.x=machine_x_first;
  data.orientation.z=machine_z_first;
  data.orientation.w=omega_l_first;
  data.orientation.y=omega_r_first;
  senddata.poses.push_back(data);

  //10(second)
  data.position.x=x_vel_second;
  data.position.y=y_vel_second;
  data.orientation.x=machine_x_second;
  data.orientation.z=machine_z_second;
  data.orientation.w=omega_l_second;
  data.orientation.y=omega_r_second;
  senddata.poses.push_back(data);

  //11(blockの真値)
  data.position.x = block.pose.pose.position.x;
  data.position.y = block.pose.pose.position.y;
  data.position.z = tf::getYaw(block.pose.pose.orientation);
  senddata.poses.push_back(data);



  pub_data.publish(senddata);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_create");
	data_create data_create;
	ros::spin();
	return 0;
}
