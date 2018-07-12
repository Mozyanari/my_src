//搬送物のサブゴール間の距離
#define separete_distance 1.0
//搬送物の角度の間隔
#define separate_theta 0.174532

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>


#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include <iostream>



class path_plan_time{
public:
  path_plan_time();
private:
/*
関数定義
*/
  //コールバック定義

  //制御点の目標位置から機体の位置の算出
  void calc_machine_position(const geometry_msgs::Pose2D::ConstPtr &position);

  //目標位置へ到達するときの機体の時間を計算
  void send_machine_speed(void);

  //目標位置へ到達する時間
  void calc_arrived_time(const std_msgs::Int32::ConstPtr &number);

  //機体のオフセット位置のオドメトリ取得
  void cb_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg);
  void cb_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg);

  //使用関数
  void send_target_point(void);
  void send_target_marker(void);


/*
使用するTopicの定義
*/
  //目標位置を取得
  ros::Subscriber sub_target_point;
  //機体のオドメトリデータ取得
  ros::Subscriber sub_offset_odom_first;
  ros::Subscriber sub_offset_odom_second;
  //機体の到着位置を取得
  ros::Subscriber sub_arrive_position_first;
  ros::Subscriber sub_arrive_position_second;
  //time_contorolerから番号を取得
  ros::Subscriber sub_time_controler;

  //機体の目標位置と速度を送信
  ros::Publisher pub_target_point_first;
  ros::Publisher pub_target_point_second;

  //目標時間単体を送信
  ros::Publisher pub_time;

  //Marker_define
  ros::Publisher marker_pub;

  //ノードハンドラ作成
	ros::NodeHandle nh;

/*
現在の機体のデータ関係
*/
  /*path_plan_time::cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg)*/
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_first;
  double world_offset_position_y_first;

  /*path_plan_time::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg)*/
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_second;
  double world_offset_position_y_second;

  /*path_plan_time::path_plan_time()*/
  //機体の最高加速度[m/s^2]
  double acc_max;
  //最高速度[m/s]
  double Max_speed;
  //最低速度
  double Min_speed;
  //基準となる使用する速度
  double use_speed;
  //オフセット距離
  double s;
  //機体間距離[m]
  double distance_multi;
  //制御点までの距離
  double control_point;

/*
目標位置データ関係
*/
  //動的に配列を取得する
  //サブゴールの分割数
  int sub_goal_number;
  //世界座標系におけるサブゴールの位置の配列
  //制御点に関して
  double *sub_goal_x;
  double *sub_goal_y;
  double *sub_goal_rad;
  //first機体に関して
  double *sub_goal_first_x;
  double *sub_goal_first_y;
  //second機体に関して
  double *sub_goal_second_x;
  double *sub_goal_second_y;
  //データ送信のためのサブゴールデータ変数
  nav_msgs::Odometry sub_goal_first;
  nav_msgs::Odometry sub_goal_second;
/*
マーカデータ
*/
  //Marker_data
  visualization_msgs::Marker marker_control;
  visualization_msgs::Marker marker_first;
  visualization_msgs::Marker marker_second;

};

path_plan_time::path_plan_time(){
  //購読するトピックの定義
  sub_target_point = nh.subscribe("/target_point", 5, &path_plan_time::calc_machine_position,this);
  //first
  sub_offset_odom_first = nh.subscribe("/first/offset_odom_true", 5, &path_plan_time::cb_odom_first,this);
  //second
  sub_offset_odom_second = nh.subscribe("/second/offset_odom_true", 5, &path_plan_time::cb_odom_second,this);
  //サブゴールのnumberを取得する
  sub_time_controler = nh.subscribe("/time_result", 5, &path_plan_time::calc_arrived_time,this);

  //配布するトピックの定義
  //それぞれの機体に目標位置と時間を送信
  pub_target_point_first = nh.advertise<nav_msgs::Odometry>("/first/target_point", 1);
  pub_target_point_second = nh.advertise<nav_msgs::Odometry>("/second/target_point", 1);

  //時間と番号を送信
  pub_time = nh.advertise<std_msgs::Header>("/time_check", 1);

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

/*
現在の機体のデータ関係
*/
  //オフセット距離[m]160mm
  s = 0.16;
  //機体の最高加速度:80[mm/s^2]
  acc_max = 0.08;
  //最高速度:0.1[m/s],100[mm/s]
  Max_speed = 0.1;
  //最低速度:0.05[m/s],50[mm/s]
  Min_speed = 0.05;
  //使用速度:0.04[m/s]
  use_speed = 0.04;
  //機体間距離[m]570mm,シミュレーション1m
  distance_multi = 1.0;
  //制御点までの距離
  control_point = distance_multi / 2;

/*
目標位置データ関係
*/
  //変数の初期化

  //動的に取得しているので0で初期化
  sub_goal_x = 0;
  sub_goal_y = 0;
  sub_goal_rad = 0;
  //first機体に関して
  sub_goal_first_x = 0;
  sub_goal_first_y = 0;
  //second機体に関して
  sub_goal_second_x = 0;
  sub_goal_second_y = 0;

  //デバック用に搬送物の制御点の初期化
  world_offset_position_x_first = 1;
  world_offset_position_y_first = 0;
  world_offset_position_x_second = 0;
  world_offset_position_y_second = 0;
}

//関数定義-----------------------------------------------------------------------
//cb_odom_first関数定義
//機体のオドメトリデータの取得
void path_plan_time::cb_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg){
  //first機体の状態受信
  world_offset_position_x_first = msg->x;
  world_offset_position_y_first = msg->y;
}
//cb_odom_second関数定義
//機体のオドメトリデータの取得
void path_plan_time::cb_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg){
  //second機体の状態受信
  world_offset_position_x_second = msg->x;
  world_offset_position_y_second = msg->y;
}



//制御点の目標位置が更新された時にそれぞれの機体のサブゴールを作成
void path_plan_time::calc_machine_position(const geometry_msgs::Pose2D::ConstPtr &position){

  //目標の(x,y,θ)位置を取得
  double target_control_point_x = position->x;
  double target_control_point_y = position->y;
  double target_control_point_rad = position->theta;

  //最終的な目標とするfirstとsecondのそれぞれの位置を計算
  //first
  double target_x_first = target_control_point_x + (control_point * cos(target_control_point_rad));
  double target_y_first = target_control_point_y + (control_point * sin(target_control_point_rad));
  //second
  double target_x_second = target_control_point_x - (control_point * cos(target_control_point_rad));
  double target_y_second = target_control_point_y - (control_point * sin(target_control_point_rad));


  //現在の制御点の位置と姿勢を計算
  double now_control_point_x = (world_offset_position_x_first + world_offset_position_x_second)/2.0;
  double now_control_point_y = (world_offset_position_y_first + world_offset_position_y_second)/2.0;
  double now_control_point_rad = atan2((world_offset_position_y_first - world_offset_position_y_second),(world_offset_position_x_first - world_offset_position_x_second));

  ROS_INFO("now_control_point_x=%f",now_control_point_x);
  ROS_INFO("now_control_point_y=%f",now_control_point_y);
  ROS_INFO("now_control_point_rad=%f",now_control_point_rad);

  //現在機体の位置と姿勢を取得
  //現在の制御点と目標の制御点までの違いを計算
  double diff_machine_position_x = target_control_point_x - now_control_point_x;
  double diff_machine_position_y = target_control_point_y - now_control_point_y;
  double diff_machine_rad = target_control_point_rad - now_control_point_rad;


  //サブゴールの設定
  //サブゴールの数を計算
  sub_goal_number = 1;
  double among_sub_goal_x = 0;
  double among_sub_goal_y = 0;
  double among_sub_goal_rad = 0;
  for(;;sub_goal_number++){
    ROS_INFO("sub_goal_number=%d",sub_goal_number);
    //搬送物の制御点の間隔
    among_sub_goal_x=diff_machine_position_x/(double)(sub_goal_number);
    among_sub_goal_y=diff_machine_position_y/(double)(sub_goal_number);
    among_sub_goal_rad=diff_machine_rad/(double)(sub_goal_number);

    if((among_sub_goal_x <separete_distance)&&(among_sub_goal_y <separete_distance)&&(among_sub_goal_rad < separate_theta)){
      break;
    }
  }

  //まずは以前使用したメモリを削除
  delete[] sub_goal_x;
  delete[] sub_goal_y;
  delete[] sub_goal_rad;

  delete[] sub_goal_first_x;
  delete[] sub_goal_first_y;

  delete[] sub_goal_second_x;
  delete[] sub_goal_second_y;

  //ROS_INFO("delete");

  //sub_goal_numberの数だけ動的にメモリを取得
  //搬送物に関して
  sub_goal_x = new double[sub_goal_number];
  sub_goal_y = new double[sub_goal_number];
  sub_goal_rad = new double[sub_goal_number];
  //first機体に関して
  sub_goal_first_x = new double[sub_goal_number];
  sub_goal_first_y = new double[sub_goal_number];
  //second機体に関して
  sub_goal_second_x = new double[sub_goal_number];
  sub_goal_second_y = new double[sub_goal_number];

  //搬送物のサブゴール位置を決定
  int i=0;
  while(i<sub_goal_number-1){
    sub_goal_x[i]=now_control_point_x + (among_sub_goal_x * (i+1));
    sub_goal_y[i]=now_control_point_y + (among_sub_goal_y * (i+1));
    sub_goal_rad[i]=now_control_point_rad + (among_sub_goal_rad *(i+1));
    i++;
  }
  //最終地点を入力
  sub_goal_x[sub_goal_number-1]=target_control_point_x;
  sub_goal_y[sub_goal_number-1]=target_control_point_y;
  sub_goal_rad[sub_goal_number-1]=target_control_point_rad;


  //機体のサブゴールの位置
  //first
  i=0;
  while(i<sub_goal_number-1){
    //firstのサブゴールを入力
    //角度を考慮
    sub_goal_first_x[i] = sub_goal_x[i] + (control_point * cos(sub_goal_rad[i]));
    sub_goal_first_y[i] = sub_goal_y[i] + (control_point * sin(sub_goal_rad[i]));
    i++;
  }
  //最終地点を入力
  sub_goal_first_x[sub_goal_number-1]=target_x_first;
  sub_goal_first_y[sub_goal_number-1]=target_y_first;

  //second
  i=0;
  while(i<sub_goal_number-1){
    //secondのサブゴールを入力
    //角度を考慮
    sub_goal_second_x[i] = sub_goal_x[i] - (control_point * cos(sub_goal_rad[i]));
    sub_goal_second_y[i] = sub_goal_y[i] - (control_point * sin(sub_goal_rad[i]));
    i++;
  }
  //最終地点を入力
  sub_goal_second_x[sub_goal_number-1]=target_x_second;
  sub_goal_second_y[sub_goal_number-1]=target_y_second;

  //デバック
  for(int k=0;k<sub_goal_number;k++){
    //ROS_INFO("sub_goal_second_x[%d]=%f",k,sub_goal_second_x[k]);
    //ROS_INFO("sub_goal_second_y[%d]=%f",k,sub_goal_second_y[k]);
  }

  //位置データと時間を計算
  //まずは今の位置からサブゴールまでの距離
  double diff_distance_first = sqrt( (pow((sub_goal_first_x[0] - world_offset_position_x_first),2)) + (pow(sub_goal_first_y[0] - world_offset_position_y_first,2)) );
  double diff_distance_second = sqrt( (pow((sub_goal_second_x[0] - world_offset_position_x_second),2)) + (pow(sub_goal_second_y[0] - world_offset_position_y_second,2)) );
  //遅延が1秒あると考えて，最高速度は0.08m/sだから少なくとも1秒あれば次の位置に行けるようにしたい
  double time = 2;
  while(1){
    if(((diff_distance_first / time) < use_speed) && ((diff_distance_second / time) < use_speed)){
      //時間は十分と判定
      break;
    }
    time++;
  }
  //時間をデバック
  ROS_INFO("time=%f",time);
  //位置と時間と番号を代入
  sub_goal_first.pose.pose.position.x = sub_goal_first_x[0];
  sub_goal_first.pose.pose.position.y = sub_goal_first_y[0];
  sub_goal_first.pose.pose.position.z = 0;
  sub_goal_first.header.stamp = ros::Time::now() + ros::Duration(time);

  sub_goal_second.pose.pose.position.x = sub_goal_second_x[0];
  sub_goal_second.pose.pose.position.y = sub_goal_second_y[0];
  sub_goal_second.pose.pose.position.z = 0;
  sub_goal_second.header.stamp = ros::Time::now() + ros::Duration(time);

  //データを送信
  pub_target_point_first.publish(sub_goal_first);
  pub_target_point_second.publish(sub_goal_second);

  //時間を送信
  //stampに時間、frame_idにnumberを入れる
  std_msgs::Header time_pub;
  time_pub.stamp = ros::Time::now() + ros::Duration(time);
  time_pub.seq = 0;
  pub_time.publish(time_pub);

  //マーカを送信

  send_target_marker();

}
//位置データと時間を計算
void path_plan_time::calc_arrived_time(const std_msgs::Int32::ConstPtr &data){
  //intに変換するためにnumberに代入
  int number = data->data;
  //numberがマイナスもしくは，sub_goal_number-1以上だと異常値と判定
  if((number < 0) && (number > (sub_goal_number-1))){
    return;
  }
  //sub_goal_number-1が来たらストップ信号を出す
  if(number == (sub_goal_number-1)){
    //first
    sub_goal_first.pose.pose.position.x = 0;
    sub_goal_first.pose.pose.position.y = 0;
    sub_goal_first.pose.pose.position.z = -1;
    sub_goal_first.header.stamp = ros::Time::now();

    //second
    sub_goal_second.pose.pose.position.x = 0;
    sub_goal_second.pose.pose.position.y = 0;
    sub_goal_second.pose.pose.position.z = -1;
    sub_goal_second.header.stamp = ros::Time::now();

    //データを送信
    pub_target_point_first.publish(sub_goal_first);
    pub_target_point_second.publish(sub_goal_second);

    //時間を送信
    //stampに時間、frame_idにnumberを入れる
    std_msgs::Header time_pub;
    time_pub.stamp = ros::Time::now();
    time_pub.seq = -1;
    pub_time.publish(time_pub);

    return;
  }

  //時間を計算
  double diff_distance_first = sqrt( (pow((sub_goal_first_x[number+1] - world_offset_position_x_first),2)) + (pow(sub_goal_first_y[number+1] - world_offset_position_y_first,2)) );
  double diff_distance_second = sqrt( (pow((sub_goal_second_x[number+1] - world_offset_position_x_second),2)) + (pow(sub_goal_second_y[number+1] - world_offset_position_y_second,2)) );

  //遅延が1秒あると考えて，最高速度は0.08m/sだから少なくとも1秒あれば次の位置に行けるようにしたい
  int time = 2;
  while(1){
    if(((diff_distance_first / time) < use_speed) && ((diff_distance_second / time) < use_speed)){
      //時間は十分と判定
      break;
    } 
    time++;
  }

  //時間のデバック
  ROS_INFO("time=%d",time);

  //次の場所のサブゴールと時間を送信
  sub_goal_first.pose.pose.position.x = sub_goal_first_x[number+1];
  sub_goal_first.pose.pose.position.y = sub_goal_first_y[number+1];
  sub_goal_first.pose.pose.position.z = number+1;
  sub_goal_first.header.stamp = ros::Time::now() + ros::Duration(time);

  sub_goal_second.pose.pose.position.x = sub_goal_second_x[number+1];
  sub_goal_second.pose.pose.position.y = sub_goal_second_y[number+1];
  sub_goal_second.pose.pose.position.z = number+1;
  sub_goal_second.header.stamp = ros::Time::now() + ros::Duration(time);

  //データを送信
  pub_target_point_first.publish(sub_goal_first);
  pub_target_point_second.publish(sub_goal_second);

  //時間を送信
  //stampに時間、frame_idにnumberを入れる
  std_msgs::Header time_pub;
  time_pub.stamp = ros::Time::now() + ros::Duration(time);
  time_pub.seq = number+1;
  pub_time.publish(time_pub);
}


void path_plan_time::send_target_marker(void){
  //control_target_positionの目標位置をマークする
  for(int i=0;i<sub_goal_number;i++){
    std::ostringstream ss;
    ss << i;
    marker_control.header.frame_id = "/map";
    marker_control.header.stamp = ros::Time::now();
    marker_control.ns = ("control_marker"+ss.str());
    marker_control.id = 0;
    marker_control.type = visualization_msgs::Marker::SPHERE;
    marker_control.action = visualization_msgs::Marker::ADD;
    marker_control.pose.position.x = sub_goal_x[i];
    marker_control.pose.position.y = sub_goal_y[i];
    marker_control.pose.position.z = 0;
    marker_control.pose.orientation.x = 0.0;
    marker_control.pose.orientation.y = 0.0;
    marker_control.pose.orientation.z = 0.0;
    marker_control.pose.orientation.w = 1.0;

    marker_control.scale.x = 0.05;
    marker_control.scale.y = 0.05;
    marker_control.scale.z = 0.05;

    marker_control.color.r = 1.0f;
    marker_control.color.g = 0.0f;
    marker_control.color.b = 0.0f;
    marker_control.color.a = 0.8;

    marker_control.lifetime = ros::Duration();

    marker_pub.publish(marker_control);

    ros::Duration(0.01).sleep();

    //ROS_INFO("i=%d",i);
  }
  //firstの目標位置をマークする
  for(int i=0;i<sub_goal_number;i++){
    std::ostringstream ss;
    ss << i;
    marker_control.header.frame_id = "/map";
    marker_control.header.stamp = ros::Time::now();
    marker_control.ns = ("first_marker"+ss.str());
    marker_control.id = 0;
    marker_control.type = visualization_msgs::Marker::SPHERE;
    marker_control.action = visualization_msgs::Marker::ADD;
    marker_control.pose.position.x = sub_goal_first_x[i];
    marker_control.pose.position.y = sub_goal_first_y[i];
    marker_control.pose.position.z = 0;
    marker_control.pose.orientation.x = 0.0;
    marker_control.pose.orientation.y = 0.0;
    marker_control.pose.orientation.z = 0.0;
    marker_control.pose.orientation.w = 1.0;

    marker_control.scale.x = 0.05;
    marker_control.scale.y = 0.05;
    marker_control.scale.z = 0.05;

    marker_control.color.r = 0.0f;
    marker_control.color.g = 1.0f;
    marker_control.color.b = 0.0f;
    marker_control.color.a = 0.8;

    marker_control.lifetime = ros::Duration();

    marker_pub.publish(marker_control);

    ros::Duration(0.01).sleep();

    //ROS_INFO("i=%d",i);
  }
  //secondの目標位置をマークする
  for(int i=0;i<sub_goal_number;i++){
    std::ostringstream ss;
    ss << i;
    marker_control.header.frame_id = "/map";
    marker_control.header.stamp = ros::Time::now();
    marker_control.ns = ("second_marker"+ss.str());
    marker_control.id = 0;
    marker_control.type = visualization_msgs::Marker::SPHERE;
    marker_control.action = visualization_msgs::Marker::ADD;
    marker_control.pose.position.x = sub_goal_second_x[i];
    marker_control.pose.position.y = sub_goal_second_y[i];
    marker_control.pose.position.z = 0;
    marker_control.pose.orientation.x = 0.0;
    marker_control.pose.orientation.y = 0.0;
    marker_control.pose.orientation.z = 0.0;
    marker_control.pose.orientation.w = 1.0;

    marker_control.scale.x = 0.05;
    marker_control.scale.y = 0.05;
    marker_control.scale.z = 0.05;

    marker_control.color.r = 0.0f;
    marker_control.color.g = 0.0f;
    marker_control.color.b = 1.0f;
    marker_control.color.a = 0.8;

    marker_control.lifetime = ros::Duration();

    marker_pub.publish(marker_control);

    ros::Duration(0.01).sleep();

    //ROS_INFO("i=%d",i);
  }
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_plan_time");
	path_plan_time path_plan_time;
	ros::spin();
	return 0;
}
