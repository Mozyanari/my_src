//サブゴールを何分割するか
//つまり，最終地点を含めたゴールの数
#define split_number 5

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include <iostream>



class single_path_plan_map{
public:
  single_path_plan_map();
  //Listener定義
  tf::TransformListener listener;
private:
/*
関数定義
*/
  //コールバック定義
  //それぞれの到達ポジションの取得
  void Arrive_position_second(const nav_msgs::Odometry::ConstPtr &position);

  //制御点の目標位置から機体の位置の算出
  void calc_machine_position(const nav_msgs::Odometry::ConstPtr &position);

  //目標位置へ到達するときの機体の速度を計算
  void send_machine_speed(void);

  //機体からのオドメトリ取得
  void cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg);

  //使用関数
  void send_target_point(void);
  void send_target_marker(void);
  void update_odom_second(void);


/*
使用するTopicの定義
*/
  //目標位置を取得
  ros::Subscriber sub_target_point;
  //機体のオドメトリデータ取得
  ros::Subscriber sub_odom_second;
  //機体の到着位置を取得
  ros::Subscriber sub_arrive_position_second;

  //機体の目標位置と速度を送信
  ros::Publisher pub_target_point_second;

  //Marker_define
  ros::Publisher marker_pub;

  //ノードハンドラ作成
	ros::NodeHandle nh;

/*
現在の機体のデータ関係
*/
  /*single_path_plan_map::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg)*/
  nav_msgs::Odometry odom_second;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_second;
  double world_offset_position_y_second;

  /*single_path_plan_map::single_path_plan_map()*/
  //機体の最高加速度[m/s^2]
  double acc_max;
  //最高速度[m/s]
  double Max_speed;
  //最低速度
  double Min_speed;
  //オフセット距離
  double s;
  //機体間距離[m]
  double distance_multi;

/*
目標位置データ関係
*/
  //世界座標系におけるサブゴールの位置の配列
  //second機体に関して
  double sub_goal_second_x[split_number];
  double sub_goal_second_y[split_number];

  //データ送信のためのサブゴールデータ変数
  nav_msgs::Odometry sub_goal_second;

  //それぞれの機体の目標速度
  double sub_goal_speed_second;
  //前回のスピードを保持
  double old_speed_second;

/*
複数のデバイス間で通信するため，そのためのフラグデータ
*/
  //今何番目のサブゴールに到達したかの判定番号
  int second_number;
  int old_second_number;

  int last_frag;
/*
マーカデータ
*/
  //Marker_data
  visualization_msgs::Marker marker_control;
  visualization_msgs::Marker marker_second;

};

single_path_plan_map::single_path_plan_map(){
  //購読するトピックの定義
  sub_target_point = nh.subscribe("/target_point", 5, &single_path_plan_map::calc_machine_position,this);
  sub_odom_second = nh.subscribe("/ypspur_ros_second/odom", 5, &single_path_plan_map::cb_odom_second,this);
  sub_arrive_position_second=nh.subscribe("/frag_data_second", 5, &single_path_plan_map::Arrive_position_second,this);

  //配布するトピックの定義
  pub_target_point_second = nh.advertise<nav_msgs::Odometry>("target_point_second", 1);

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

/*
現在の機体のデータ関係
*/
  //odomのクオータニオンを初期化
  odom_second.pose.pose.orientation.w = 1.0;
  //オフセット距離[m]160mm
  s = 0.16;
  //機体の最高加速度:80[mm/s^2]
  acc_max = 0.08;
  //最高速度:0.1[m/s],100[mm/s]
  Max_speed = 0.2;
  //最低速度:0.05[m/s],50[mm/s]
  Min_speed = 0.05;
  //機体間距離[m]570mm
  distance_multi = 0.57;

/*
目標位置データ関係
*/
  //変数の初期化
  old_speed_second = 0;

/*
複数のデバイス間で通信するため，そのためのフラグデータ
*/
  //サブゴール番号変数の初期化
  second_number=-1;
  old_second_number = -1;

}

//関数定義-----------------------------------------------------------------------
//cb_odom_second関数定義
//機体のオドメトリデータの取得
void single_path_plan_map::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg){
  //second機体の状態受信
  odom_second = *msg;
}
//機体の世界座標系における位置の更新
void single_path_plan_map::update_odom_second(void){
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


//制御点の目標位置が更新された時にそれぞれの機体のサブゴールを作成
void single_path_plan_map::calc_machine_position(const nav_msgs::Odometry::ConstPtr &position){
  //目標位置を取得
  nav_msgs::Odometry target_position = *position;

  //目標の(x,y)位置を取得
  double target_point_x=target_position.pose.pose.position.x;
  double target_point_y=target_position.pose.pose.position.y;

  //最終的な目標とするsecondとsecondのそれぞれの位置を計算
  //second
  double target_x_second = target_point_x;
  double target_y_second = target_point_y;

  //機体の位置を更新
  update_odom_second();

  //現在機体の位置と姿勢を取得
  //現在の制御点と目標の制御点までの違いを計算
  double diff_machine_position_x = target_point_x - world_offset_position_x_second;
  double diff_machine_position_y = target_point_y - world_offset_position_y_second;

  //サブゴールの設定
  //サブゴールの間隔を計算
  double among_sub_goal_x=diff_machine_position_x/split_number;
  double among_sub_goal_y=diff_machine_position_y/split_number;

  //機体のサブゴールの位置
  for(int j=0;j<(split_number-1);j++){
    //secondのサブゴールを入力
    sub_goal_second_x[j] = world_offset_position_x_second + (among_sub_goal_x * (j+1));
    sub_goal_second_y[j] = world_offset_position_y_second + (among_sub_goal_y * (j+1));
  }
  //最終地点を入力
  sub_goal_second_x[split_number-1] = target_x_second;
  sub_goal_second_y[split_number-1] = target_y_second;


  for(int k=0;k<split_number;k++){
    ROS_INFO("sub_goal_second_x[%d]=%f",k,sub_goal_second_x[k]);
    ROS_INFO("sub_goal_second_y[%d]=%f",k,sub_goal_second_y[k]);
  }
  //番号を初期化
  second_number = -1;
  old_second_number = -1;

  //位置データと番号を入力
  send_target_point();

  //目標スピードを入力
  send_machine_speed();

  //データを送信
  pub_target_point_second.publish(sub_goal_second);

  //マーカを送信
  send_target_marker();

}
//目標地点を入力
void single_path_plan_map::send_target_point(void){
  //second機体の次の目標位置を入力(map)
  geometry_msgs::PointStamped map_point;
  geometry_msgs::PointStamped world_point;

  map_point.header.frame_id = "map";
  world_point.header.frame_id = "world";

  std::string map = "map";
  std::string world = "world";


  if((-2<second_number) && (second_number<(split_number-1))){
    map_point.point.x=sub_goal_second_x[second_number+1];
    map_point.point.y=sub_goal_second_y[second_number+1];
    //sub_goal_second.pose.pose.position.x = sub_goal_second_x[second_number+1];
    //sub_goal_second.pose.pose.position.y = sub_goal_second_y[second_number+1];
    sub_goal_second.pose.pose.position.z=(second_number+1);
  }
  //world上の値へ変換
  try{
    //wait_until_trans
    listener.waitForTransform(map,world,ros::Time::now(),ros::Duration(5.0));
    //trans_to_wolrd_point
    listener.transformPoint(map,ros::Time(0),map_point,world,world_point);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  //返還後の位置を送信
  sub_goal_second.pose.pose.position.x = world_point.point.x;
  sub_goal_second.pose.pose.position.y = world_point.point.y;

}
//目標位置へ到達するときの機体の速度を計算
void single_path_plan_map::send_machine_speed(void){
  //変数定義
  double second_judge = 0;
  double speed_second = 0;
  static double old_speed_second = 0;

  //機体の位置を更新
  update_odom_second();

  //それぞれの機体の位置からの目標地点までの距離を計算
  //サブゴール
  double a=sub_goal_second.pose.pose.position.x-world_offset_position_x_second;
  double b=sub_goal_second.pose.pose.position.y-world_offset_position_y_second;

  double target_distance_second = sqrt((pow(a,2)) + (pow(b,2)));


  //viが実数解を持つためのtiを決める
  int time = 1;
  for(;time<100;time++){
    //条件判定式
    second_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_second)-(2*acc_max*target_distance_second);
    //条件判定でどちらも正になるtimeがわかればそれを使う
    if(second_judge>0){
      break;
    }
  }

  //viを計算
  while(1){
    speed_second = old_speed_second + (acc_max * time) - sqrt(second_judge);
    //viMaxを超えているかを判定
    if(speed_second < Max_speed){
      break;
    }else{
      //もう一度timeを増やして計算
      time++;
      second_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_second)-(2*acc_max*target_distance_second);
    }
  }
  ROS_INFO("time=%d",time);

  //速度を代入
  sub_goal_second.twist.twist.linear.z=speed_second;

  //速度の保持
  old_speed_second = speed_second;
}

//secondがサブゴールに到達したら実行する
void single_path_plan_map::Arrive_position_second(const nav_msgs::Odometry::ConstPtr &position){
  //到達位置を取得
  nav_msgs::Odometry second_position= *position;

  //今何番目のサブゴールかを判定
  second_number = second_position.pose.pose.position.z;

  //マシンにサブゴールを更新するデータを送るかのフラグ
  int send_frag = 0;
  //0~(split_number-2)番目なら，次のサブゴールをセット.(split_number-1)番目は最終地点へ到達したとして次の目的地の更新はしない
  //その他ならエラー
  if((-1<second_number) && (second_number < (split_number-1))){
    send_frag = 1;
  }else if(second_number == (split_number-1)){
    ROS_INFO("End");
  }else{
    ROS_INFO("Error");
  }

  //前回のフラグと比較して実行するか判定
  if((send_frag == 1) && (old_second_number != second_number)){
    //目標位置を入力
    send_target_point();
    //目標スピードを入力
    send_machine_speed();
    //データを送信
    pub_target_point_second.publish(sub_goal_second);

    //フラグ保持
    old_second_number = second_number;
  }
}





void single_path_plan_map::send_target_marker(void){
  //secondの目標位置をマークする
  for(int i=0;i<split_number;i++){
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

    marker_control.color.r = 1.0f;
    marker_control.color.g = 0.0f;
    marker_control.color.b = 0.0f;
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
	ros::init(argc, argv, "single_path_plan_map");
	single_path_plan_map single_path_plan_map;
	ros::spin();
	return 0;
}
