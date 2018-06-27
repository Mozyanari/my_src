//サブゴールを何分割するか
//つまり，最終地点を含めたゴールの数
#define split_number 20

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>

#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include <iostream>



class two_amcl_path_plan{
public:
  two_amcl_path_plan();
private:
/*
関数定義
*/
  //コールバック定義
  //それぞれの到達ポジションの取得
  void Arrive_position_first(const nav_msgs::Odometry::ConstPtr &position);
  void Arrive_position_second(const nav_msgs::Odometry::ConstPtr &position);


  //制御点の目標位置から機体の位置の算出
  void calc_machine_position(const geometry_msgs::Pose2D::ConstPtr &position);

  //目標位置へ到達するときの機体の速度を計算
  void send_machine_speed(void);

  //機体のオフセット位置のオドメトリ取得
  void cb_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg);
  void cb_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg);

  //使用関数
  void send_target_point(void);
  void send_target_marker(void);
  void update_odom_first(void);
  void update_odom_second(void);


/*
使用するTopicの定義
*/
  //目標位置を取得
  ros::Subscriber sub_target_point;
  //機体のオドメトリデータ取得
  ros::Subscriber sub_odom_first;
  ros::Subscriber sub_odom_second;
  //機体の到着位置を取得
  ros::Subscriber sub_arrive_position_first;
  ros::Subscriber sub_arrive_position_second;

  //機体の目標位置と速度を送信
  ros::Publisher pub_target_point_first;
  ros::Publisher pub_target_point_second;

  //Marker_define
  ros::Publisher marker_pub;

  //ノードハンドラ作成
	ros::NodeHandle nh;

/*
現在の機体のデータ関係
*/
  /*two_amcl_path_plan::cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg)*/
  geometry_msgs::Pose2D odom_first;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_first;
  double world_offset_position_y_first;

  /*two_amcl_path_plan::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg)*/
  geometry_msgs::Pose2D odom_second;
  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_second;
  double world_offset_position_y_second;

  /*two_amcl_path_plan::two_amcl_path_plan()*/
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
  //制御点までの距離
  double control_point;

/*
目標位置データ関係
*/
  //世界座標系におけるサブゴールの位置の配列
  //制御点に関して
  double sub_goal_x[split_number];
  double sub_goal_y[split_number];
  double sub_goal_rad[split_number];
  //first機体に関して
  double sub_goal_first_x[split_number];
  double sub_goal_first_y[split_number];
  //second機体に関して
  double sub_goal_second_x[split_number];
  double sub_goal_second_y[split_number];
  //データ送信のためのサブゴールデータ変数
  nav_msgs::Odometry sub_goal_first;
  nav_msgs::Odometry sub_goal_second;

  //それぞれの機体の目標速度
  double sub_goal_speed_first;
  double sub_goal_speed_second;
  //前回のスピードを保持
  double old_speed_first;
  double old_speed_second;

/*
複数のデバイス間で通信するため，そのためのフラグデータ
*/
  //今何番目のサブゴールに到達したかの判定番号
  int first_number;
  int old_first_number;
  int second_number;
  int old_second_number;

  int last_frag;
/*
マーカデータ
*/
  //Marker_data
  visualization_msgs::Marker marker_control;
  visualization_msgs::Marker marker_first;
  visualization_msgs::Marker marker_second;

};

two_amcl_path_plan::two_amcl_path_plan(){
  //購読するトピックの定義
  sub_target_point = nh.subscribe("/target_point", 5, &two_amcl_path_plan::calc_machine_position,this);
  //first
  sub_odom_first = nh.subscribe("/first/offset_position", 5, &two_amcl_path_plan::cb_odom_first,this);
  sub_arrive_position_first=nh.subscribe("/first/frag_data", 5, &two_amcl_path_plan::Arrive_position_first,this);
  //second
  sub_odom_second = nh.subscribe("/second/offset_position", 5, &two_amcl_path_plan::cb_odom_second,this);
  sub_arrive_position_second=nh.subscribe("/second/frag_data", 5, &two_amcl_path_plan::Arrive_position_second,this);

  //配布するトピックの定義
  pub_target_point_first = nh.advertise<nav_msgs::Odometry>("/first/target_point", 1);
  pub_target_point_second = nh.advertise<nav_msgs::Odometry>("/second/target_point", 1);

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
  //機体間距離[m]570mm,シミュレーション1m
  distance_multi = 1.0;
  //制御点までの距離
  control_point = distance_multi / 2;

/*
目標位置データ関係
*/
  //変数の初期化
  old_speed_first = 0;
  old_speed_second = 0;

/*
複数のデバイス間で通信するため，そのためのフラグデータ
*/
  //サブゴール番号変数の初期化
  first_number=-1;
  old_first_number = -1;
  second_number=-1;
  old_second_number = -1;

}

//関数定義-----------------------------------------------------------------------
//cb_odom_first関数定義
//機体のオドメトリデータの取得
void two_amcl_path_plan::cb_odom_first(const geometry_msgs::Pose2D::ConstPtr &msg){
  //first機体の状態受信
  odom_first = *msg;
}
//機体の世界座標系における位置の更新
void two_amcl_path_plan::update_odom_first(void){
  //first機体に関する計算----------------------------------------------------------
  //first機体の位置を代入
  double position_x_first = odom_first.x;
  double position_y_first = odom_first.y;
  //first機体の角度を代入
  double rad_first = odom_first.theta;

  //first機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_first = position_x_first;
  world_offset_position_y_first = position_y_first;

  //ROS_INFO("x_first=%f",world_offset_position_x_first);
  //ROS_INFO("y_first=%f",world_offset_position_y_first);
}
//cb_odom_second関数定義
//機体のオドメトリデータの取得
void two_amcl_path_plan::cb_odom_second(const geometry_msgs::Pose2D::ConstPtr &msg){
  //second機体の状態受信
  odom_second = *msg;
}
//機体の世界座標系における位置の更新
void two_amcl_path_plan::update_odom_second(void){
  //second機体に関する計算----------------------------------------------------------
  //second機体の位置を代入
  double position_x_second = odom_second.x;
  double position_y_second = odom_second.y;
  //second機体の角度を代入
  double rad_second = odom_second.theta;

  //second機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_second = position_x_second;
  world_offset_position_y_second = position_y_second;

  //ROS_INFO("x_second=%f",world_offset_position_x_second);
  //ROS_INFO("y_second=%f",world_offset_position_y_second);
}


//制御点の目標位置が更新された時にそれぞれの機体のサブゴールを作成
void two_amcl_path_plan::calc_machine_position(const geometry_msgs::Pose2D::ConstPtr &position){
  //目標位置を取得
  geometry_msgs::Pose2D control_target_position = *position;

  //機体の位置を更新
  update_odom_first();
  update_odom_second();

  //目標の(x,y,θ)位置を取得
  double target_control_point_x = control_target_position.x;
  double target_control_point_y = control_target_position.y;
  double target_control_point_rad = control_target_position.theta;


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
  double now_control_point_rad = asin((world_offset_position_y_first - world_offset_position_y_second) / distance_multi);

  //ROS_INFO("now_control_point_x=%f",now_control_point_x);
  //ROS_INFO("now_control_point_y=%f",now_control_point_y);
  //ROS_INFO("now_control_point_rad=%f",now_control_point_rad);

  //現在機体の位置と姿勢を取得
  //現在の制御点と目標の制御点までの違いを計算
  double diff_machine_position_x = target_control_point_x - now_control_point_x;
  double diff_machine_position_y = target_control_point_y - now_control_point_y;
  double diff_machine_rad = target_control_point_rad - now_control_point_rad;


  //サブゴールの設定
  //サブゴールの間隔を計算
  double among_sub_goal_x=diff_machine_position_x/split_number;
  double among_sub_goal_y=diff_machine_position_y/split_number;
  double among_sub_goal_rad=diff_machine_rad/split_number;

  //制御点のサブゴールの位置の計算
  for(int i=0;i<split_number-1;i++){
    sub_goal_x[i]=now_control_point_x + (among_sub_goal_x * (i+1));
    sub_goal_y[i]=now_control_point_y + (among_sub_goal_y * (i+1));
    sub_goal_rad[i]=now_control_point_rad + (among_sub_goal_rad *(i+1));
  }
  //最終地点を入力
  sub_goal_x[split_number-1]=target_control_point_x;
  sub_goal_y[split_number-1]=target_control_point_y;
  sub_goal_rad[split_number-1]=target_control_point_rad;


  //機体のサブゴールの位置
  //first
  for(int j=0;j<(split_number-1);j++){
    //firstのサブゴールを入力
    //角度を考慮
    sub_goal_first_x[j] = sub_goal_x[j] + (control_point * cos(sub_goal_rad[j]));
    sub_goal_first_y[j] = sub_goal_y[j] + (control_point * sin(sub_goal_rad[j]));
  }
  //最終地点を入力
  sub_goal_first_x[split_number-1] = target_x_first;
  sub_goal_first_y[split_number-1] = target_y_first;

  //second
  for(int j=0;j<(split_number-1);j++){
    //secondのサブゴールを入力
    sub_goal_second_x[j] = sub_goal_x[j] - (control_point * cos(sub_goal_rad[j]));
    sub_goal_second_y[j] = sub_goal_y[j] - (control_point * sin(sub_goal_rad[j]));
  }
  //最終地点を入力
  sub_goal_second_x[split_number-1] = target_x_second;
  sub_goal_second_y[split_number-1] = target_y_second;


  for(int k=0;k<split_number;k++){
    //ROS_INFO("sub_goal_first_x[%d]=%f",k,sub_goal_first_x[k]);
    //ROS_INFO("sub_goal_first_y[%d]=%f",k,sub_goal_first_y[k]);
  }
  //番号を初期化
  first_number = -1;
  old_first_number = -1;
  second_number = -1;
  old_second_number = -1;

  //位置データと番号を入力
  send_target_point();

  //目標スピードを入力
  send_machine_speed();

  //データを送信
  pub_target_point_first.publish(sub_goal_first);
  pub_target_point_second.publish(sub_goal_second);

  //マーカを送信
  send_target_marker();

}
//目標地点を入力
void two_amcl_path_plan::send_target_point(void){
  //first機体の次の目標位置を入力
  if((-2<first_number) && (first_number<(split_number-1))){
    sub_goal_first.pose.pose.position.x=sub_goal_first_x[first_number+1];
    sub_goal_first.pose.pose.position.y=sub_goal_first_y[first_number+1];
    sub_goal_first.pose.pose.position.z=(first_number+1);
  }
  //second機体の次の目標位置を入力
  if((-2<second_number) && (second_number<(split_number-1))){
    sub_goal_second.pose.pose.position.x=sub_goal_second_x[second_number+1];
    sub_goal_second.pose.pose.position.y=sub_goal_second_y[second_number+1];
    sub_goal_second.pose.pose.position.z=(second_number+1);
  }
}

//目標位置へ到達するときの機体の速度を計算
void two_amcl_path_plan::send_machine_speed(void){
  //変数定義
  double first_judge = 0;
  double speed_first = 0;
  static double old_speed_first = 0;
  double second_judge = 0;
  double speed_second = 0;
  static double old_speed_second = 0;

  //機体の位置を更新
  update_odom_first();
  update_odom_second();

  //それぞれの機体の位置からの目標地点までの距離を計算
  //サブゴール
  double a=sub_goal_first_x[first_number+1]-world_offset_position_x_first;
  double b=sub_goal_first_y[first_number+1]-world_offset_position_y_first;
  double c=sub_goal_second_x[second_number+1]-world_offset_position_x_second;
  double d=sub_goal_second_y[second_number+1]-world_offset_position_y_second;

  double target_distance_first = sqrt((pow(a,2)) + (pow(b,2)));
  double target_distance_second = sqrt((pow(c,2)) + (pow(d,2)));


  //viが実数解を持つためのtiを決める
  int time = 1;
  for(;time<100;time++){
    //条件判定式
    first_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_first)-(2*acc_max*target_distance_first);
    second_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_second)-(2*acc_max*target_distance_second);
    //条件判定でどちらも正になるtimeがわかればそれを使う
    if((first_judge>0)&&(second_judge>0)){
      break;
    }
  }

  //viを計算
  while(1){
    speed_first = old_speed_first + (acc_max * time) - sqrt(first_judge);
    speed_second = old_speed_second + (acc_max * time) - sqrt(second_judge);

    //viMaxを超えているかを判定
    if((speed_first < Max_speed) && (speed_second < Max_speed)){
      break;
    }else{
      //もう一度timeを増やして計算
      time++;
      first_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_first)-(2*acc_max*target_distance_first);
      second_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_second)-(2*acc_max*target_distance_second);
    }
  }

  //速度を代入
  sub_goal_first.twist.twist.linear.z=speed_first;
  sub_goal_second.twist.twist.linear.z=speed_second;

  //速度の保持
  old_speed_first = speed_first;
  old_speed_second = speed_second;
}

//firstがサブゴールに到達したら実行する
void two_amcl_path_plan::Arrive_position_first(const nav_msgs::Odometry::ConstPtr &position){
  //到達位置を取得
  nav_msgs::Odometry first_position= *position;

  //今何番目のサブゴールかを判定
  first_number = first_position.pose.pose.position.z;

  //マシンにサブゴールを更新するデータを送るかのフラグ
  int send_frag = 0;
  //0~(split_number-2)番目なら，次のサブゴールをセット.
  //(split_number-1)番目は最終地点へ到達したとして次の目的地の更新はせず，old_first_numberとold_speed_secondを0に初期化
  //その他ならエラー
  if((-1<first_number) && (first_number < (split_number-1))){
    send_frag = 1;
  }else if(first_number == (split_number-1)){
    ROS_INFO("End");
  }else{
    ROS_INFO("Error");
  }

  //前回のフラグと比較して実行するか判定
  if((first_number == second_number) && (send_frag == 1) && (old_first_number != first_number)){
    //目標位置を入力
    send_target_point();
    //目標スピードを入力
    send_machine_speed();
    //データを送信
    pub_target_point_first.publish(sub_goal_first);
    pub_target_point_second.publish(sub_goal_second);

    //フラグ保持
    old_first_number = first_number;
    old_second_number = second_number;
  }else if((first_number == second_number) && (first_number == (split_number-1)) && (old_first_number != first_number)){
    //両方の機体が最終到達地点についたとしてold_speed_firstとold_speed_secondを初期化
    //変数初期化
    old_speed_first=0;
    old_speed_second=0;
    //フラグ保持
    old_first_number = first_number;
    old_second_number = second_number;
  }
}

//secondがサブゴールに到達したら実行する
void two_amcl_path_plan::Arrive_position_second(const nav_msgs::Odometry::ConstPtr &position){
  //到達位置を取得
  nav_msgs::Odometry second_position= *position;

  //今何番目のサブゴールかを判定
  second_number = second_position.pose.pose.position.z;

  //マシンにサブゴールを更新するデータを送るかのフラグ
  int send_frag = 0;
  //0~(split_number-2)番目なら，次のサブゴールをセット.
  //(split_number-1)番目は最終地点へ到達したとして次の目的地の更新はせず，old_second_numberとold_speed_secondを0に初期化
  //その他ならエラー
  if((-1<second_number) && (second_number < (split_number-1))){
    send_frag = 1;
  }else if(second_number == (split_number-1)){
    ROS_INFO("End");
  }else{
    ROS_INFO("Error");
  }
  //前回のフラグと比較して実行するか判定
  if((first_number == second_number) && (send_frag == 1) && (old_second_number != second_number)){
    //目標位置を入力
    send_target_point();
    //目標スピードを入力
    send_machine_speed();
    //データを送信
    pub_target_point_first.publish(sub_goal_first);
    pub_target_point_second.publish(sub_goal_second);

    //フラグ保持
    old_first_number = first_number;
    old_second_number = second_number;
  }else if((first_number == second_number) && (second_number == (split_number-1)) && (old_second_number != second_number)){
    //両方の機体が最終到達地点についたとしてold_speed_firstとold_speed_secondを初期化
    //変数初期化
    old_speed_first=0;
    old_speed_second=0;
    //フラグ保持
    old_first_number = first_number;
    old_second_number = second_number;
  }
}





void two_amcl_path_plan::send_target_marker(void){
  //control_target_positionの目標位置をマークする
  for(int i=0;i<split_number;i++){
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
  for(int i=0;i<split_number;i++){
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
	ros::init(argc, argv, "two_amcl_path_plan");
	two_amcl_path_plan two_amcl_path_plan;
	ros::spin();
	return 0;
}
