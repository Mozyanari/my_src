//サブゴールを何分割するか
#define devide_number 50

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include <iostream>



class path_planning{
public:
  path_planning();
private:
  //コールバック定義
  //それぞれの到達ポジションの取得
  void Arrive_position_first(const nav_msgs::Odometry::ConstPtr &position);
  void Arrive_position_second(const nav_msgs::Odometry::ConstPtr &position);

  //制御点の目標位置からそれぞれの機体の位置の算出
  void calc_machine_position(const nav_msgs::Odometry::ConstPtr &position);

  //目標位置へ到達するときの機体の速度を計算
  void calc_machine_speed(void);

  //機体からのオドメトリ取得
  void cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg);
  void cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg);

  //使用関数
  void send_target_point(void);
  void send_target_marker(void);



  //使用するTopicの定義
  ros::Subscriber sub_target_point;
  ros::Subscriber sub_odom_first;
  ros::Subscriber sub_odom_second;
  ros::Subscriber sub_arrive_position_first;
  ros::Subscriber sub_arrive_position_second;

  ros::Publisher pub_target_point_first;
  ros::Publisher pub_target_point_second;

  //Marker_define
  ros::Publisher marker_pub;




  //ノードハンドラ作成
	ros::NodeHandle nh;

  //現在の機体位置
  nav_msgs::Odometry odom_first;
  nav_msgs::Odometry odom_second;

  //機体のロボット座標における位置
  double position_x_first;
  double position_y_first;
  double rad_first;
  double position_x_second;
  double position_y_second;
  double rad_second;

  //現在の機体の世界座標におけるオフセット位置
  double world_offset_position_x_first;
  double world_offset_position_y_first;
  double world_offset_position_x_second;
  double world_offset_position_y_second;


  //現在のの制御点の位置
  double now_control_point_x;
  double now_control_point_y;
  double now_control_point_rad;

  //目標の制御点の位置
  nav_msgs::Odometry control_target_position;
  double target_control_point_x;
  double target_control_point_y;
  double target_control_point_rad;

  //目標のfirstの位置
  double target_x_first;
  double target_y_first;

  //目標のsecondの位置
  double target_x_second;
  double target_y_second;

  //機体間距離
  double distance_multi;

  //制御点までの距離
  double control_point;

  //オフセット距離
  double s;

  //機体の最高加速度
  double acc_max;

  //世界座標系におけるサブゴールの位置の配列
  //制御点に関して
  double sub_goal_x[devide_number];
  double sub_goal_y[devide_number];
  double sub_goal_rad[devide_number];

  //first機体に関して
  double sub_goal_first_x[devide_number];
  double sub_goal_first_y[devide_number];

  //second機体に関して
  double sub_goal_second_x[devide_number];
  double sub_goal_second_y[devide_number];

  //データ送信のためのサブゴールデータ変数
  nav_msgs::Odometry sub_goal_first;
  nav_msgs::Odometry sub_goal_second;

  //今何番目のサブゴールに到達したかの判定番号
  int first_number;
  int second_number;

  //それぞれの機体の目標速度
  double sub_goal_speed_first;
  double sub_goal_speed_second;



  //Marker_data
  visualization_msgs::Marker marker_control;
  visualization_msgs::Marker marker_first;
  visualization_msgs::Marker marker_second;

};

path_planning::path_planning(){
  //機体パラメータ
  //機体間距離[m]570mm
  distance_multi = 0.57;

  //制御点までの距離
  control_point = distance_multi / 2;

  //オフセット距離[m]160mm
  s = 0.16;

  //機体の最高加速度:80[mm/s^2]
  acc_max = 0.08;

  //サブゴールの最小距離[0.5m]
  //min_goal_length = 0.5;

  //odomのクオータニオンを初期化
  odom_first.pose.pose.orientation.w = 1.0;
  odom_second.pose.pose.orientation.w = 1.0;

  //購読するトピックの定義
  sub_target_point = nh.subscribe("/target_point", 5, &path_planning::calc_machine_position,this);
  sub_odom_first = nh.subscribe("/ypspur_ros_first/odom", 5, &path_planning::cb_odom_first,this);
  sub_arrive_position_first=nh.subscribe("/frag_data_first", 5, &path_planning::Arrive_position_first,this);

  sub_odom_second = nh.subscribe("/ypspur_ros_second/odom", 5, &path_planning::cb_odom_second,this);
  sub_arrive_position_second=nh.subscribe("/frag_data_second", 5, &path_planning::Arrive_position_second,this);


  //配布するトピックの定義
  pub_target_point_first = nh.advertise<nav_msgs::Odometry>("target_point_first", 1);
  pub_target_point_second = nh.advertise<nav_msgs::Odometry>("target_point_second", 1);

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

}

//関数定義-----------------------------------------------------------------------
//cb_odom_first関数定義
void path_planning::cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg){
  //first機体の状態受信
  odom_first = *msg;

  //first機体に関する計算----------------------------------------------------------
  //first機体の角度を代入
  rad_first = tf::getYaw(odom_first.pose.pose.orientation);
  //first機体の位置を代入
  position_x_first = odom_first.pose.pose.position.x;
  position_y_first = odom_first.pose.pose.position.y;
  //first機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_first = (position_x_first + s + distance_multi) - (s * cos(rad_first));
  world_offset_position_y_first = position_y_first - (s * sin(rad_first));
}

//cb_odom_second関数定義
void path_planning::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg){
  //機体の状態受信
  odom_second = *msg;

  //secon機体についての計算---------------------------------------------------------
  //second機体の角度を代入
  rad_second = tf::getYaw(odom_second.pose.pose.orientation);
  //second機体の位置を代入
  position_x_second = odom_second.pose.pose.position.x;
  position_y_second = odom_second.pose.pose.position.y;
  //seccond機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_second = (position_x_second + s) - (s * cos(rad_second));
  world_offset_position_y_second = position_y_second - (s * sin(rad_second));
}


//制御点の目標位置が更新された時にそれぞれの機体のサブゴールを作成
void path_planning::calc_machine_position(const nav_msgs::Odometry::ConstPtr &position){
  //目標位置を取得
  control_target_position = *position;

  //目標の(x,y,θ)位置を取得
  target_control_point_x=control_target_position.pose.pose.position.x;
  target_control_point_y=control_target_position.pose.pose.position.y;
  target_control_point_rad=tf::getYaw(control_target_position.pose.pose.orientation);
  ROS_INFO("target_control_point_rad=%f",target_control_point_rad);

  //最終的な目標とするfirstとsecondのそれぞれの位置を計算
  //first
  target_x_first = target_control_point_x + (control_point * cos(target_control_point_rad));
  target_y_first = target_control_point_y + (control_point * sin(target_control_point_rad));

  //second
  target_x_second = target_control_point_x - (control_point * cos(target_control_point_rad));
  target_y_second = target_control_point_y - (control_point * sin(target_control_point_rad));

  //現在の制御点の位置と姿勢を取得
  now_control_point_x = (world_offset_position_x_first + world_offset_position_x_second)/2;
  now_control_point_y = (world_offset_position_y_first + world_offset_position_y_second)/2;
  now_control_point_rad = asin((world_offset_position_y_first - world_offset_position_y_second) / distance_multi);

  //現在の制御点と目標の制御点までの違いを計算
  double diff_machine_position_x = target_control_point_x - now_control_point_x;
  double diff_machine_position_y = target_control_point_y - now_control_point_y;

  double diff_machine_rad = target_control_point_rad - now_control_point_rad;

  //それぞれのサブゴールの設定
  //今回は5つのサブゴール作成
  //サブゴールの間隔を計算
  double among_sub_goal_x=diff_machine_position_x/(devide_number+1);
  double among_sub_goal_y=diff_machine_position_y/(devide_number+1);
  double among_sub_goal_rad=diff_machine_rad/(devide_number+1);

  //制御点のサブゴールの位置の計算
  for(int i=0;i<devide_number;i++){
    sub_goal_x[i]=now_control_point_x + (among_sub_goal_x * (i+1));
    sub_goal_y[i]=now_control_point_y + (among_sub_goal_y * (i+1));
    sub_goal_rad[i]=now_control_point_rad + (among_sub_goal_rad *(i+1));
  }

  //制御点のサブゴールからわかる，それぞれの機体のサブゴールの位置
  for(int j=0;j<devide_number;j++){
    //first
    sub_goal_first_x[j] = sub_goal_x[j] + control_point * cos(sub_goal_rad[j]);
    sub_goal_first_y[j] = sub_goal_y[j] + control_point * sin(sub_goal_rad[j]);

    //second
    sub_goal_second_x[j] = sub_goal_x[j] - control_point * cos(sub_goal_rad[j]);
    sub_goal_second_y[j] = sub_goal_y[j] - control_point * sin(sub_goal_rad[j]);
  }



  //デバック
  for(int k=0;k<devide_number;k++){
    ROS_INFO("sub_goal_x[%d]=%f",k,sub_goal_x[k]);
    ROS_INFO("sub_goal_y[%d]=%f",k,sub_goal_y[k]);
  }
  for(int k=0;k<devide_number;k++){
    ROS_INFO("sub_goal_first_x[%d]=%f",k,sub_goal_first_x[k]);
    ROS_INFO("sub_goal_first_y[%d]=%f",k,sub_goal_first_y[k]);
  }
  for(int k=0;k<devide_number;k++){
    ROS_INFO("sub_goal_second_x[%d]=%f",k,sub_goal_second_x[k]);
    ROS_INFO("sub_goal_second_y[%d]=%f",k,sub_goal_second_y[k]);
  }

  //１つめの目標位置を送信
  //firstの１つめのデータをセット
  sub_goal_first.pose.pose.position.x = sub_goal_first_x[0];
  sub_goal_first.pose.pose.position.y = sub_goal_first_y[0];
  sub_goal_first.pose.pose.position.z = 0;

  //secondの１つめのデータをセット
  sub_goal_second.pose.pose.position.x = sub_goal_second_x[0];
  sub_goal_second.pose.pose.position.y = sub_goal_second_y[0];
  sub_goal_second.pose.pose.position.z = 0;

  //データを送信
  send_target_point();

  //マーカを送信
  send_target_marker();

}

//firstがサブゴールに到達したら実行する
//到達したサブゴールを判定して，secondとの比較を行いながら，次のサブゴールを送信する
//最終地点に到達したなら，サブゴールを送信しない
void path_planning::Arrive_position_first(const nav_msgs::Odometry::ConstPtr &position){
  //到達位置を取得
  nav_msgs::Odometry first_position= *position;
  //今何番目のサブゴールかを判定
  //最終地点なら6とする
  first_number = first_position.pose.pose.position.z;

  //何番目についたかデバック
  ROS_INFO("First_Arrive_%d",first_number);

  //マシンにサブゴールを更新するデータを送るかのフラグ
  int send_frag = 0;
  //0~(devide_number-2)番目なら，次のサブゴールをセット.(devide_number-1)番目なら最終到達地点へセット,(devide_number)番目は最終地点へ到達したとして次の目的地の更新はしない
  //その他ならエラー
  if((-1<first_number) && (first_number<=(devide_number-1))){
    send_frag = 1;
  }else if(first_number == devide_number){
    ROS_INFO("End");
  }else{
    ROS_INFO("Error");
  }

  //secondと比較してnumberが同じならどちらも同じ番号のゴールにたどり着いたとして次の目的地を送信する
  //firstと比較してnumberが同じならどちらも同じ番号のゴールにたどり着いたとして次の目的地を送信する
  if((first_number == second_number) && (send_frag ==1)){
    //目標位置を入力
    send_target_point();
    //目標スピードを入力
    calc_machine_speed();

    //データを送信
    pub_target_point_first.publish(sub_goal_first);
    pub_target_point_second.publish(sub_goal_second);

    ROS_INFO("send_next_point");
  }
}

void path_planning::Arrive_position_second(const nav_msgs::Odometry::ConstPtr &position){
  //到達位置を取得
  nav_msgs::Odometry second_position= *position;
  //今何番目のサブゴールかを判定
  second_number = second_position.pose.pose.position.z;

  //何番目についたかデバック
  ROS_INFO("Second_Arrive_%d",second_number);

  //マシンにサブゴールを更新するデータを送るかのフラグ
  int send_frag = 0;

  //0~(devide_number-2)番目なら，次のサブゴールをセット.(devide_number-1)番目なら最終到達地点へセット,(devide_number)番目は最終地点へ到達したとして次の目的地の更新はしない
  //その他ならエラー
  if((-1<second_number) && (second_number<=(devide_number-1))){
    send_frag = 1;
  }else if(second_number == devide_number){
    ROS_INFO("End");
  }else{
    ROS_INFO("Error");
  }
  //firstと比較してnumberが同じならどちらも同じ番号のゴールにたどり着いたとして次の目的地を送信する
  if((first_number == second_number) && (send_frag ==1)){
    //目標位置を入力
    send_target_point();
    //目標スピードを入力
    calc_machine_speed();

    //データを送信
    pub_target_point_first.publish(sub_goal_first);
    pub_target_point_second.publish(sub_goal_second);

    ROS_INFO("send_next_point");
  }
}

//目標地点をfirst,secondともに送信する
void path_planning::send_target_point(void){
  //first機体の目標位置を入力
  if((-1<first_number) && (first_number<(devide_number-1))){
    //サブゴールを入力
    sub_goal_first.pose.pose.position.x=sub_goal_first_x[first_number+1];
    sub_goal_first.pose.pose.position.y=sub_goal_first_y[first_number+1];
    sub_goal_first.pose.pose.position.z=(first_number+1);
  }else if(first_number == (devide_number-1)){
    //最終到達地点を入力
    sub_goal_first.pose.pose.position.x=target_x_first;
    sub_goal_first.pose.pose.position.y=target_y_first;
    sub_goal_first.pose.pose.position.z=devide_number;
  }


  //second機体の目標位置を入力
  if((-1<second_number) && (second_number<(devide_number-1))){
    //サブゴールを入力
    sub_goal_second.pose.pose.position.x=sub_goal_second_x[second_number+1];
    sub_goal_second.pose.pose.position.y=sub_goal_second_y[second_number+1];
    sub_goal_second.pose.pose.position.z=(second_number+1);
  }else if(second_number == (devide_number-1)){
    //最終到達地点を入力
    sub_goal_second.pose.pose.position.x=target_x_second;
    sub_goal_second.pose.pose.position.y=target_y_second;
    sub_goal_second.pose.pose.position.z=devide_number;
  }
}

//目標位置へ到達するときの機体の速度を計算
void path_planning::calc_machine_speed(void){
  static double old_speed_first=0;
  static double old_speed_second=0;

  //それぞれの機体の位置からの目標地点までの距離を計算
  double target_distance_first = sqrt( (pow((sub_goal_first_x[first_number+1]-world_offset_position_x_first),2)) + (pow((sub_goal_first_y[first_number+1]-world_offset_position_y_first),2)) );
  double target_distance_second = sqrt( (pow((sub_goal_second_x[second_number+1]-world_offset_position_x_second),2)) + (pow((sub_goal_second_y[second_number+1]-world_offset_position_y_second),2)) );

  //viが実数解を持つためのtiを決める
  time = 1;
  for(;time<100;time++){
    //条件判定式
    double first_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_first)-(2*acc_max*target_distance_first);
    double second_judge = pow((acc_max*time),2)+(2*acc_max*time*old_speed_second)-(2*acc_max*target_distance_second);
    //条件判定でどちらも正になるtimeがわかればそれを使う
    if((first_judge>0)&&(second_judge>0)){
      break;
    }
  }
  while(1){
    //viを計算
    speed_first = old_speed_first + (acc_max * time) - sqrt(first_judge);
    speed_second = old_speed_second + (acc_max * time) - sqrt(second_judge);

    //viMaxを超えているかを判定
    if((speed_first < Max_speed) && (speed_second < Max_speed)){
      break;
    }else{
      //もう一度timeを増やして計算
      time++;
    }
  }

  //速度の保持
  old_speed_first = speed_first;
  old_speed_second = speed_second;

}

void path_planning::send_target_marker(void){
  //制御点の目標位置とサブゴールをマークする
  //サブゴールから
  for(int i=0;i<devide_number;i++){
    std::ostringstream ss;
    ss << i;
    marker_control.header.frame_id = "/world";
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

    marker_control.scale.x = 0.1;
    marker_control.scale.y = 0.1;
    marker_control.scale.z = 0.1;

    marker_control.color.r = 0.0f;
    marker_control.color.g = 1.0f;
    marker_control.color.b = 0.0f;
    marker_control.color.a = 0.8;

    marker_control.lifetime = ros::Duration();

    marker_pub.publish(marker_control);

    ros::Duration(0.01).sleep();

    //ROS_INFO("i=%d",i);
  }
  //最終目標位置をマーク
  marker_control.header.frame_id = "/world";
  marker_control.header.stamp = ros::Time::now();
  marker_control.ns = ("control_marker_target_point");
  marker_control.id = 0;
  marker_control.type = visualization_msgs::Marker::SPHERE;
  marker_control.action = visualization_msgs::Marker::ADD;
  marker_control.pose.position.x = target_control_point_x;
  marker_control.pose.position.y = target_control_point_y;
  marker_control.pose.position.z = 0;
  marker_control.pose.orientation.x = 0.0;
  marker_control.pose.orientation.y = 0.0;
  marker_control.pose.orientation.z = 0.0;
  marker_control.pose.orientation.w = 1.0;

  marker_control.scale.x = 0.2;
  marker_control.scale.y = 0.2;
  marker_control.scale.z = 0.2;

  marker_control.color.r = 0.0f;
  marker_control.color.g = 1.0f;
  marker_control.color.b = 0.0f;
  marker_control.color.a = 1.0;

  marker_control.lifetime = ros::Duration();

  marker_pub.publish(marker_control);

  ros::Duration(0.01).sleep();

  //firstの目標位置とサブゴールをマークする
  //サブゴールから
  for(int i=0;i<devide_number;i++){
    std::ostringstream ss;
    ss << i;
    marker_control.header.frame_id = "/world";
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

    marker_control.scale.x = 0.1;
    marker_control.scale.y = 0.1;
    marker_control.scale.z = 0.1;

    marker_control.color.r = 1.0f;
    marker_control.color.g = 0.0f;
    marker_control.color.b = 0.0f;
    marker_control.color.a = 0.8;

    marker_control.lifetime = ros::Duration();

    marker_pub.publish(marker_control);

    ros::Duration(0.01).sleep();

    //ROS_INFO("i=%d",i);
  }
  //最終目標位置をマーク
  marker_control.header.frame_id = "/world";
  marker_control.header.stamp = ros::Time::now();
  marker_control.ns = ("first_marker_target_point");
  marker_control.id = 0;
  marker_control.type = visualization_msgs::Marker::SPHERE;
  marker_control.action = visualization_msgs::Marker::ADD;
  marker_control.pose.position.x = target_x_first;
  marker_control.pose.position.y = target_y_first;
  marker_control.pose.position.z = 0;
  marker_control.pose.orientation.x = 0.0;
  marker_control.pose.orientation.y = 0.0;
  marker_control.pose.orientation.z = 0.0;
  marker_control.pose.orientation.w = 1.0;

  marker_control.scale.x = 0.2;
  marker_control.scale.y = 0.2;
  marker_control.scale.z = 0.2;

  marker_control.color.r = 1.0f;
  marker_control.color.g = 0.0f;
  marker_control.color.b = 0.0f;
  marker_control.color.a = 1.0;

  marker_control.lifetime = ros::Duration();

  marker_pub.publish(marker_control);

  ros::Duration(0.01).sleep();

  //secondの目標位置とサブゴールをマークする
  //サブゴールから
  for(int i=0;i<devide_number;i++){
    std::ostringstream ss;
    ss << i;
    marker_control.header.frame_id = "/world";
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

    marker_control.scale.x = 0.1;
    marker_control.scale.y = 0.1;
    marker_control.scale.z = 0.1;

    marker_control.color.r = 0.0f;
    marker_control.color.g = 0.0f;
    marker_control.color.b = 1.0f;
    marker_control.color.a = 0.8;

    marker_control.lifetime = ros::Duration();

    marker_pub.publish(marker_control);

    ros::Duration(0.01).sleep();

    //ROS_INFO("i=%d",i);
  }
  //最終目標位置をマーク
  marker_control.header.frame_id = "/world";
  marker_control.header.stamp = ros::Time::now();
  marker_control.ns = ("second_marker_target_point");
  marker_control.id = 0;
  marker_control.type = visualization_msgs::Marker::SPHERE;
  marker_control.action = visualization_msgs::Marker::ADD;
  marker_control.pose.position.x = target_x_second;
  marker_control.pose.position.y = target_y_second;
  marker_control.pose.position.z = 0;
  marker_control.pose.orientation.x = 0.0;
  marker_control.pose.orientation.y = 0.0;
  marker_control.pose.orientation.z = 0.0;
  marker_control.pose.orientation.w = 1.0;

  marker_control.scale.x = 0.2;
  marker_control.scale.y = 0.2;
  marker_control.scale.z = 0.2;

  marker_control.color.r = 0.0f;
  marker_control.color.g = 0.0f;
  marker_control.color.b = 1.0f;
  marker_control.color.a = 1.0;

  marker_control.lifetime = ros::Duration();

  marker_pub.publish(marker_control);

  ros::Duration(0.001).sleep();
}








//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning");
	path_planning path_planning;
	ros::spin();
	return 0;
}
