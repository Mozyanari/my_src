#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>



class icart_time{
public:
  icart_time();
private:
  //コールバック定義
  //ターゲットポジション取得
  void target_odom(const nav_msgs::Odometry::ConstPtr &target);

  //機体のodomを取得
  void cb_odom(const nav_msgs::Odometry::ConstPtr &data);

  //機体のオフセットポジションを取得
  void cb_offset_position(const geometry_msgs::Pose2D::ConstPtr &msg);

  //目標時間と位置から速度を計算
  void calc_machine_speed(void);

  //ターゲットポジションと現在のオフセットポジションからPIDによる位置制御，またフラグ処理
  void calc_wheel_speed(void);

  //PC側からの目標位置の更新とフラグ処理を行う
  void receive_target_point(const nav_msgs::Odometry::ConstPtr &data);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  //0.1秒ごとに速度更新するための関数
  void pub_speed(const ros::TimerEvent&);

  //時間の関数作成
  ros::Timer timer;

/*
使用するTopicの定義
*/
  //使用するpub/sebの定義
  ros::Subscriber sub_odom;
  ros::Subscriber sub_offset_position;
  ros::Subscriber sub_flag_receive;
  ros::Publisher pub_vel;
  ros::Publisher pub_odom_true;

//

  //フラグデータ(ポジションのx,yに到達した場所，zにフラグデータを挿入)
  nav_msgs::Odometry flag;

  //機体に送信するデータ
  geometry_msgs::Twist twist;

  //機体パラメータ
  //機体のオドメトリデータの受信
  nav_msgs::Odometry odom;
  //オフセット位置更新前の位置データ
  nav_msgs::Odometry old_odom;

  //amclによる機体のオフセット位置のデータ受信
  geometry_msgs::Pose2D odom_offset;

  //オドメトリを用いて修正した位置のデータ送信
  geometry_msgs::Pose2D odom_offset_true;

  //目標地点データを受信
  nav_msgs::Odometry target_point;

  //タイヤの半径[mm]
  double r;

  //トレッド距離[mm]
  double d;

  //機体の姿勢角度[rad]
  double rad;

  //最高速度[m/s]
  double Max_speed;

  //制限加速度
  double Reg_acc;

  //機体の目標位置
  double set_target_x;
  double set_target_y;

  //機体の到達目標時間
  double set_target_time;
  //機体の目標速度
  double set_target_speed;
  //機体の一つ前の目標速度
  double old_set_target_speed;

  //何番目のフラグ
  int get_frag;

  //連結距離
  double distance_multi;

  //それぞれの機体のオフセットとワールド座標を考慮した位置
  double world_offset_position_x;
  double world_offset_position_y;

  //オフセット距離
  double s;

  //シミュレーション時のオドメトリからオフセットまでの距離
  double ss;

  //目標位置
  double target_x;
  double target_y;
};

icart_time::icart_time(){
  //変数初期化
  //タイヤの半径[m]32.5mm
  r = 0.0325;
  //トレッド距離[m]160mm???????????????
  d = 0.178;
  //オフセット距離[m]160mm
  s = 0.16;
  //機体の角度
  rad = 0.0;

  //最高速度:0.1[m/s],100[mm/s]
  Max_speed = 0.02;

  //制限加速度[0.001]
  Reg_acc = 0.001;


  //機体間距離[m]570mm
  distance_multi = 0.57;

  //フラグは-1で初期化
  get_frag = -1;

  //速度も0で初期化
  set_target_speed = 0;
  old_set_target_speed = 0;

  //暴走を防ぐための初期化
  old_odom.pose.pose.orientation.w = 1.0;

  //購読するトピックの定義
  sub_odom=nh.subscribe("ypspur_ros/odom", 5, &icart_time::cb_odom,this);
  sub_offset_position=nh.subscribe("offset_position", 5, &icart_time::cb_offset_position,this);
  sub_flag_receive=nh.subscribe("target_point", 5, &icart_time::receive_target_point,this);
  //配布するトピックの定義
  pub_vel= nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1);
  pub_odom_true=nh.advertise<geometry_msgs::Pose2D>("offset_odom_true",1,true);

  //timer定義
  timer = nh.createTimer(ros::Duration(0.1), &icart_time::pub_speed,this);

}
//関数定義-----------------------------------------------------------------------
void icart_time::cb_odom(const nav_msgs::Odometry::ConstPtr &data){
  //データ受信
  odom = *data;

  //過去データとの差分を計算
  //amclで推定位置は分かるが更新頻度が少ないため，更新するまでの間はオドメトリで位置を補完する
  //まず，過去データからオフセット位置を計算
  double old_odom_theta = tf::getYaw(old_odom.pose.pose.orientation);
  double old_odom_x = old_odom.pose.pose.position.x - (s * cos(old_odom_theta));
  double old_odom_y = old_odom.pose.pose.position.y - (s * sin(old_odom_theta));

  //現在のオドメトリのオフセット位置を計算
  double odom_theta = tf::getYaw(odom.pose.pose.orientation);
  double odom_x = odom.pose.pose.position.x - (s * cos(odom_theta));
  double odom_y = odom.pose.pose.position.y - (s * sin(odom_theta));

  //オフセット位置の進み具合を計算
  double diff_odom_x = odom_x - old_odom_x;
  double diff_odom_y = odom_y - old_odom_y;
  double diff_odom_theta = odom_theta - old_odom_theta;

  //amclの推定位置とオドメトリからオフセット位置を修正
  world_offset_position_x = odom_offset.x + diff_odom_x;
  world_offset_position_y = odom_offset.y + diff_odom_y;
  rad = odom_offset.theta + diff_odom_theta;

  //world_offset_position_x = odom_offset.x;
  //world_offset_position_y = odom_offset.y;
  //rad = odom_offset.theta;

  //ROS_INFO("world_offset_position_x = %f",world_offset_position_x);
  //ROS_INFO("world_offset_position_y = %f",world_offset_position_y);
  //ROS_INFO("rad = %f",rad);

  //推定位置とオドメトリから修正した位置を送信
  odom_offset_true.x = world_offset_position_x;
  odom_offset_true.y = world_offset_position_y;
  odom_offset_true.theta = rad;
  pub_odom_true.publish(odom_offset_true);
  
  //目標時間と現在の位置からマシンの速度を計算
  //calc_target_speed();
  //ホイールの速度を生成
  //calc_wheel_speed();
}
void icart_time::cb_offset_position(const geometry_msgs::Pose2D::ConstPtr &msg){
  //機体の状態データ
  //機体のデータ受信
  odom_offset = *msg;

  //補間するためのodomを保存
  old_odom = odom;
}
//目標時間と現在の位置からマシンの速度を計算
void icart_time::calc_machine_speed(void){
    if(get_frag < 0){
        //fragの番号が負なら速度0
        set_target_speed = 0;
        old_set_target_speed = 0;
        return;
    }
    //目標時間と今の時間の差を計算
    double diff_time = (target_point.header.stamp - ros::Time::now()).toSec();
    ROS_INFO("diff_time=%f",diff_time);
    //もし，目標時間が現在時間より遅いとエラーと判定
    if(diff_time < 0){
        set_target_speed = 0;
        ROS_INFO("time_error");
        return;
    }

    //目標位置までの直線距離
    double diff_distance = sqrt( (pow((set_target_x - world_offset_position_x),2)) + (pow(set_target_y - world_offset_position_y,2)) );
    ROS_INFO("diff_distance=%f",diff_distance);
    //目標
    double a = 2*Reg_acc*diff_time*old_set_target_speed + (pow(Reg_acc,2))*(pow(diff_time,2)) - 2*Reg_acc*diff_distance;
    //ここが負になるのは目標時間内にこのままの制限加速度だと到着が不可能なので，満たす制限加速度にする
    if(a<0){
      //負になるのはサブゴールに近づいた時なので速度の更新を行わない
      a=0;
    }
    //目標時間で到達するための速度を計算
    double calc_target_speed = old_set_target_speed + (Reg_acc * diff_time) - sqrt(a);
    //double calc_target_speed = diff_distance/diff_time;
    ROS_INFO("calc_target_speed=%f",calc_target_speed);
    /*
    if((old_set_target_speed - calc_target_speed) > Reg_acc){
      calc_target_speed = old_set_target_speed - Reg_acc;
    }else if((calc_target_speed - old_set_target_speed) > Reg_acc){
      calc_target_speed = old_set_target_speed + Reg_acc;
    }
    */
    ROS_INFO("calc_target_speed=%f",calc_target_speed);
    //実機での想定最高スピードを超えないようにする
    if(Max_speed < calc_target_speed){
      calc_target_speed = Max_speed;
    }
    /*
    //速度更新の判定
    //直線距離が10cm以内ならもう速度を上げることは行わない
    if(diff_distance<0.10){
      //速度が一つ前より早いかどうかを判定
      if(old_set_target_speed < calc_target_speed){
        //早いなら更新なし
        set_target_speed = old_set_target_speed;
      }else{
        //遅いなら更新
        set_target_speed = calc_target_speed;
      }
    }else{
      //10cmより離れているなら速度の更新
      set_target_speed = calc_target_speed;
    }
    */
   set_target_speed = calc_target_speed;
    //次回使用する一つ前の速度を保存
    old_set_target_speed = set_target_speed;
}

//ホイール速度の計算と速度のpublish
void icart_time::calc_wheel_speed(void){
  //与える速度の計算
  //制御点と目標位置の差を計算
  double diff_x = (set_target_x - world_offset_position_x);
  double diff_y = (set_target_y - world_offset_position_y);

  //与えられた速度を今の位置からx,y方向に振り分ける計算
  //まずは，x,y方向の誤差から角度を計算
  double target_rad = atan2(diff_y,diff_x);

  //角度から速度をx,yに振り分け
  double x_vel = set_target_speed * cos(target_rad);
  double y_vel = set_target_speed * sin(target_rad);

    //左右の車輪の速度を計算
  //右の車輪計算
  double omega_r = (((cos(rad)+((d/s)*sin(rad)))*x_vel) + ((sin(rad)-(d/s)*cos(rad))*y_vel))/r;
  //左の車輪計算
  double omega_l = (((cos(rad)-((d/s)*sin(rad)))*x_vel) + ((sin(rad)+(d/s)*cos(rad))*y_vel))/r;

  //ROS_INFO("omega_r=%f",omega_r);
  //ROS_INFO("omega_l=%f",omega_l);

  //速度のpublish
  //車輪の計算からx方向とω方向の速度計算
  twist.linear.x  = (r*(omega_r + omega_l)) / 2;
  twist.angular.z = ((r*(omega_r - omega_l)) / (2*d));

  //twist.linear.x = x_vel * cos(rad) + y_vel * sin(rad);
  //twist.angular.z = 1/s * (x_vel * sin(rad) - y_vel * cos(rad));

  //関連する速度のデバック
  twist.angular.x = x_vel;
  twist.angular.y = y_vel;

  //twist.linear.y = omega_l;
  //twist.linear.z = omega_r;

  //計算したTopicを送る
  //ROS_INFO("get_frag=%d",get_frag);
  pub_vel.publish(twist);
  //ROS_INFO("x_flag=%d",x_flag);
  //ROS_INFO("y_flag=%d",y_flag);

}

//目標地点の更新とflag処理
void icart_time::receive_target_point(const nav_msgs::Odometry::ConstPtr &data){
  //目標地点データを受信
  target_point=*data;

  //目標地点を更新
  set_target_x=target_point.pose.pose.position.x;
  set_target_y=target_point.pose.pose.position.y;
  //フラグの番号を更新
  get_frag = target_point.pose.pose.position.z;
  //目標時間の受信はその都度計算する
  //set_target_time = target_point.header.stamp.toSec();

  //ROS_INFO("target_odom_update");
  //速度の更新
  calc_machine_speed();
}
void icart_time::pub_speed(const ros::TimerEvent&){
  //目標時間と現在の位置からマシンの速度を計算
  //calc_machine_speed();
  //ホイールの速度を生成
  calc_wheel_speed();
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "icart_time");
	icart_time icart_time;
	ros::spin();
	return 0;
}
