#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


class icart_move{
public:
  icart_move();
private:
  //コールバック定義
  //ターゲットポジション取得
  void target_odom(const nav_msgs::Odometry::ConstPtr &target);

  //機体のodomを取得
  void cb_odom(const nav_msgs::Odometry::ConstPtr &msg);

  //ターゲットポジションと現在のodomからPIDによる位置制御，またフラグ処理
  void calc_speed(void);

  //PC側からの目標位置の更新とフラグ処理を行う
  void receive_target_point(const nav_msgs::Odometry::ConstPtr &data);

  //ノードハンドラ作成
	ros::NodeHandle nh;

  //使用するpub/sebの定義
  ros::Subscriber sub_odom;
  ros::Subscriber sub_flag_receive;

  ros::Publisher pub_vel;
  ros::Publisher pub_flag_publish;

  //機体の状態データ
  nav_msgs::Odometry odom;

  //フラグデータ(ポジションのx,yに到達した場所，zにフラグデータを挿入)
  nav_msgs::Odometry flag;

  //機体に送信するデータ
  geometry_msgs::Twist twist;

  //機体パラメータ
  //速度
  double x_vel;
  double y_vel;
  double omega_vel;

  //タイヤの半径[mm]
  double r;

  //トレッド距離[mm]
  double d;

  //機体の姿勢角度[rad]
  double rad;

  //機体のロボット座標における位置
  double position_x;
  double position_y;

  //機体の目標位置
  double set_target_x;
  double set_target_y;
  //何番目のフラグ
  int get_frag;

  //連結距離
  double distance_multi;

  //それぞれの機体のオフセットとワールド座標を考慮した位置
  //secondのオフセット位置が(0.0)になるように設定
  double world_offset_position_x;
  double world_offset_position_y;

  //左右の車輪の速度
  double omega_l, omega_r;

  //オフセット距離
  double s;

  //目標位置
  double target_x;
  double target_y;

  //フラグ
  int x_flag;
  int y_flag;
};

icart_move::icart_move(){
  //変数初期化
  //タイヤの半径[m]32.5mm
  r = 0.0325;
  //トレッド距離[m]160mm???????????????
  d = 0.178;
  //オフセット距離[m]160mm
  s = 0.16;
  //機体の角度
  rad = 0.0;

  //機体間距離[m]570mm
  distance_multi = 0.57;

  //フラグの初期化
  //フラグは最初は1として動作不可にする
  x_flag=1;
  y_flag=1;

  //odomのクオータニオンを初期化
  odom.pose.pose.orientation.w = 1.0;

  //購読するトピックの定義
  sub_odom= nh.subscribe("/ypspur_ros_second/odom", 5, &icart_move::cb_odom,this);
  sub_flag_receive=nh.subscribe("/target_point_second", 5, &icart_move::receive_target_point,this);
  //配布するトピックの定義
  pub_vel= nh.advertise<geometry_msgs::Twist>("/ypspur_ros_second/cmd_vel", 1);
  pub_flag_publish=nh.advertise<nav_msgs::Odometry>("/frag_data_second", 1);

}
//関数定義-----------------------------------------------------------------------
void icart_move::cb_odom(const nav_msgs::Odometry::ConstPtr &msg){
  //機体のデータ受信
  odom = *msg;

  //機体に関する計算----------------------------------------------------------
  //機体の角度を代入
  rad = tf::getYaw(odom.pose.pose.orientation);
  //second機体の位置を代入
  position_x = odom.pose.pose.position.x;
  position_y = odom.pose.pose.position.y;

  //機体のworld座標におけるオフセット位置を計算(second)
  world_offset_position_x = (position_x + s) - (s * cos(rad));
  world_offset_position_y = position_y - (s * sin(rad));

  //機体に与える速度に関する計算と速度の送信，到達したかの判定
  calc_speed();
}

//速度の計算と速度のpublish
void icart_move::calc_speed(void){
  //与える速度の計算
  //制御点と目標位置の差を計算
  double diff_x = (set_target_x - world_offset_position_x);
  double diff_y = (set_target_y - world_offset_position_y);

  //位置のズレが0.01[m]以上なら速度を出す
  //0.01以内なら到達したと判定し，それぞれのフラグを1とする

  if(diff_x > 0.01){
    x_vel = 0.1;
  }else if(diff_x < -0.01){
    x_vel = -0.1;
  }else{
    x_vel = 0;
    x_flag=1;
  }

  if(diff_y > 0.01){
    y_vel = 0.1;
  }else if(diff_y < -0.01){
    y_vel = -0.1;
  }else{
    y_vel = 0;
    y_flag=1;
  }
  if((x_flag==1) && (y_flag==1)){
    //フラグがONの限り速度は0
    x_vel = 0;
    y_vel = 0;

    //フラグをPC側に送信
    //今到達している場所を登録
    flag.pose.pose.position.x=set_target_x;
    flag.pose.pose.position.y=set_target_y;
    //何番目のフラグ処理をしたかを返す
    flag.pose.pose.position.z=get_frag;

    pub_flag_publish.publish(flag);
    ROS_INFO("Reach_target_point_second");

  }

  //左右の車輪の速度を計算
  //右の車輪計算
  omega_r = (((cos(rad)+((d/s)*sin(rad)))*x_vel) + ((sin(rad)-(d/s)*cos(rad))*y_vel))/r;
  //左の車輪計算
  omega_l = (((cos(rad)-((d/s)*sin(rad)))*x_vel) + ((sin(rad)+(d/s)*cos(rad))*y_vel))/r;

  //速度のpublish
  //車輪の計算からx方向とω方向の速度計算
  twist.linear.x  = (r*(omega_r + omega_l)) / 2;
  twist.angular.z = ((r*(omega_r - omega_l)) / (2*d));

  //計算したTopicを送る
  pub_vel.publish(twist);
  //ROS_INFO("x_flag=%d",x_flag);
  //ROS_INFO("y_flag=%d",y_flag);
}

//目標地点の更新とflag処理
void icart_move::receive_target_point(const nav_msgs::Odometry::ConstPtr &data){
  //目標地点データを受信
  nav_msgs::Odometry target_point=*data;

  //目標地点を更新
  set_target_x=target_point.pose.pose.position.x;
  set_target_y=target_point.pose.pose.position.y;

  //フラグを折る
  x_flag=0;
  y_flag=0;

  //フラグの番号を更新
  get_frag = target_point.pose.pose.position.z;

  ROS_INFO("target_odom_update");
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "icart_move");
	icart_move icart_move;
	ros::spin();
	return 0;
}
