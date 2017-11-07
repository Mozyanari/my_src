#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


//クラス定義(どのような構造か定義)-----------------------------------------------------
class multi_icart_with_ps3
{
  public:
    multi_icart_with_ps3();

  private:

  //コールバック定義
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  void cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg);
  void cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg);

  //関数定義
  //連結機体に関する計算
  void calc_linking_wheel_speed(void);
  void calc_linking_rad_multi(void);
  void calc_linking_position(void);

  //一秒ごとにデバックするための関数
  void debug(const ros::TimerEvent&);


  //ノードハンドラ作成
	ros::NodeHandle nh;


  //使用するpub/sebの定義
  ros::Subscriber sub_odom_first_;
  ros::Subscriber sub_odom_second_;

	ros::Publisher  pub_vel_first_;
  ros::Publisher  pub_vel_second_;

	ros::Subscriber sub_joy_;

  //使用パラメータ定義
	int vel_linear, vel_angular;
	double l_scale_, a_scale_;
  double default_target_position_x, default_target_position_y, default_target_position_rad;

  //機体の状態データ
  nav_msgs::Odometry odom_first;
  nav_msgs::Odometry odom_second;

  //機体に送信するデータ
  geometry_msgs::Twist twist_first;
  geometry_msgs::Twist twist_second;

  //時間の関数作成
  ros::Timer timer;

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
  double rad_first;
  double rad_second;
  double rad_multi;

  //機体のロボット座標における位置
  double position_x_first;
  double position_y_first;
  double position_x_second;
  double position_y_second;

  //それぞれの機体のオフセットとワールド座標を考慮した位置
  //secondのオフセット位置が(0.0)になるように設定
  double world_offset_position_x_first;
  double world_offset_position_y_first;
  double world_offset_position_x_second;
  double world_offset_position_y_second;

  //world座標における制御点の位置
  double world_control_point_x;
  double world_control_point_y;

  //左右の車輪の速度
  double omega_l_first, omega_r_first;
  double omega_l_second, omega_r_second;

  //オフセット距離
  double s;

  //機体間距離
  double distance_multi;

  //制御点までの距離
  double control_point;

};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
multi_icart_with_ps3::multi_icart_with_ps3():
//ps3コントローラパラメータ
vel_linear(3), vel_angular(0), a_scale_(4.0),l_scale_(2.0)
{

  //変数初期化
  //タイヤの半径[m]32.5mm
  r = 0.0325;
  //トレッド距離[m]160mm???????????????
  d = 0.178;
  //オフセット距離[m]160mm
  s = 0.16;
  //機体の角度
  rad_first = 0.0;
  rad_second = 0.0;

  rad_multi = 0.0;

  //機体の位置
  position_x_first = 0.0;
  position_y_first = 0.0;
  position_x_second = 0.0;
  position_y_second = 0.0;
  world_offset_position_x_first = 0.0;
  world_offset_position_y_first = 0.0;
  world_offset_position_x_second = 0.0;
  world_offset_position_y_second = 0.0;

  //目標に位置に関するパラメータ
  default_target_position_x=1.0;
  default_target_position_y=0.5;
  default_target_position_rad=0.0;

  //制御点のworld座標における位置
  world_control_point_x = 0.0;
  world_control_point_y = 0.0;


  //機体の車輪の速度
  omega_l_first = 0.0;
  omega_r_first = 0.0;
  omega_l_second = 0.0;
  omega_r_second = 0.0;

  //機体間距離[m]570mm
  distance_multi = 0.57;

  //制御点までの距離
  control_point = distance_multi / 2;


  //ps3コントローラに関するparam
	nh.param("axis_linear"  , vel_linear , vel_linear);
	nh.param("axis_angular" , vel_angular, vel_angular);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear" , l_scale_, l_scale_);

  //odomのクオータニオンを初期化
  odom_first.pose.pose.orientation.w = 1.0;
  odom_second.pose.pose.orientation.w = 1.0;

  //購読するトピックの定義
	sub_joy_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &multi_icart_with_ps3::joyCallback, this);
  sub_odom_first_ = nh.subscribe("/ypspur_ros_first/odom", 5, &multi_icart_with_ps3::cb_odom_first,this);
  sub_odom_second_ = nh.subscribe("/ypspur_ros_second/odom", 5, &multi_icart_with_ps3::cb_odom_second,this);

  //配布するトピックの定義
  pub_vel_first_ = nh.advertise<geometry_msgs::Twist>("/ypspur_ros_first/cmd_vel", 1);
  pub_vel_second_ = nh.advertise<geometry_msgs::Twist>("/ypspur_ros_second/cmd_vel", 1);

  //debugtimer定義
  timer = nh.createTimer(ros::Duration(1.0), &multi_icart_with_ps3::debug,this);

}
//関数定義-----------------------------------------------------------------------
//cb_odom_first関数定義
void multi_icart_with_ps3::cb_odom_first(const nav_msgs::Odometry::ConstPtr &msg){
  //first機体の状態受信
  odom_first = *msg;

  //機体に送る速度の計算

  //first機体に関する計算----------------------------------------------------------
  //first機体の角度を代入
  rad_first = tf::getYaw(odom_first.pose.pose.orientation);
  //first機体の位置を代入
  position_x_first = odom_first.pose.pose.position.x;
  position_y_first = odom_first.pose.pose.position.y;
  //first機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_first = (position_x_first + s + distance_multi) - (s * cos(rad_first));
  world_offset_position_y_first = position_y_first - (s * sin(rad_first));

  //連結機体に関する計算------------------------------------------------------------
  //連結機帯の制御点のworld座標における位置を計算
  calc_linking_position();
  //連結機体の角度を計算
  calc_linking_rad_multi();
  //左右の車輪の速度を計算
  calc_linking_wheel_speed();

  //計算したTopicを送る
  pub_vel_first_.publish(twist_first);
  pub_vel_second_.publish(twist_second);

  //
}

//cb_odom_second関数定義
void multi_icart_with_ps3::cb_odom_second(const nav_msgs::Odometry::ConstPtr &msg){
  //機体の状態受信
  odom_second = *msg;

  //機体に送る速度の計算

  //secon機体についての計算---------------------------------------------------------
  //second機体の角度を代入
  rad_second = tf::getYaw(odom_second.pose.pose.orientation);
  //second機体の位置を代入
  position_x_second = odom_second.pose.pose.position.x;
  position_y_second = odom_second.pose.pose.position.y;
  //seccond機体のworld座標におけるオフセット位置を計算
  world_offset_position_x_second = (position_x_second + s) - (s * cos(rad_second));
  world_offset_position_y_second = position_y_second - (s * sin(rad_second));


  //連結機体に関する計算------------------------------------------------------------
  //連結機帯の制御点のworld座標における位置を計算
  calc_linking_position();
  //連結機体の角度を計算
  calc_linking_rad_multi();
  //左右の車輪の速度を計算
  calc_linking_wheel_speed();

  //計算したTopicを送る
  pub_vel_first_.publish(twist_first);
  pub_vel_second_.publish(twist_second);


  //デバック
  //ROS_INFO("odom_second_come_on!!");
}

//Joycallback関数定義
void multi_icart_with_ps3::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  //ローカル変数定義

  //x,y方向の速度の更新
  x_vel = joy->axes[vel_linear]*0.1;
  y_vel = joy->axes[vel_angular]*0.1;

  if(joy->buttons[10]==1){
    omega_vel=-0.1;
  }else if(joy->buttons[11]==1){
    omega_vel=0.1;
  }else{
    omega_vel=0.0;
  }
  //omega_vel = joy->button[]

  //double path1 = (cos(rad)+((d/s)*sin(rad)))*x_vel;
  //double path2 = sin(rad)-(d/s)*cos(rad);

  /*
  twist.linear.x = l_scale_*joy->axes[vel_linear];
  twist.angular.z = a_scale_*joy->axes[vel_angular];
  */

  //デバック
  //ROS_INFO("r %f",r);
  //ROS_INFO("d %f",d);

  //ROS_INFO("s %f",s);
  //ROS_INFO("rad %f",rad);

  ROS_INFO("x_vel %f",x_vel);
  ROS_INFO("y_vel %f",y_vel);
  ROS_INFO("omega_vel %f",omega_vel);


  //ROS_INFO("path1 %f",path1);
  //ROS_INFO("path2 %f",path2);


  //ROS_INFO("omega_l %f",omega_l);
  //ROS_INFO("omega_r %f",omega_r);

  //ROS_INFO("vel %f",twist.linear.x);
  //ROS_INFO("angular %f",twist.angular.z);

  //ROS_INFO_STREAM("(" << joy->axes[vel_linear] << " " << joy->axes[vel_angular] << ")");

  //ROS_INFO("position_x_first %f",position_x_first);
  //ROS_INFO("position_y_first %f",position_y_first);
  //ROS_INFO("position_x_second %f",position_x_second);
  //ROS_INFO("position_y_second %f",position_y_second);

  //ROS_INFO("rad_multi %f",rad_multi);

}

//連結機体の計算に関する計算関数------------------------------------------------------
//連結機体それぞれの左右の車輪の速度を計算して、pubできる形にする
void multi_icart_with_ps3::calc_linking_wheel_speed(void){
  //左右の車輪の速度を計算
  //First機体の計算
  //右の車輪計算
  omega_r_first = (x_vel*(cos(rad_first)+(d/s)*sin(rad_first)) +
    y_vel*(sin(rad_first)-(d/s)*cos(rad_first)) +
      omega_vel*(-control_point*sin(rad_multi)*(cos(rad_first)+(d/s)*sin(rad_first)) + (control_point*cos(rad_first)*(sin(rad_first)-(d/s)*cos(rad_first)))))/r;
  //左の車輪計算
  omega_l_first = (x_vel*(cos(rad_first)-(d/s)*sin(rad_first)) +
    y_vel*(sin(rad_first)+(d/s)*cos(rad_first)) +
      omega_vel*(-control_point*sin(rad_multi)*(cos(rad_first)-(d/s)*sin(rad_first)) + (control_point*cos(rad_first)*(sin(rad_first)+(d/s)*cos(rad_first)))))/r;

  //Second機体の計算
  //右の車輪計算
  omega_r_second = (x_vel*(cos(rad_second)+(d/s)*sin(rad_second)) +
    y_vel*(sin(rad_second)-(d/s)*cos(rad_second)) +
      omega_vel*(-control_point*sin(rad_multi)*(cos(rad_second)+(d/s)*sin(rad_second)) + (control_point*cos(rad_second)*(sin(rad_second)-(d/s)*cos(rad_second)))))/r;
  //左の車輪計算
  omega_l_second = (x_vel*(cos(rad_second)-(d/s)*sin(rad_second)) +
    y_vel*(sin(rad_second)+(d/s)*cos(rad_second)) +
      omega_vel*(-control_point*sin(rad_multi)*(cos(rad_second)-(d/s)*sin(rad_second)) + (control_point*cos(rad_second)*(sin(rad_second)+(d/s)*cos(rad_second)))))/r;

  //車輪の計算からx方向とω方向の速度計算
  twist_first.linear.x  = (r*(omega_r_first + omega_l_first)) / 2;
  twist_first.angular.z = ((r*(omega_r_first - omega_l_first)) / (2*d));

  twist_second.linear.x  = (r*(omega_r_second + omega_l_second)) / 2;
  twist_second.angular.z = ((r*(omega_r_second - omega_l_second)) / (2*d));
}

//連結機体の角度を計算
void multi_icart_with_ps3::calc_linking_rad_multi(void){
  //連結機体の角度を計算
  rad_multi = asin((world_offset_position_y_first - world_offset_position_y_second) / distance_multi);
}

//連結機帯の制御点のworld座標における位置を計算
void multi_icart_with_ps3::calc_linking_position(void){
  //連結機帯の制御点のworld座標における位置を計算
  world_control_point_x = (world_offset_position_x_first + world_offset_position_x_second) / 2;
  world_control_point_y = (world_offset_position_y_first + world_offset_position_y_second) / 2;
}



//一秒ごとにdebugしたい変数を見るための関数
void multi_icart_with_ps3::debug(const ros::TimerEvent&){
  //first機体に関するデバック
  ROS_INFO("world_offset_position_x_first %f",world_offset_position_x_first);
  ROS_INFO("world_offset_position_y_first %f",world_offset_position_y_first);
  ROS_INFO("rad_first %f",rad_first*180/M_PI);

  //second機体に関するデバック
  ROS_INFO("world_offset_position_x_second %f",world_offset_position_x_second);
  ROS_INFO("world_offset_position_y_second %f",world_offset_position_y_second);
  ROS_INFO("rad_second %f",rad_second*180/M_PI);


  //連結機体に関するデバック
  ROS_INFO("world_control_point_x %f",world_control_point_x);
  ROS_INFO("world_control_point_y %f",world_control_point_y);
  ROS_INFO("rad_multi %f",rad_multi*180/M_PI);

  //目標位置に関するデバック
  double set_target_x,set_target_y,set_target_rad;
  ros::param::get("/multi_icart_with_ps3/target_position_x",set_target_x);
  ros::param::get("/multi_icart_with_ps3/target_position_y",set_target_y);
  ros::param::get("/multi_icart_with_ps3/target_position_rad",set_target_rad);

  ROS_INFO("target_position_x %f",set_target_x);
  ROS_INFO("target_position_y %f",set_target_y);
  ROS_INFO("target_rad %f",set_target_rad);




}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "multi_icart_with_ps3");
	multi_icart_with_ps3 multi_icart_with_ps3;
	ros::spin();
	return 0;
}
