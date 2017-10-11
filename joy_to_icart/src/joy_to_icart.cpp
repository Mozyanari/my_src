#define _USE_MATH_DEFINES
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


//クラス定義(どのような構造か定義)-----------------------------------------------------
class joy_to_icart
{
  //
  public:
	joy_to_icart();

  private:

  //コールバック関数定義
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void cb_odom(const nav_msgs::Odometry::ConstPtr &msg);




  //ノードハンドラ作成
	ros::NodeHandle nh;

  //使用変数定義
	int vel_linear, vel_angular;
	double l_scale_, a_scale_;

  //使用するTopicの定義
  ros::Subscriber sub_odom;
	ros::Publisher  vel_pub_;
	ros::Subscriber joy_sub_;

  //機体の状態データ
  nav_msgs::Odometry odom;
  //機体に送信するデータ
  geometry_msgs::Twist twist;

  //機体パラメータ
  //目標速度
  double x_vel;
  double y_vel;
  //タイヤの半径[mm]
  double r;
  //トレッド距離[mm]
  double d;
  //機体の姿勢角度[rad]
  double rad;
  //左右の車輪の速度
  double omega_l, omega_r;
  //オフセット距離
  double s;
  //angularの倍率
  double rate;
  //機体のロボット座標における位置
  double position_x;
  double position_y;

  //機体のオフセット位置の座標
  double offset_x;
  double offset_y;

  double diff_x;
  double diff_y;

  //目標位置
  double target_x;
  double target_y;



};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
joy_to_icart::joy_to_icart(): vel_linear(1), vel_angular(2), a_scale_(4.0),l_scale_(2.0)
{
  //変数初期化
  //タイヤの半径[m]32.5mm
  r = 0.0325;
  //トレッド距離[m]160mm???????????????
  d = 0.178;
  //オフセット距離[m]160mm
  s = 0.04;
  //機体の角度
  rad = 0.0;
  //機体の車輪の速度
  omega_l = 0.0;
  omega_r = 0.0;

  //目標位置
  target_x = 1.0;
  target_y = 0.0;


	nh.param("axis_linear"  , vel_linear , vel_linear);
	nh.param("axis_angular" , vel_angular, vel_angular);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear" , l_scale_, l_scale_);

  //odomのクオータニオンを初期化
  odom.pose.pose.orientation.w = 1.0;

  //購読するトピックの定義
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joy_to_icart::joyCallback, this);
  sub_odom = nh.subscribe("/ypspur_ros/odom", 5, &joy_to_icart::cb_odom,this);

  //配布するトピックの定義
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);
}
//関数定義-----------------------------------------------------------------------
//cb_odom関数定義
void joy_to_icart::cb_odom(const nav_msgs::Odometry::ConstPtr &msg){
  //機体の状態受信
  odom = *msg;

  //機体に送る速度の計算
  //機体の角度を代入
  rad = tf::getYaw(odom.pose.pose.orientation);

  //second機体の位置を代入
  position_x = odom.pose.pose.position.x;
  position_y = odom.pose.pose.position.y;

  offset_x = (position_x + s) - (s * cos(rad));
  offset_y = position_y - s * sin(rad);

  //位置から速度を算出
  //目標までの差
  diff_x = (target_x-offset_x);
  diff_y = (target_y-offset_y);

  /*
  //位置のズレが0.01[m]以上なら速度を出す
  if(diff_x > 0.01){
    x_vel = 0.1;
  }else if(diff_x < -0.01){
    x_vel = -0.1;
  }else{
    x_vel = 0;
  }

  if(diff_y > 0.01){
    y_vel = 0.1;
  }else if(diff_y < -0.01){
    y_vel = -0.1;
  }else{
    y_vel = 0;
  }
  */

  //左右の車輪の速度を計算
  //右の車輪計算
  omega_r = (((cos(rad)+((d/s)*sin(rad)))*x_vel) + ((sin(rad)-(d/s)*cos(rad))*y_vel))/r;
  //左の車輪計算
  omega_l = (((cos(rad)-((d/s)*sin(rad)))*x_vel) + ((sin(rad)+(d/s)*cos(rad))*y_vel))/r;


  //車輪の計算からx方向とω方向の速度計算
  twist.linear.x  = (r*(omega_r + omega_l)) / 2;
  twist.angular.z = ((r*(omega_r - omega_l)) / (2*d));
  //twist.linear.x = 0;
  //twist.angular.z = 0.5;
  //計算したTopicを送る
  vel_pub_.publish(twist);

  ROS_INFO("rad %f",rad*180/M_PI);

  //ROS_INFO("position_x %f",position_x);
  //ROS_INFO("position_y %f",position_y);

  ROS_INFO("offset_x %f",offset_x);
  ROS_INFO("offset_y %f",offset_y);





  //デバック
  //ROS_INFO("odom_come_on!!");


}

//Joycallback関数定義
void joy_to_icart::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  //ローカル変数定義

  //x,y方向の速度の更新
  x_vel = joy->axes[vel_linear]*0.1;
  y_vel = joy->axes[vel_angular]*0.1;

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
  //ROS_INFO("rad %f",rad*2*M_PI);

  //ROS_INFO("x_vel %f",x_vel);
  //ROS_INFO("y_vel %f",y_vel);

  //ROS_INFO("path1 %f",path1);
  //ROS_INFO("path2 %f",path2);


  //ROS_INFO("omega_l %f",omega_l);
  //ROS_INFO("omega_r %f",omega_r);

  //ROS_INFO("position_x %f",position_x);
  //ROS_INFO("position_y %f",position_y);

  //ROS_INFO("vel %f",twist.linear.x);
  //ROS_INFO("angular %f",twist.angular.z);

  //ROS_INFO_STREAM("(" << joy->axes[vel_linear] << " " << joy->axes[vel_angular] << ")");


}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_to_icart");
	joy_to_icart joy_to_icart;
	ros::spin();
	return 0;
}
