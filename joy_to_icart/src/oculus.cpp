#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

# define M_PI       3.14159265358979323846  /* pi */



//クラス定義(どのような構造か定義)-----------------------------------------------------
class oculus
{
  //
  public:
	oculus();
  private:

  //コールバック関数定義
  void cb_oculus(const geometry_msgs::PoseStamped::ConstPtr &data);
  void cb_contoroller(const geometry_msgs::PoseStamped::ConstPtr &data);
  void cb_triger(const sensor_msgs::Joy::ConstPtr &data);

  //今のコントローラの角度
  double roll, pitch, yaw;

  //基準となるコントローラの角度
  double standard_roll, standard_pitch, standard_yaw;


  //ノードハンドラ作成
  ros::NodeHandle nh;

  //使用変数定義
  //使用するTopicの定義
  ros::Subscriber sub_oculus;
  ros::Subscriber sub_controller;
  ros::Subscriber sub_triger;

  ros::Publisher  pub_head_theta;
  ros::Publisher  pub_speed;

};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
oculus::oculus(){
    sub_oculus = nh.subscribe("/camera", 5, &oculus::cb_oculus,this);
    sub_controller = nh.subscribe("/contoroller", 5, &oculus::cb_contoroller,this);
    sub_triger = nh.subscribe("/triger", 5, &oculus::cb_triger,this);

    pub_head_theta = nh.advertise<std_msgs::Float64>("head_pose", 1000);
    pub_speed = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1000);
}
//関数定義-----------------------------------------------------------------------
//Oculus go本体の角度を取得
void oculus::cb_oculus(const geometry_msgs::PoseStamped::ConstPtr &data){
    geometry_msgs::PoseStamped a = *data;
    double theta = tf::getYaw(a.pose.orientation);
    ROS_INFO("theta=%f",theta);
    std_msgs::Float64 send;
    send.data = theta;
    pub_head_theta.publish(send);
}
//Oculus goのコントローラの角度を取得
void oculus::cb_contoroller(const geometry_msgs::PoseStamped::ConstPtr &data){
  static double old_roll;
  static double old_pitch;
  static double old_yaw;

  tf::Quaternion quat(data->pose.orientation.x, data->pose.orientation.y ,data->pose.orientation.z, data->pose.orientation.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  roll = roll * (180.0 / M_PI);
  pitch = pitch * (180.0 / M_PI);
  yaw = yaw * (180.0 / M_PI);
  ROS_INFO("roll=%f, pitch=%f, yaw=%f",roll,pitch,yaw);

  if((abs(old_roll-roll)>abs(old_pitch-pitch)) && (abs(old_roll-roll)>abs(old_yaw-yaw))){
    ROS_INFO("roll_rotation");
  }else if((abs(old_pitch-pitch)>abs(old_roll-roll)) && (abs(old_pitch-pitch)>abs(old_yaw-yaw))){
    ROS_INFO("pitch_rotation");
  }else if((abs(old_yaw-yaw)>abs(old_roll-roll)) && (abs(old_yaw-yaw)>abs(old_pitch-pitch))){
    ROS_INFO("yaw_rotation");
  }

  old_roll = roll;
  old_pitch = pitch;
  old_yaw = yaw;


  ROS_INFO("roll=%f, pitch=%f, yaw=%f",roll,pitch,yaw);
}

//コントローラのtrigerの状態を取得
void oculus::cb_triger(const sensor_msgs::Joy::ConstPtr &data){
  static int state = 0;
  static int old_state = 0;

  geometry_msgs::Twist robo_speed;

  //今のコントローラの状態を入力
  state = data->buttons[0];

  //今trigerが押されたか判定
  if((state == 1)&&(old_state == 0)){
    //今押された
    standard_roll = roll;
    standard_pitch = pitch;
    standard_yaw = yaw;
  }else if((state ==1)&&(old_state == 1)){
    //押されて続けてるからそれに合わせて速度
    double diff_roll = roll - standard_roll;
    double diff_pitch = pitch - standard_pitch;

    //速度を代入
    robo_speed.linear.x = -0.05*(diff_roll);
    robo_speed.angular.z = 0.05*(diff_pitch);

    //0.5以下なら0とみなす
    if(abs(robo_speed.linear.x)<0.5){
      robo_speed.linear.x=0;
    }
    if(abs(robo_speed.angular.z)<0.5){
      robo_speed.angular.z=0;
    }

    //速度を送信
    pub_speed.publish(robo_speed);
  }else{
    //押されてないから停止
    robo_speed.linear.x = 0;
    robo_speed.angular.z = 0;

    //速度を送信
    pub_speed.publish(robo_speed);
  }

  //今の状態を保存
  old_state = state;
}


//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "oculus");
	oculus oculus;
	ros::spin();
	return 0;
}
