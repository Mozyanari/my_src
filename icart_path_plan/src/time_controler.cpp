#define TIME 0.0
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>

class time_controler{
public:
  time_controler();
private:
  //コールバック定義
  //目標時間に近いかどうかをチェック
  void time_checker(const ros::TimerEvent&);

  //目標時間を取得
  void cb_time_get(const std_msgs::Header::ConstPtr &data);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  //購読するトピックの定義
  ros::Subscriber sub_time;

  //配布するトピックの定義
  ros::Publisher pub_time_controler;
  ros::Publisher pub_target_point;

  //時間の関数作成
  ros::Timer timer;

  //受信した時間と番号
  std_msgs::Header receive_time;

};

time_controler::time_controler(){
    //購読するトピックの定義
    sub_time=nh.subscribe("/time_check", 5, &time_controler::cb_time_get,this);

    //配布するトピックの定義
    pub_time_controler=nh.advertise<std_msgs::Int32>("/time_result", 1000);
    pub_target_point=nh.advertise<geometry_msgs::Pose2D>("/target_point",1000);

    //時間で起動する関数の定義
    timer = nh.createTimer(ros::Duration(0.001), &time_controler::time_checker,this);

    //receive_timeの初期化
    receive_time.seq = -1;

}
//時間と番号を取得
void time_controler::cb_time_get(const std_msgs::Header::ConstPtr &data){
    receive_time = *data;
    ROS_INFO("cb_time_get");

    ROS_INFO("seq_%d",(int)receive_time.seq);
    static int old_seq = 0;
    static int point = 6;
    if(((int)receive_time.seq < 0) && (old_seq != (int)receive_time.seq)){
      if(point == 1){
        geometry_msgs::Pose2D pose;
        pose.x = 2.0;
        pose.y = 0.0;
        pose.theta = 1.57;
        pub_target_point.publish(pose);
        point++;
      }else if(point == 2){
        geometry_msgs::Pose2D pose;
        pose.x = 2.0;
        pose.y = 2.0;
        pose.theta = 3.14;
        pub_target_point.publish(pose);
        point++;
      }else if(point == 3){
        geometry_msgs::Pose2D pose;
        pose.x = 0.0;
        pose.y = 2.0;
        pose.theta = 1.57;
        pub_target_point.publish(pose);
        point++;
      }else if(point == 4){
        geometry_msgs::Pose2D pose;
        pose.x = 0.0;
        pose.y = 0.0;
        pose.theta = 3.14;
        pub_target_point.publish(pose);
        point++;
      }else if(point == 6){
        geometry_msgs::Pose2D pose;
        pose.x = 5.0;
        pose.y = 0.0;
        pose.theta = 1.57;
        pub_target_point.publish(pose);
        point++;
      }else if(point == 7){
        geometry_msgs::Pose2D pose;
        pose.x = 10.0;
        pose.y = 0.0;
        pose.theta = 0.0;
        pub_target_point.publish(pose);
        point++;
      }else{
        point = -1;
      }
    }
    old_seq = (int)receive_time.seq;
}

//目標時間に近いか判定を行う
void time_controler::time_checker(const ros::TimerEvent&){
  std_msgs::Int32 time_controler;
  //seqがマイナスだと判定しない
  //マイナスだと次の目標地点を出す
  //ROS_INFO("seq_%d",(int)receive_time.seq);
  if((int)receive_time.seq < 0){
    //ROS_INFO("not judge");
    return;
  }

  //到着時間に近いかどうかの判定
  double diff_time = (receive_time.stamp - ros::Time::now()).toSec();
  //0.0s以内なら次のサブゴールを送信する信号を出す
  if(diff_time < TIME){
    time_controler.data = receive_time.seq;
    ROS_INFO("receive_time.seq=%d",receive_time.seq);
    pub_time_controler.publish(time_controler);
  }

  //ROS_INFO("time_controler.data %f",time_controler);


}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "time_controler");
	time_controler time_controler;
	ros::spin();
	return 0;
}
