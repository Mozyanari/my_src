#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PoseWithCovarianceStamped_to_PoseStamped{
public:
  PoseWithCovarianceStamped_to_PoseStamped();
private:
  //コールバック定義
  void cb_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  ros::Publisher pub_PoseStamped;
  ros::Subscriber sub_PoseWithCovarianceStamped;
};

//コンストラクタ定義
PoseWithCovarianceStamped_to_PoseStamped::PoseWithCovarianceStamped_to_PoseStamped(){
  //Pub,Sub定義
  sub_PoseWithCovarianceStamped = nh.subscribe("PoseWithCovarianceStamped", 5, &PoseWithCovarianceStamped_to_PoseStamped::cb_PoseWithCovarianceStamped,this);
  pub_PoseStamped = nh.advertise<geometry_msgs::PoseStamped>("PoseStamped", 1000, true);
}

void PoseWithCovarianceStamped_to_PoseStamped::cb_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data){
    geometry_msgs::PoseStamped send_data;
    send_data.header = data->header;
    send_data.pose = data->pose.pose;
    pub_PoseStamped.publish(send_data);
}


//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "PoseWithCovarianceStamped_to_PoseStamped");
	PoseWithCovarianceStamped_to_PoseStamped PoseWithCovarianceStamped_to_PoseStamped;
	ros::spin();
	return 0;
}
