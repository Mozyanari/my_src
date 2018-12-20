#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>



class particle_offset{
public:
  particle_offset();
private:
  //コールバック定義
  void calc_particle_offset(const geometry_msgs::PoseArray::ConstPtr& data);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  ros::Publisher pub_particle_offset;
  ros::Publisher pub_pointcloud_offset;
  ros::Subscriber sub_particle;

  //オフセット距離[m]
  double s;
};

//コンストラクタ定義
particle_offset::particle_offset(){
  //Pub,Sub定義
  sub_particle = nh.subscribe("particlecloud", 5, &particle_offset::calc_particle_offset,this);
  pub_particle_offset = nh.advertise<geometry_msgs::PoseArray>("particlecloud_offset", 1000, true);
  pub_pointcloud_offset = nh.advertise<sensor_msgs::PointCloud>("pointcloud_offset", 1000, true);
  //オドメトリ位置からオフセットまでの距離[m]
  s = 0.16;
}

//オフセット位置計算
void particle_offset::calc_particle_offset(const geometry_msgs::PoseArray::ConstPtr& data){
  //オフセット位置
  geometry_msgs::PoseArray particle_offset;
  sensor_msgs::PointCloud pointcloud_offset;

  //パーティクルの数
  int number = data->poses.size();
  //パーティクルの数だけリサイズする
  particle_offset.poses.resize(number);
  pointcloud_offset.points.resize(number);

  //位置以外の情報はそのまま保持
  particle_offset.header = data->header;
  pointcloud_offset.header = data->header;

  for(int i = 0;i<number ;i++){
    double x = data->poses[i].position.x - (s * cos(tf::getYaw(data->poses[i].orientation)));
    double y = data->poses[i].position.y - (s * sin(tf::getYaw(data->poses[i].orientation)));
    //PoseArray
    //位置の代入
    particle_offset.poses[i].position.x = x;
    particle_offset.poses[i].position.y = y;

    //Pointcloud
    //位置の代入
    pointcloud_offset.points[i].x = x;
    pointcloud_offset.points[i].y = y;


    //クオータニオンの代入
    particle_offset.poses[i].orientation = data->poses[i].orientation;
  }
  
  //amcl_poseからparticle_offsetに変換
  /*
  particle_offset.theta = tf::getYaw(data->pose.pose.orientation);
  particle_offset.x = data->pose.pose.position.x - (s * cos(particle_offset.theta));
  particle_offset.y = data->pose.pose.position.y - (s * sin(particle_offset.theta));
  */
  //particle_offset.pose.pose.position.z = tf::getYaw(data.pose.pose.orientation);
  //particle_offset.data=sqrt( (pow(transform.getOrigin().x(),2)) + (pow(transform.getOrigin().y(),2)) );
  //ROS_INFO("particle_offset.data %f",particle_offset);

  pub_particle_offset.publish(particle_offset);
  pub_pointcloud_offset.publish(pointcloud_offset);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "particle_offset");
	particle_offset particle_offset;
	ros::spin();
	return 0;
}
