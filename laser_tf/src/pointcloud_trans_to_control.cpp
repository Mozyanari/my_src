#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>



class pointcloud_trans_to_control{
public:
  pointcloud_trans_to_control();

  //Listener定義
  tf::TransformListener listener;
private:
  //ノードハンドラ定義
  ros::NodeHandle nh;

  //購読するトピックの定義
  ros::Subscriber sub_point;

  //配布するトピックの定義
  ros::Publisher pub_offset_point;

  //callback
  void cb_offset_cloud(const sensor_msgs::PointCloud &cloud);
};

//construct
pointcloud_trans_to_control::pointcloud_trans_to_control(): nh("~")
{
  //sub_define
  sub_point = nh.subscribe("/cloud", 5, &pointcloud_trans_to_control::cb_offset_cloud,this);
  //pub_define
  pub_offset_point = nh.advertise<sensor_msgs::PointCloud>("/cloud_offset",10);

  //変換座標パラメータ設定
  //nh.setParam("cloud_trans_mother","base_link");
  //nh.setParam("cloud_trans_chile","offset_base_link");
}

//callback
void pointcloud_trans_to_control::cb_offset_cloud(const sensor_msgs::PointCloud &cloud){
  sensor_msgs::PointCloud offset_cloud;
  std::string cloud_trans_mother;
  std::string cloud_trans_chile;

  //動的に変換座標取得
  nh.getParam("cloud_trans_mother",cloud_trans_mother);
  nh.getParam("cloud_trans_chile",cloud_trans_chile);


  //trans_cloud_to_offset_cloud
  try{
    //wait_until_trans
    listener.waitForTransform(cloud_trans_mother,cloud_trans_chile,ros::Time::now(),ros::Duration(5.0));
    //trans_to_offset_cloud
    listener.transformPointCloud(cloud_trans_chile,ros::Time(0),cloud,cloud_trans_mother,offset_cloud);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  //publish
  pub_offset_point.publish(offset_cloud);
}

//main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_trans_to_control");

    pointcloud_trans_to_control pointcloud_trans_to_control;

    ros::spin();

    return 0;
}
