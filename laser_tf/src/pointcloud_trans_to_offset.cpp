#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>



class pointcloud_trans_to_offset{
public:
  //
  pointcloud_trans_to_offset();

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
pointcloud_trans_to_offset::pointcloud_trans_to_offset(){
  //sub_define
  sub_point = nh.subscribe("/cloud", 5, &pointcloud_trans_to_offset::cb_offset_cloud,this);

  //pub_define
  pub_offset_point = nh.advertise<sensor_msgs::PointCloud>("/cloud_offset",10);
}

//callback
void pointcloud_trans_to_offset::cb_offset_cloud(const sensor_msgs::PointCloud &cloud){
  sensor_msgs::PointCloud offset_cloud;

  //trans_cloud_to_offset_cloud
  try{
    //wait_until_trans
    listener.waitForTransform("/base_link","/offset_base_link",ros::Time::now(),ros::Duration(5.0));
    //trans_to_offset_cloud
    listener.transformPointCloud("/offset_base_link",ros::Time(0),cloud,"base_link",offset_cloud);
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
    ros::init(argc, argv, "pointcloud_trans_to_offset");

    pointcloud_trans_to_offset pointcloud_trans_to_offset;

    ros::spin();

    return 0;
}
