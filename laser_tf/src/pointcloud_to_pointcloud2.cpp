#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

//pcl_library
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/point_cloud_conversion.h>

class pointcloud_to_pointcloud2{
public:
  pointcloud_to_pointcloud2();

public:
  //ノードハンドラ定義
  ros::NodeHandle nh;

  //購読するトピックの定義
  ros::Subscriber sub_point;

  //配布するトピックの定義
  ros::Publisher pub_point2;

  //callback
  void cb_point2(const sensor_msgs::PointCloud &cloud);
};

//construct
pointcloud_to_pointcloud2::pointcloud_to_pointcloud2(){
  //sub_define
  sub_point = nh.subscribe("/cloud_offset", 5, &pointcloud_to_pointcloud2::cb_point2,this);

  //pub_define
  pub_point2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_offset",10);
}

//callback
void pointcloud_to_pointcloud2::cb_point2(const sensor_msgs::PointCloud &cloud){
  sensor_msgs::PointCloud2 point_cloud2;
  //trans
  sensor_msgs::convertPointCloudToPointCloud2(cloud, point_cloud2);

  pub_point2.publish(point_cloud2);
}

//main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_pointcloud2");

    pointcloud_to_pointcloud2 pointcloud_to_pointcloud2;

    ros::spin();

    return 0;
}
