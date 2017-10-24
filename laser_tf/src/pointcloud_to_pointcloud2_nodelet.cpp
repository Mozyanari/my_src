#include "pointcloud_to_pointcloud2_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace pointcloud_to_pointcloud2_ns{

  pointcloud_to_pointcloud2_nodelet::pointcloud_to_pointcloud2_nodelet(){}

  void pointcloud_to_pointcloud2_nodelet::cb_point2(const sensor_msgs::PointCloud::ConstPtr &cloud){
    sensor_msgs::PointCloud2 point_cloud2;
    //trans
    sensor_msgs::convertPointCloudToPointCloud2(*cloud, point_cloud2);

    pub_point2.publish(point_cloud2);
  }

  void pointcloud_to_pointcloud2_nodelet::onInit(){
    //sub_define
    sub_point = nh.subscribe("/cloud_offset", 5, &pointcloud_to_pointcloud2_nodelet::cb_point2,this);
    //pub_define
    pub_point2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2_offset",10);
  }
}

PLUGINLIB_EXPORT_CLASS(pointcloud_to_pointcloud2_ns::pointcloud_to_pointcloud2_nodelet, nodelet::Nodelet)
