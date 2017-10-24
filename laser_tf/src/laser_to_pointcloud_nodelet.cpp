#include "laser_to_pointcloud_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace laser_to_pointcloud_ns{

  laser_to_pointcloud_nodelet::laser_to_pointcloud_nodelet(){}

  void laser_to_pointcloud_nodelet::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
      sensor_msgs::PointCloud cloud;
      projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
      point_cloud_publisher_.publish(cloud);
  }

  void laser_to_pointcloud_nodelet::onInit(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &laser_to_pointcloud_nodelet::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
  }
}

PLUGINLIB_EXPORT_CLASS(laser_to_pointcloud_ns::laser_to_pointcloud_nodelet, nodelet::Nodelet)
