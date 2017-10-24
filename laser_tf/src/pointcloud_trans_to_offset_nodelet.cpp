#include "pointcloud_trans_to_offset_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace pointcloud_trans_to_offset_ns{

  pointcloud_trans_to_offset_nodelet::pointcloud_trans_to_offset_nodelet(){}

  //callback
  void pointcloud_trans_to_offset_nodelet::cb_offset_cloud(const sensor_msgs::PointCloud::ConstPtr &cloud){
    sensor_msgs::PointCloud offset_cloud;

    //trans_cloud_to_offset_cloud
    try{
      //wait_until_trans
      listener.waitForTransform("/base_link","/offset_base_link",ros::Time::now(),ros::Duration(5.0));
      //trans_to_offset_cloud
      listener.transformPointCloud("/offset_base_link",ros::Time(0),*cloud,"base_link",offset_cloud);
    }catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //publish
    pub_offset_point.publish(offset_cloud);
  }

  void pointcloud_trans_to_offset_nodelet::onInit(){
    //sub_define
    sub_point = nh.subscribe("/cloud", 5, &pointcloud_trans_to_offset_nodelet::cb_offset_cloud,this);
    //pub_define
    pub_offset_point = nh.advertise<sensor_msgs::PointCloud>("/cloud_offset",10);
  }
}

PLUGINLIB_EXPORT_CLASS(pointcloud_trans_to_offset_ns::pointcloud_trans_to_offset_nodelet, nodelet::Nodelet)
