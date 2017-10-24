#ifndef POINTCLOUD_TO_POINTCLOUD2_H_
#define POINTCLOUD_TO_POINTCLOUD2_H_

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

#include <nodelet/nodelet.h>

namespace pointcloud_to_pointcloud2_ns{
  class pointcloud_to_pointcloud2_nodelet : public nodelet::Nodelet
  {
  public:
    pointcloud_to_pointcloud2_nodelet();
    virtual void onInit();

  private:
    //ノードハンドラ定義
    ros::NodeHandle nh;

    //購読するトピックの定義
    ros::Subscriber sub_point;

    //配布するトピックの定義
    ros::Publisher pub_point2;

    //callback
    void cb_point2(const sensor_msgs::PointCloud::ConstPtr &cloud);
  };
}

#endif
