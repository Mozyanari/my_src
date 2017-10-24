#ifndef LASER_TO_POINTCLOUD_NODELET_H_
#define LASER_TO_POINTCLOUD_NODELET_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <nodelet/nodelet.h>

namespace laser_to_pointcloud_ns{
  class laser_to_pointcloud_nodelet : public nodelet::Nodelet
  {
  public:
    laser_to_pointcloud_nodelet();
    virtual void onInit();

  private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    //ノードハンドラ定義
    ros::NodeHandle node_;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Subscriber scan_sub_;
  };
}


#endif
