#ifndef POINTCLOUD_TRANS_TO_OFFSET_NODELET_H_
#define POINTCLOUD_TRANS_TO_OFFSET_NODELET_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <nodelet/nodelet.h>

namespace pointcloud_trans_to_offset_ns{
  class pointcloud_trans_to_offset_nodelet : public nodelet::Nodelet
  {
  public:
    pointcloud_trans_to_offset_nodelet();
    //Listener定義
    tf::TransformListener listener;

    virtual void onInit();

  private:
    //ノードハンドラ定義
    ros::NodeHandle nh;

    //購読するトピックの定義
    ros::Subscriber sub_point;

    //配布するトピックの定義
    ros::Publisher pub_offset_point;

    //callback
    void cb_offset_cloud(const sensor_msgs::PointCloud::ConstPtr &cloud);
  };

}

#endif
