#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class laser_to_pointcloud{
public:
  laser_to_pointcloud();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  //ノードハンドラ定義
  ros::NodeHandle node_;

  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;

  std::string default_base_link;
};

laser_to_pointcloud::laser_to_pointcloud():node_("~")
{
  scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &laser_to_pointcloud::scanCallback, this);
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);
  tfListener_.setExtrapolationLimit(ros::Duration(0.1));

  //変換座標パラメータ設定
  default_base_link="base_link";
  //node_.setParam("laser_base",default_base_link);
}

void laser_to_pointcloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud cloud;
    std::string laser_base;

    //動的に変換座標取得
    node_.getParam("laser_base",laser_base);
    //scanからpointcloudに変換
    projector_.transformLaserScanToPointCloud(laser_base, *scan, cloud, tfListener_);
    //pointcloudをpublish
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_pointcloud");

    laser_to_pointcloud laser_to_pointcloud;

    ros::spin();

    return 0;
}
