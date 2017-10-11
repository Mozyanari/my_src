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
};

laser_to_pointcloud::laser_to_pointcloud(){
  scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &laser_to_pointcloud::scanCallback, this);
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);
  tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void laser_to_pointcloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_pointcloud");

    laser_to_pointcloud laser_to_pointcloud;

    ros::spin();

    return 0;
}
