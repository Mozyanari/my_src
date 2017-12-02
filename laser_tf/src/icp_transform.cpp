#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>


class icp_transform{
  public:
    icp_transform();

  private:
    //pointcloud2を取得
    void cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2);
    void cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2);

    void transform(const ros::TimerEvent&);

    //ノードハンドラ作成
    ros::NodeHandle nh;

    ros::Subscriber sub_cloud2_first;
    ros::Subscriber sub_cloud2_second;

    //cloud2のデータ群
    sensor_msgs::PointCloud2 cloud2_first;
    sensor_msgs::PointCloud2 cloud2_second;

    //pcl_point_cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_first;
    pcl::PointCloud<pcl::PointXYZ> pcl_second;
    //pcl::PointCloud<pcl::PointXYZ> pcl_second


    //時間の関数作成
    ros::Timer timer;

};
//コンストラクタ初期化
icp_transform::icp_transform(){
  sub_cloud2_first=nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_first", 100, &icp_transform::cb_catch_cloud2_first, this);
  sub_cloud2_second=nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_second", 100, &icp_transform::cb_catch_cloud2_second, this);
  timer = nh.createTimer(ros::Duration(1.0), &icp_transform::transform,this);
}

//pointcloud取得関数
void icp_transform::cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2){
  cloud2_first=*cloud2;
  pcl::fromROSMsg(cloud2_first,pcl_first);
}

void icp_transform::cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2){
  cloud2_second=*cloud2;
  pcl::fromROSMsg(cloud2_second,pcl_second);
}

void icp_transform::transform(const ros::TimerEvent&){
  //cloud_firstとcloud_secondをICPアルゴズムにより変換matrixを求める。
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  //boost::function<(pcl::PointCloud<pcl::PointXYZ>::Ptr)> cloud_in = pcl_first;

  //icp.setInputCloud(cloud_in);
  //icp.setInputTarget(*pcl_second);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);


}


//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_transform");

    icp_transform icp_transform;

    ros::spin();

    return 0;
}
