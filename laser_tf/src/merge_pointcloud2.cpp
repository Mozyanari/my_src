#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_conversions/pcl_conversions.h>

class merge_pointcloud2{
  public:
    merge_pointcloud2();

    //cloud2のデータ群
    sensor_msgs::PointCloud2 cloud2_first;
    sensor_msgs::PointCloud2 cloud2_second;
    sensor_msgs::PointCloud2 cloud2_merge;

  private:
    //pointcloud2を取得
    void cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2);
    void cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2);

    void merge(void);

    //ノードハンドラ作成
    ros::NodeHandle nh;

    ros::Subscriber sub_cloud2_first;
    ros::Subscriber sub_cloud2_second;

    ros::Publisher  pub_cloud_merge;



};

//コンストラクタ初期化
merge_pointcloud2::merge_pointcloud2(){
  sub_cloud2_first=nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_first", 100, &merge_pointcloud2::cb_catch_cloud2_first, this);
  sub_cloud2_second=nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_second", 100, &merge_pointcloud2::cb_catch_cloud2_second, this);

  pub_cloud_merge=nh.advertise<sensor_msgs::PointCloud2> ("/cloud2_merge", 100, false);
}

//pointcloud取得関数
void merge_pointcloud2::cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2){
  cloud2_first=*cloud2;
  merge();
}

void merge_pointcloud2::cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2){
  cloud2_second=*cloud2;
  merge();
}

void merge_pointcloud2::merge(void){
  pcl::concatenatePointCloud(cloud2_first, cloud2_second, cloud2_merge);
  pub_cloud_merge.publish(cloud2_merge);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_pointcloud2");

    merge_pointcloud2 merge_pointcloud2;

    ros::spin();

    return 0;
}
