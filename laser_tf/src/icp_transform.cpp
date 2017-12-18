#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


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

    ros::Publisher pub_pcl_first;

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
  pub_pcl_first = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/pcl_data",10);

  sub_cloud2_first=nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_control_point_first", 100, &icp_transform::cb_catch_cloud2_first, this);
  sub_cloud2_second=nh.subscribe<sensor_msgs::PointCloud2> ("/cloud2_control_point_second", 100, &icp_transform::cb_catch_cloud2_second, this);

  timer = nh.createTimer(ros::Duration(1.0), &icp_transform::transform,this);
}

//pointcloud取得関数
void icp_transform::cb_catch_cloud2_first(const sensor_msgs::PointCloud2::ConstPtr& cloud2){
  cloud2_first=*cloud2;
  pcl::fromROSMsg(cloud2_first,pcl_first);

  //ROS_INFO("first");
  //pub_pcl_first.publish(pcl_first);
}

void icp_transform::cb_catch_cloud2_second(const sensor_msgs::PointCloud2::ConstPtr& cloud2){
  cloud2_second=*cloud2;
  pcl::fromROSMsg(cloud2_second,pcl_second);
}

//表示関数
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


void icp_transform::transform(const ros::TimerEvent&){
  //cloud_firstとcloud_secondをICPアルゴズムにより変換matrixを求める。
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  //pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in (&pcl_first);

  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_in (&pcl_first);
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_out (&pcl_second);

  //icp.setInputSource(cloud_in);
  //icp.setInputTarget(cloud_out);
  icp.setInputSource(pcl_first.makeShared());
  icp.setInputTarget(pcl_second.makeShared());

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  //変換matrixを表示する
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  transformation_matrix = icp.getFinalTransformation ().cast<double>();
  print4x4Matrix (transformation_matrix);
}


//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_transform");

    icp_transform icp_transform;

    ros::spin();

    return 0;
}
