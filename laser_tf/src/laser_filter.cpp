#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

class laser_filter{
public:
  laser_filter();

private:
  //callback_define
  void cb_catch_scan_first(const sensor_msgs::LaserScan::ConstPtr& scan);
  void cb_catch_scan_second(const sensor_msgs::LaserScan::ConstPtr& scan);
  void cb_catch_scan_control_point(const sensor_msgs::LaserScan::ConstPtr& scan);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  ros::Subscriber sub_scan_first;
  ros::Subscriber sub_scan_second;
  ros::Subscriber sub_scan_control_point;

  ros::Publisher pub_scan_filter;

  //
  sensor_msgs::LaserScan scan_first_buff;
  sensor_msgs::LaserScan scan_second_buff;
  sensor_msgs::LaserScan scan_control_point_buff;
};

//construct_define
laser_filter::laser_filter(){
  //sub_define
  sub_scan_first=nh.subscribe<sensor_msgs::LaserScan> ("/scan_first", 100, &laser_filter::cb_catch_scan_first, this);
  sub_scan_second=nh.subscribe<sensor_msgs::LaserScan> ("/scan_second", 100, &laser_filter::cb_catch_scan_second, this);
  sub_scan_control_point=nh.subscribe<sensor_msgs::LaserScan> ("/scan_control_point", 100, &laser_filter::cb_catch_scan_control_point, this);

  //pub_define
  pub_scan_filter=nh.advertise<sensor_msgs::LaserScan> ("/scan_filter", 100, false);

}

//callback
void laser_filter::cb_catch_scan_first(const sensor_msgs::LaserScan::ConstPtr& scan){
  scan_first_buff=*scan;

}

void laser_filter::cb_catch_scan_second(const sensor_msgs::LaserScan::ConstPtr& scan){
  scan_second_buff=*scan;
  ROS_INFO("%f",scan_second_buff.time_increment);
}

void laser_filter::cb_catch_scan_control_point(const sensor_msgs::LaserScan::ConstPtr& scan){
  sensor_msgs::LaserScan scan_filter;

  scan_filter=*scan;

  scan_filter.header.stamp=scan_first_buff.header.stamp;
  scan_filter.time_increment=scan_first_buff.time_increment;

  pub_scan_filter.publish(scan_filter);



}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_filter");

    laser_filter laser_filter;

    ros::spin();

    return 0;
}
