#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



class diijkstra{
public:
  diijkstra();
private:
  //コールバック定義
  void sub_map(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void sub_target_place(const geometry_msgs::PoseStamped::ConstPtr &target_place);

  //ダイクストラ法による経路計画
  void calc_path(void);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  ros::Subscriber sub_goal;
  ros::Subscriber sub_grid_map;
  ros::Publisher pub_path;

  nav_msgs::OccupancyGrid current_map;
  std_msgs::Bool receive_map_flag;
  geometry_msgs::PoseStamped goal;

};

//コンストラクタ定義
diijkstra::diijkstra(){
  //Pub,Sub定義
  sub_grid_map = nh.subscribe("/map", 5, &diijkstra::sub_map,this);
  sub_goal = nh.subscribe("/move_base_simple/goal", 5, &diijkstra::sub_target_place, this);

  pub_path = nh.advertise<nav_msgs::Path>("robot_path", 1000, true);

  receive_map_flag.data =false;
}

//mapを取得
void diijkstra::sub_map(const nav_msgs::OccupancyGrid::ConstPtr& map){
    current_map = *map;
    receive_map_flag.data = true;
}

//
void diijkstra::sub_target_place(const geometry_msgs::PoseStamped::ConstPtr &target_place){
    if(receive_map_flag.data == true){
        goal =*target_place;
        
    }
    calc_path();
}

void diijkstra::calc_path(){
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(3);
    path.poses[0].pose.position.x = 0.0;
    path.poses[0].pose.position.y = 0.0;
    path.poses[0].pose.position.z = 0.0;
    path.poses[0].pose.orientation.w = 1.0;

    path.poses[1].pose.position.x = 1.0;
    path.poses[1].pose.position.y = 1.0;
    path.poses[1].pose.position.z = 1.0;
    path.poses[1].pose.orientation.w = 1.0;

    path.poses[2].pose.position.x = 2.0;
    path.poses[2].pose.position.y = 2.0;
    path.poses[2].pose.position.z = 2.0;
    path.poses[2].pose.orientation.w = 1.0;

    pub_path.publish(path);
}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "diijkstra");
	diijkstra diijkstra;
	ros::spin();
	return 0;
}
