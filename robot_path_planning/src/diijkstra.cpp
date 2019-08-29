#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

#include <geometry_msgs/Pose2D.h>

#include "eigen3/Eigen/Core"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

struct CostList{
    int cost;
    int parent;
};

class diijkstra{
public:
  diijkstra();
private:
  //コールバック定義
  void sub_map(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void sub_target_place(const geometry_msgs::PoseStamped::ConstPtr &target_place);

  //ダイクストラ法による経路計画
  //返り値が-1なら失敗，1なら成功
  int calc_path(void);

  void free_node_list_check(int parent,int now);
  void open_node_list_check(int parent,int now);
  void close_node_list_check(int parent,int now);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  ros::Subscriber sub_goal;
  ros::Subscriber sub_grid_map;
  ros::Publisher pub_path;

  nav_msgs::OccupancyGrid current_map;
  nav_msgs::OccupancyGrid cost_map;
  std_msgs::Bool receive_map_flag;
  geometry_msgs::PoseStamped goal;

  char state_list[];
  CostList cost_list[];

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
    cost_map = current_map;
    receive_map_flag.data = true;
    ROS_INFO("receive map");
}

//
void diijkstra::sub_target_place(const geometry_msgs::PoseStamped::ConstPtr &target_place){
    if(receive_map_flag.data == true){
        goal =*target_place;
        calc_path();
    }
    
}

int diijkstra::calc_path(){
    //マップのセル数の取得
    int cell_width = current_map.info.width;
    int cell_height = current_map.info.height;
    int cell_length = cell_width*cell_height;
    ROS_INFO("width=%d height=%d",cell_width,cell_height);

    int start_x = 0;
    int start_y = 0;

    int goal_x = 10;
    int goal_y = 10;

    //オープンリスト，クローズリスト，フリーリストの初期化
    //値が1ならオープン，-1ならクローズ，0ならフリー，2なら次にオープン，-100なら通れない場所
    //0で初期化
    state_list[cell_length];
    for(int i = 0; i< cell_length;i++){
        //mapのデータは100なら障害物，0なら床，-1なら未定となる
        if(cost_map.data[i] == 100){
            state_list[i] = -100;
        }else{
            state_list[i] = 0;
        }
    }

    //Debug
    for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",state_list[i*(cell_width)+j]);
        }
        printf("\n");
    }

    //コストリストの初期化
    cost_list[cell_length];
    for(int i = 0; i< cell_length;i++){
        cost_list[i].cost = 0;
        cost_list[i].parent = 0;
    }

    //スタートノードをオープンリストに入れる
    state_list[(cell_width)*start_y+start_x] = 1;

    ROS_INFO("start_calclate");
    //コスト計算をする
    while(1){
        for(int i = 0; i< cell_length;i++){
            //オープンリストに入ってるノードを発見したら、クローズリストにして、隣接したノードを計算する
            if(state_list[i] == 1){
                //クローズリストにする
                state_list[i] = -1;
                //周囲のノードのチェック
                int check_number;
                //周囲のノードの番号が範囲外かどうかチェックして，登録されているリストによって計算
                //左下
                check_number = i-cell_width-1;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //下
                check_number = i-cell_width;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //右下
                check_number = i-cell_width+1;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //左
                check_number = i-1;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //右
                check_number = i+1;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //左上
                check_number = i+cell_width-1;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //上
                check_number = i+cell_width;
                if( !(check_number < 0) || !(cell_length < check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //右上
                check_number = i+cell_width+1;
                if(state_list[i-1] == 0)free_node_list_check(i,i-1);
                if(state_list[i+1] == 0)free_node_list_check(i,i+1);
                if(state_list[i+cell_width-1] == 0)free_node_list_check(i,i+cell_width-1);
                if(state_list[i+cell_width] == 0)free_node_list_check(i,i+cell_width);
                if(state_list[i+cell_width+1] == 0)free_node_list_check(i,i+cell_width+1);

                ROS_INFO("%d",i);
            }
        }

        //debug
        for(int i=0;i<cell_width;i++){
            for(int j=0;j<cell_height;j++){
                printf("%4d",state_list[i*(cell_width)+j]);
            }
            printf("\n");
        }

        //セルを一通り計算したので、
        //次にオープンリストにするノードをオープンリストにする
        for(int i = 0; i< cell_length;i++){
            if(state_list[i] == 2){
                state_list[i] = 1;
            }
        }
        
        //オープンリストの中にゴールの中に到達したものがあれば終了
        if(state_list[(cell_width)*goal_y+goal_x] == 1){
            ROS_INFO("serch path fin!");
            break;
        }

        //オープンリストがなくなったら失敗と判定
        //todo
        for(int i = 0; i< cell_width;i++){
            for(int j = 0; j<cell_height;j++){
                if(state_list[(cell_width*i)+j] == 1){
                }
            }
        }
    }

    ROS_INFO("robot path plan ready");

    for(int i=0;i<11;i++){
        for(int j=0;j<11;j++){
            ROS_INFO("%d_%d = %d",i,j,cost_list[(cell_width)*j+i].cost);
        }
    }
    
    geometry_msgs::Pose2D start_position;
    geometry_msgs::Pose2D goal_position;

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
    ROS_INFO("end");
}

//隣接したノードがフリーリストなら、次のオープンリストに登録してコスト計算
//そして、元のノードを親として登録
void diijkstra::free_node_list_check(int parent,int now){
    //次のオープンリストに登録
    state_list[now] = 2;
    //コスト計算
    cost_list[now].cost = cost_list[parent].cost+1;
    //親を登録
    cost_list[now].parent = parent;
}

//隣接したノードがオープンリストなら、計算したコストと登録されたコストを比較して計算コストの方が小さければコストを更新する
//そして、元のノードを親として登録
void diijkstra::open_node_list_check(int parent,int now){
    if(cost_list[parent].cost + 1 < cost_list[now].cost){
        cost_list[now].cost = cost_list[parent].cost + 1;
        cost_list[now].parent = parent;
    }
}

//隣接したノードがクローズリストなら、計算したコストと登録されたコストを比較して計算コストの方が小さければコストを更新する。
//そして隣接したノードを次のオープンリストにする
//そして、元のノードを親として登録
void diijkstra::close_node_list_check(int parent,int now){
    //コスト計算して，以前の計算したものより小さければ更新して次のオープンリストに登録する
    //そして元のノードを親として登録
    if(cost_list[parent].cost + 1 < cost_list[now].cost){
        cost_list[now].cost = cost_list[parent].cost + 1;
        state_list[now] = 2;
        cost_list[now].parent = parent;
    }
}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "diijkstra");
	diijkstra diijkstra;
	ros::spin();
	return 0;
}
