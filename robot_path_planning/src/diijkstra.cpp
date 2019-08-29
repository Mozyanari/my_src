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

  int *state_list;
  CostList *cost_list;

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

    int goal_x = 30;
    int goal_y = 30;

    //オープンリスト，クローズリスト，フリーリストの初期化
    //値が1ならオープン，-1ならクローズ，0ならフリー，2なら次にオープン，-100なら通れない場所
    //0で初期化
    state_list = (int *)malloc(cell_length * sizeof(int));  /* メモリ領域の確保 */
    for(int i = 0; i< cell_length;i++){
        //mapのデータは100なら障害物，0なら床，-1なら未定となる
        if(cost_map.data[i] == 100){
            state_list[i] = -100;
        }else{
            state_list[i] = 0;
        }
    }

    //Debug state
    /*
    for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",state_list[i*(cell_width)+j]);
        }
        printf("\n");
    }
    printf("\n");
    */

    //コストリストの初期化
    cost_list = (CostList *)malloc(cell_length * sizeof(CostList));  /* メモリ領域の確保 */
    for(int i = 0; i< cell_length;i++){
        cost_list[i].cost = 0;
        cost_list[i].parent = 0;
    }
    //Debug cost
    for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",cost_list[i*(cell_width)+j].cost);
        }
        printf("\n");
    }
    printf("\n");

    //スタートノードをオープンリストに入れる
    state_list[(cell_width)*start_y+start_x] = 1;
    for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",state_list[i*(cell_width)+j]);
        }
        printf("\n");
    }

    ROS_INFO("start_calclate");
    //コスト計算をする
    while(1){
        for(int i = 0; i< cell_length;i++){
            //オープンリストに入ってるノードを発見したら、クローズリストにして、隣接したノードを計算する
            if(state_list[i] == 1){
                //クローズリストにする
                state_list[i] = -1;
                //周囲のノードのチェック
                //現在のノードの位置
                //xが0なら左側、cell_widthなら右側の探索を行わない
                //yが0なら下側、cell_heightなら上の探索を行わない
                int x = i%cell_width;
                int y = i/cell_width;
                //周囲のノードの番号が範囲外かどうかチェックして，登録されているリストによって計算
                int check_number;
                //左下
                check_number = i-cell_width-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                    }
                    
                }
                //下
                check_number = i-cell_width;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //右下
                check_number = i-cell_width+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                    }
                    
                }
                //左
                check_number = i-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                    }
                }
                //右
                check_number = i+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                    }
                }
                //左上
                check_number = i+cell_width-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                    }
                }
                //上
                check_number = i+cell_width;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                }
                //右上
                check_number = i+cell_width+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number);
                    }

                }
                ROS_INFO("%d",i);
            }
        }

        //debug state
        /*
        for(int i=0;i<cell_width;i++){
            for(int j=0;j<cell_height;j++){
                printf("%4d",state_list[i*(cell_width)+j]);
            }
            printf("\n");
        }
        printf("\n");
        */

        //Debug cost
        for(int i=0;i<cell_width;i++){
            for(int j=0;j<cell_height;j++){
                printf("%4d",cost_list[i*(cell_width)+j].cost);
            }
            printf("\n");
        }
        printf("\n");
        /*
        while(1){
            printf("プログラムを終了しますか？\n");
            printf("y=終了 n=キャンセル\n");
            int c = getchar();
            if (c == 'y'){
                break;
            }
        }
        */
        /*
        for(int i=0;i<cell_width;i++){
            for(int j=0;j<cell_height;j++){
                printf("%4d",state_list[i*(cell_width)+j]);
            }
            printf("\n");
        }
        */

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

    //debug
    /*`
    for(int i=0;i<11;i++){
        for(int j=0;j<11;j++){
            ROS_INFO("%d_%d = %d",i,j,cost_list[(cell_width)*j+i].cost);
        }
    }
    */
    //Debug parent
    //ゴールが初期値
    int parent = (cell_width)*goal_y+goal_x;
    while(parent != (cell_width)*start_y+start_x){
        for(int i=0;i<cell_width;i++){
            for(int j=0;j<cell_height;j++){
                if((cell_width*i)+j == parent){
                    printf("%4d",1);
                    parent = cost_list[(cell_width*i)+j].parent;
                }else{
                    printf("%4d",0);
                }
            }
            printf("\n");
        }
        printf("\n");
        while(1){
            printf("プログラムを終了しますか？\n");
            printf("y=終了 n=キャンセル\n");
            int c = getchar();
            if (c == 'y'){
                break;
            }
        }
    }
    
    //経路をPathに変換する
    nav_msgs::Path path;
    path.header.frame_id = "map";
    int path_length = cost_list[(cell_width)*goal_y+goal_x].cost+ 1;
    int path_parent = (cell_width)*goal_y+goal_x;
    path.poses.resize(path_length);
    //まずはゴールをpathに入力
    path.poses[path_length-1].pose.position.x = goal_x;
    path.poses[path_length-1].pose.position.y = goal_y;
    //残りのスタートまでのpathを入力
    for(int i= (path_length-2); i>-1; i--){
        path_parent = cost_list[path_parent].parent;
        path.poses[i].pose.position.x = path_parent%cell_width;
        path.poses[i].pose.position.y = path_parent/cell_width;
    }

    pub_path.publish(path);
    //動的に確保した変数を開放
    free(state_list);
    free(cost_list);
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
