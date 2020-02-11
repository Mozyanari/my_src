#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/Bool.h>

#include <geometry_msgs/Pose2D.h>

#include "eigen3/Eigen/Core"


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
    void sub_robot_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

    //コストマップをロボットの大きさで作成
    int make_costmap(nav_msgs::OccupancyGrid map);

    //ダイクストラ法による経路計画
    //返り値が-1なら失敗，1なら成功
    int calc_path(void);

    void free_node_list_check(int parent,int now, int path_cost);
    void open_node_list_check(int parent,int now, int path_cost);
    void close_node_list_check(int parent,int now, int path_cost);

    //ノードハンドラ作成
    ros::NodeHandle nh;

    //コンストラクタ
    ros::Subscriber sub_goal;
    ros::Subscriber sub_grid_map;
    ros::Subscriber sub_amcl_pose;
    ros::Publisher pub_path;
    ros::Publisher pub_costmap;

    //sub_map
    nav_msgs::OccupancyGrid current_map;
    nav_msgs::OccupancyGrid cost_map;
    std_msgs::Bool receive_map_flag;

    //sub_target_place
    geometry_msgs::PoseStamped goal;

    //
    int cell_robot_x = 0;
    int cell_robot_y = 0;
    std_msgs::Bool receive_robot_flag;

    int cell_goal_x = 0;
    int cell_goal_y = 0;

    //セルの状態，コストを動的一次配列で確保する
    //セルの状態
    int *state_list;
    //セルのコスト，親
    CostList *cost_list;

};

//コンストラクタ定義
diijkstra::diijkstra(){
    //Pub,Sub定義
    sub_grid_map = nh.subscribe("/map", 5, &diijkstra::sub_map,this);
    sub_goal = nh.subscribe("/move_base_simple/goal", 5, &diijkstra::sub_target_place, this);
    sub_amcl_pose = nh.subscribe("amcl_pose",5,&diijkstra::sub_robot_position,this);

    pub_path = nh.advertise<nav_msgs::Path>("robot_path", 1000, true);
    pub_costmap = nh.advertise<nav_msgs::OccupancyGrid>("costmap",1000,true);

    receive_map_flag.data =false;
    receive_robot_flag.data = false;
}

//mapを取得
void diijkstra::sub_map(const nav_msgs::OccupancyGrid::ConstPtr& map){
    current_map = *map;
    make_costmap(current_map);
    receive_map_flag.data = true;
    ROS_INFO("receive map");
    //mapが更新されたら長さ0のpathを送信する
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(0);
    pub_path.publish(path);
}
//コストmapを作成
int diijkstra::make_costmap(nav_msgs::OccupancyGrid map){
    cost_map = map;
    //マップのセル数の取得
    int cell_width = current_map.info.width;
    int cell_height = current_map.info.height;
    int cell_length = cell_width*cell_height;
    //地図の分解能が10cm，機体本体の大きさが約40cmなので4つぶん膨らませる
    for(int count = 0;count<4;count++){
        for(int i = 0; i< cell_length;i++){
            //障害物の範囲を膨らませる
            //膨らませる予定の場所に120を入れる
            //一通り探索すると100にして障害物にする
            if(cost_map.data[i] == 100){
                //xが0なら左側、cell_widthなら右側の探索を行わない
                //yが0なら下側、cell_heightなら上の探索を行わない
                int x = i%cell_width;
                int y = i/cell_width;
                //周囲のセルの番号が範囲外かどうかチェックする
                int check_number;
                //左下
                check_number = i-cell_width-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(cost_map.data[check_number] != 100){
                            cost_map.data[check_number] = 120;
                        }
                    }
                    
                }
                //下
                check_number = i-cell_width;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(cost_map.data[check_number] != 100){
                        cost_map.data[check_number] = 120;
                    }
                }
                //右下
                check_number = i-cell_width+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(cost_map.data[check_number] != 100){
                            cost_map.data[check_number] = 120;
                        }
                    }
                }
                //左
                check_number = i-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(cost_map.data[check_number] != 100){
                            cost_map.data[check_number] = 120;
                        }
                    }
                }
                //右
                check_number = i+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(cost_map.data[check_number] != 100){
                            cost_map.data[check_number] = 120;
                        }
                    }
                }
                //左上
                check_number = i+cell_width-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(cost_map.data[check_number] != 100){
                            cost_map.data[check_number] = 120;
                        }
                    }
                }
                //上
                check_number = i+cell_width;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(cost_map.data[check_number] != 100){
                        cost_map.data[check_number] = 120;
                    }
                }
                //右上
                check_number = i+cell_width+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(cost_map.data[check_number] != 100){
                            cost_map.data[check_number] = 120;
                        }
                    }
                }
            }
        }
        for(int j = 0; j< cell_length;j++){
            if(cost_map.data[j] == 120){
                cost_map.data[j] = 100;
            }
        }
    }
    //costmapを送信
    ROS_INFO("pub cost map");
    pub_costmap.publish(cost_map);
}
//現在のロボットの位置を取得
void diijkstra::sub_robot_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    if(receive_map_flag.data == true){
        receive_robot_flag.data = true;
        double cell_resolution = current_map.info.resolution;
        double offset_x = current_map.info.origin.position.x;
        double offset_y = current_map.info.origin.position.y;

        //mapトピックの座標軸でのロボット位置
        double map_robot_x = pose->pose.pose.position.x - offset_x;
        double map_robot_y = pose->pose.pose.position.y - offset_y;

        //mapトピック座標軸でロボットのセル位置計算
        cell_robot_x = (int)((double)map_robot_x / (double)cell_resolution);
        cell_robot_y = (int)((double)map_robot_y / (double)cell_resolution);

        ROS_INFO("robot cell x = %d robot cell y = %d",cell_robot_x,cell_robot_y);

    }
}

//目標位置を取得
void diijkstra::sub_target_place(const geometry_msgs::PoseStamped::ConstPtr &target_place){
    //if(receive_map_flag.data == true && receive_robot_flag.data == true){
    if(receive_map_flag.data == true){
        goal = *target_place;

        double cell_resolution = current_map.info.resolution;
        double offset_x = current_map.info.origin.position.x;
        double offset_y = current_map.info.origin.position.y;

        //mapトピックの座標軸でのゴール位置
        double map_goal_x = goal.pose.position.x - offset_x;
        double map_goal_y = goal.pose.position.y - offset_y;

        //mapトピック座標軸でゴールのセル位置計算
        cell_goal_x = (int)((double)map_goal_x / (double)cell_resolution);
        cell_goal_y = (int)((double)map_goal_y / (double)cell_resolution);
        ROS_INFO("goal cell x = %d goal cell y = %d",cell_goal_x,cell_goal_y);

        //pathを計算
        int result = calc_path();
        if(result == -1){
            ROS_INFO("path plan false");
        }
    }
}

int diijkstra::calc_path(){
    //マップのセル数の取得
    int cell_width = current_map.info.width;
    int cell_height = current_map.info.height;
    int cell_length = cell_width*cell_height;
    ROS_INFO("width=%d height=%d",cell_width,cell_height);

    int start_cell_x = cell_robot_x;
    int start_cell_y = cell_robot_y;

    int goal_cell_x = cell_goal_x;
    int goal_cell_y = cell_goal_y;

    //スタートセル，ゴールセルが障害物なら失敗
    //または地図の範囲外でも失敗
    if((cost_map.data[(cell_width)*start_cell_y+start_cell_x] == 100) || (start_cell_x < 0) || ((cell_width-1) < start_cell_x) || (start_cell_y < 0) || ((cell_height-1) < start_cell_y)){
        ROS_INFO("start cell fail!!");
        return -1;
    }else if((cost_map.data[(cell_width)*goal_cell_y+goal_cell_x] == 100) || (goal_cell_x < 0) || ((cell_width-1) < goal_cell_x) || (goal_cell_y < 0) || ((cell_height-1) < goal_cell_y)){
        ROS_INFO("goal cell fail!!");
        return -1;
    }

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
    /*
    for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",cost_list[i*(cell_width)+j].cost);
        }
        printf("\n");
    }
    printf("\n");
    */

    //スタートノードをオープンリストに入れる
    state_list[(cell_width)*start_cell_y+start_cell_x] = 1;

    /*
    for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",state_list[i*(cell_width)+j]);
        }
        printf("\n");
    }
    */

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
                //xが0なら左側、cell_width-1なら右側の探索を行わない
                //yが0なら下側、cell_height-1なら上の探索を行わない
                int x = i%cell_width;
                int y = i/cell_width;
                //周囲のノードの番号が範囲外かどうかチェックして，登録されているリストによって計算
                int check_number;
                int linear_cost = 1;
                int diagonal_cost = 2;

                //上
                check_number = i+cell_width;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number,linear_cost);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number,linear_cost);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number,linear_cost);
                    if(state_list[check_number] == 2)open_node_list_check(i,check_number,linear_cost);
                }
                //下
                check_number = i-cell_width;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(state_list[check_number] == 0)free_node_list_check(i,check_number,linear_cost);
                    if(state_list[check_number] == 1)open_node_list_check(i,check_number,linear_cost);
                    if(state_list[check_number] == -1)close_node_list_check(i,check_number,linear_cost);
                    if(state_list[check_number] == 2)open_node_list_check(i,check_number,linear_cost);
                }
                //左
                check_number = i-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number,linear_cost);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number,linear_cost);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number,linear_cost);
                        if(state_list[check_number] == 2)open_node_list_check(i,check_number,linear_cost);
                    }
                }
                //右
                check_number = i+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number,linear_cost);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number,linear_cost);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number,linear_cost);
                        if(state_list[check_number] == 2)open_node_list_check(i,check_number,linear_cost);
                    }
                }
                
                //左下
                check_number = i-cell_width-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 2)open_node_list_check(i,check_number,diagonal_cost);
                    }
                    
                }
                //右下
                check_number = i-cell_width+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 2)open_node_list_check(i,check_number,diagonal_cost);
                    }
                    
                }
                
                //左上
                check_number = i+cell_width-1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != 0){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 2)open_node_list_check(i,check_number,diagonal_cost);
                    }
                }
                //右上
                check_number = i+cell_width+1;
                if( (check_number > 0) && (cell_length > check_number) ){
                    if(x != cell_width-1){
                        if(state_list[check_number] == 0)free_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 1)open_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == -1)close_node_list_check(i,check_number,diagonal_cost);
                        if(state_list[check_number] == 2)open_node_list_check(i,check_number,diagonal_cost);
                    }

                }
                
                //ROS_INFO("%d",i);
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
        /*
        for(int i=0;i<cell_width;i++){
            for(int j=0;j<cell_height;j++){
                printf("%4d",cost_list[i*(cell_width)+j].cost);
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
        if(state_list[(cell_width)*goal_cell_y+goal_cell_x] == 1){
            ROS_INFO("serch path fin!");
            break;
        }

        //オープンリストがなくなったら失敗と判定
        int open_list_flag = 0;
        for(int i = 0; i< cell_length;i++){
            if(state_list[i] == 1){
                open_list_flag = 1;
                break;
            }
        }
        if(open_list_flag == 0){
            return -1;
        }
    }

    
    ROS_INFO("robot path plan ready");

    //debug
    /*
    for(int i=0;i<11;i++){
        for(int j=0;j<11;j++){
            ROS_INFO("%d_%d = %d",i,j,cost_list[(cell_width)*j+i].cost);
        }
    }
    */
    //Debug parent
    /*
    //ゴールが初期値
    int parent = (cell_width)*goal_cell_y+goal_cell_x;
    while(parent != (cell_width)*start_cell_y+start_cell_x){
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
   for(int i=0;i<cell_width;i++){
        for(int j=0;j<cell_height;j++){
            printf("%4d",cost_list[i*(cell_width)+j].cost);
        }
        printf("\n");
    }
    */
    
    //経路をPathに変換する
    nav_msgs::Path path;
    path.header.frame_id = "map";

    //pathの要素数を数える
    int path_length = 1;
    int count = 0;
    int parent = (cell_width)*goal_cell_y+goal_cell_x;
    while(parent != (cell_width)*start_cell_y+start_cell_x){
        count ++;
        for(int i=0;i<cell_length;i++){
            if(i == parent){
                parent = cost_list[i].parent;
                path_length++;
                break;
            }
        }
        if(count > 1000){
            ROS_INFO("path_length fail");
            return -1;
        }
    }
    int path_parent = (cell_width)*goal_cell_y+goal_cell_x;
    path.poses.resize(path_length);
    //まずはゴールをpathに入力
    path.poses[path_length-1].pose.position.x = goal_cell_x;
    path.poses[path_length-1].pose.position.y = goal_cell_y;
    //残りのスタートまでのpathを入力
    for(int i= (path_length-2); i>-1; i--){
        path_parent = cost_list[path_parent].parent;
        path.poses[i].pose.position.x = path_parent%cell_width;
        path.poses[i].pose.position.y = path_parent/cell_width;
    }
    //cellを基準にして原点が左下したpathなので，ロボット座標を原点にしたpathに変更する
    //cellの分解能
    double cell_resolution = current_map.info.resolution;
    double offset_x = current_map.info.origin.position.x;
    double offset_y = current_map.info.origin.position.y;
    for(int i= 0; i<path_length; i++){
        path.poses[i].pose.position.x = ((cell_resolution/2) + path.poses[i].pose.position.x * cell_resolution) + offset_x;
        path.poses[i].pose.position.y = ((cell_resolution/2) + path.poses[i].pose.position.y * cell_resolution) + offset_y;
    }

    pub_path.publish(path);
    //動的に確保した変数を開放
    free(state_list);
    free(cost_list);
    ROS_INFO("end");
}

//隣接したノードがフリーリストなら、次のオープンリストに登録してコスト計算
//そして、元のノードを親として登録
void diijkstra::free_node_list_check(int parent,int now,int path_cost){
    //次のオープンリストに登録
    state_list[now] = 2;
    //コスト計算
    cost_list[now].cost = cost_list[parent].cost + path_cost;
    //親を登録
    cost_list[now].parent = parent;
}

//隣接したノードがオープンリストなら、計算したコストと登録されたコストを比較して計算コストの方が小さければコストを更新する
//そして、元のノードを親として登録
void diijkstra::open_node_list_check(int parent,int now, int path_cost){
    if(cost_list[parent].cost + path_cost < cost_list[now].cost){
        cost_list[now].cost = cost_list[parent].cost + path_cost;
        cost_list[now].parent = parent;
    }
}

//隣接したノードがクローズリストなら、計算したコストと登録されたコストを比較して計算コストの方が小さければコストを更新する。
//そして隣接したノードを次のオープンリストにする
//そして、元のノードを親として登録
void diijkstra::close_node_list_check(int parent,int now, int path_cost){
    if(cost_list[parent].cost + path_cost < cost_list[now].cost){
        cost_list[now].cost = cost_list[parent].cost + path_cost;
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
