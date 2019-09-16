#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

class pure_pursuit{
    public:
    pure_pursuit();
    private:
    //コールバック定義
    void sub_diijkstra_path(const nav_msgs::Path::ConstPtr &path);
    void sub_robot_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

    //関数
    void speed_control(const ros::TimerEvent&);
    //ノードハンドラ作成
    ros::NodeHandle nh;

    ros::Subscriber diijkstra_path;
    ros::Subscriber robot_position;

    ros::Publisher robot_speed;

    //変数
    geometry_msgs::Pose2D robot_pose;
    nav_msgs::Path robot_path;
    int path_length = 0;
    int path_number = 0;
    //フラグ
    int path_flag = 0;

    //P制御
    double Kp = 0.2;

    //時間の関数作成
    ros::Timer timer;
};

//コンストラクタ定義
pure_pursuit::pure_pursuit(){
    //Pub,Sub定義
    diijkstra_path = nh.subscribe("/robot_path", 5, &pure_pursuit::sub_diijkstra_path,this);
    robot_position = nh.subscribe("/amcl_pose", 5, &pure_pursuit::sub_robot_position, this);

    robot_speed = nh.advertise<geometry_msgs::Twist>("/icart/diff_drive_controller/cmd_vel", 1000, true);

    timer = nh.createTimer(ros::Duration(0.1), &pure_pursuit::speed_control,this);
}

//現在のロボットの位置を取得
void pure_pursuit::sub_robot_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    robot_pose.x = pose->pose.pose.position.x;
    robot_pose.y = pose->pose.pose.position.y;
    robot_pose.theta = tf::getYaw(pose->pose.pose.orientation);
}

//経路を取得
void pure_pursuit::sub_diijkstra_path(const nav_msgs::Path::ConstPtr &target_path){
    //パスを取得したらフラグを立てる
    path_flag = 1;
    //経路を取得
    robot_path = *target_path;
    //経路の長さを取得
    path_length = robot_path.poses.size();
    //numberをリセットする
    path_number = 0;
}

//速度計算を行う
void pure_pursuit::speed_control(const ros::TimerEvent&){
    //最後のpathならフラグを折る
    if(path_number == path_length){
        path_flag = 0;
        path_number = 0;
        //停止コマンドを送る
        geometry_msgs::Twist speed;
        speed.linear.x = 0.0;
        speed.angular.z = 0.0;
        robot_speed.publish(speed);
    }
    //pathフラグが立っていたら計算して速度を出す
    if(path_flag == 1){
        //現在位置から目標位置までの距離
        double diff_x = robot_path.poses[path_number].pose.position.x - robot_pose.x;
        double diff_y = robot_path.poses[path_number].pose.position.y - robot_pose.y;
        double L = sqrt(pow(diff_x,2) + pow(diff_y,2));
        ROS_INFO("number=%d,L=%f",path_number,L);
        //距離が10cm以内なら次のPathに設定して終了
        if(L<0.1){
            path_number++;
            return;
        }
        //ロボットと次のpathまでの角度を計算
        double alpha = atan2(diff_y,diff_x) - robot_pose.theta;

        ROS_INFO("alpha=%f path_theta=%f,robot_theta=%f",alpha,atan2(diff_y,diff_x)*180/M_PI,robot_pose.theta*180/M_PI);
        //P制御で適当にVrを決める
        double vr = L * Kp;
        //Wrを式から求める
        double wr = (2.0 * vr * sin(alpha)) / L ;

        geometry_msgs::Twist speed;
        
        if((alpha < 0.5) && (-0.5 < alpha)){
            speed.linear.x = vr;
            speed.angular.z = wr;
        }else if((alpha < 2*M_PI) && (2*M_PI-0.5 < alpha)){
            speed.linear.x = vr;
            speed.angular.z = wr;
        }else if((alpha > -2*M_PI) && (-(2*M_PI-0.5) > alpha)){
            speed.linear.x = vr;
            speed.angular.z = wr;
        }else{
            speed.linear.x = 0.0;
            speed.angular.z = wr;
        }
        
        robot_speed.publish(speed);
    }
}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pure_pursuit");
	pure_pursuit pure_pursuit;
	ros::spin();
	return 0;
}
