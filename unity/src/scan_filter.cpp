#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <sensor_msgs/CompressedImage.h>

//クラス定義(どのような構造か定義)-----------------------------------------------------
class scan_filter
{
public:
scan_filter();
private:

//コールバック関数定義
void cb_scan_filter(const sensor_msgs::LaserScan::ConstPtr &data);
void cb_image(const sensor_msgs::CompressedImage::ConstPtr &data);

//ノードハンドラ作成
ros::NodeHandle nh;

//使用変数定義
//使用するTopicの定義
ros::Subscriber sub_scan;
ros::Subscriber sub_image;
ros::Publisher  pub_scan_filter;

};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
scan_filter::scan_filter(){
    sub_scan = nh.subscribe("/scan", 5, &scan_filter::cb_scan_filter,this);

    pub_scan_filter = nh.advertise<sensor_msgs::LaserScan>("/scan_filter", 1000);
}
//関数定義-----------------------------------------------------------------------
void scan_filter::cb_scan_filter(const sensor_msgs::LaserScan::ConstPtr &data){
    //一度すべてコピー
    sensor_msgs::LaserScan send_data = *data;
    //データ数を数える
    int data_number =data->ranges.size();
    for(int i=0;i<data_number;i++){
      //nanかどうかの判定
    if(std::isnan(send_data.ranges[i])){
        send_data.ranges[i] = 0;
    }
    //infかどうかの判定
    if(std::isinf(send_data.ranges[i])){
        send_data.ranges[i] = 0;
    }
    }
    pub_scan_filter.publish(send_data);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_filter");
	scan_filter scan_filter;
	ros::spin();
	return 0;
}