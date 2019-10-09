#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloud2_filter{
    public:
    PointCloud2_filter();
    private:
    //コールバック
    void cb_pointcloud2_filter(const sensor_msgs::PointCloud2::ConstPtr& data);

    //ノードハンドラ作成
    ros::NodeHandle nh;

    ros::Publisher pub_pointcloud2_filter;
    ros::Subscriber sub_pointcloud2;
};

//コンストラクタ
PointCloud2_filter::PointCloud2_filter(){
    sub_pointcloud2 = nh.subscribe("/depth_camera_link/points", 5, &PointCloud2_filter::cb_pointcloud2_filter,this);
    pub_pointcloud2_filter = nh.advertise<sensor_msgs::PointCloud2>("scan_PointCloud2", 1000);
}

//関数定義-----------------------------------------------------------------------
void PointCloud2_filter::cb_pointcloud2_filter(const sensor_msgs::PointCloud2::ConstPtr& data){
    //
    sensor_msgs::PointCloud2 filter_data;

    //PointCloudの情報を取得
    int x_offset = data->fields[0].offset;
    int y_offset = data->fields[1].offset;
    int z_offset = data->fields[2].offset;
    int data_step = data->point_step;
    //二次元配列
    std::vector<std::vector<double> > point(9, std::vector<double>(10));
    //デバック
    for(int i=0;i<4;i++){
        for(int j=0;j<3;j++){
            //データ取得
            if(j == 1){
                float x = 0;
                float y = 0;
                float z = 0;
                /*
                uint8_t x_temp[4];
                for(int k=0;k<4;k++){
                    x_temp[k] = data->data[i*data_step + x_offset + k];
                    ROS_INFO("%d x_temp[%d]=%d",i,k,x_temp[k]);
                }
                */
                memcpy(&x, &data->data[i*data_step + x_offset], sizeof(float));
                memcpy(&y, &data->data[i*data_step + y_offset], sizeof(float));
                memcpy(&z, &data->data[i*data_step + z_offset], sizeof(float));
                ROS_INFO("%d x=%f y=%f z=%f",i,x,y,z);
            }
        }
    }
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloud2_filter");
	PointCloud2_filter PointCloud2_filter;
	ros::spin();
	return 0;
}