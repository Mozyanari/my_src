#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloud2_filter{
    public:
    PointCloud2_filter();
    private:
    //コールバック
    void cb_pointcloud2_filter(const sensor_msgs::PointCloud2::ConstPtr& data);

    //タイマーで起動するpublish関数
    void pub_scan_PointCloud2(const ros::TimerEvent&);
    //ノードハンドラ作成
    ros::NodeHandle nh;

    ros::Publisher pub_pointcloud2_filter;
    ros::Subscriber sub_pointcloud2;

    //保持するPointCloud2データ
    sensor_msgs::PointCloud2 recent_pointcloud;

    //時間の関数作成
    ros::Timer timer;

    //PointCloudを何分の1にするか設定
    int reduce_size = 7;

    //何Hzでデータをpublishするか
    double pub_hz = 2.0;
};

struct PointCloud2_data
{
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};
struct PointCloud_cast{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};


//コンストラクタ
PointCloud2_filter::PointCloud2_filter(){
    sub_pointcloud2 = nh.subscribe("/depth_camera_link/points", 5, &PointCloud2_filter::cb_pointcloud2_filter,this);
    pub_pointcloud2_filter = nh.advertise<sensor_msgs::PointCloud2>("scan_PointCloud2", 1000);

    //timer定義
    timer = nh.createTimer(ros::Duration(1.0/pub_hz), &PointCloud2_filter::pub_scan_PointCloud2,this);
}

//関数定義-----------------------------------------------------------------------
void PointCloud2_filter::cb_pointcloud2_filter(const sensor_msgs::PointCloud2::ConstPtr& data){
    //データ取得だけを行う
    recent_pointcloud = *data;
}

void PointCloud2_filter::pub_scan_PointCloud2(const ros::TimerEvent&){
    if(recent_pointcloud.data.size() != 0){
        //PointCloudの情報を取得
        int x_offset = recent_pointcloud.fields[0].offset;
        int y_offset = recent_pointcloud.fields[1].offset;
        int z_offset = recent_pointcloud.fields[2].offset;
        int rgb_offset = recent_pointcloud.fields[3].offset;
        int data_step = recent_pointcloud.point_step;
        int data_width = recent_pointcloud.width;
        int data_height = recent_pointcloud.height;
        int data_raw_step = recent_pointcloud.row_step;

        //変換後のPointCloudのサイズ
        int cast_width = data_width / reduce_size;
        int cast_height = data_height /reduce_size;
        int cast_size = cast_width * cast_height;
        //ROS_INFO("%d",cast_size);

        //もし、cast_width、cast_heightが0なら縮小しすぎで失敗とする
        if(cast_width == 0 && cast_height == 0){
            ROS_ERROR("Reduce size Error");
            throw;
        }

        //二次元配列を動的に確保
        PointCloud2_data *temp_data = new PointCloud2_data[cast_size];

        //データを取得
        /*
        for(int i=0;i<cast_size;i++){
            //データ取得
            memcpy(&temp_data[i].x, &recent_pointcloud.data[i*reduce_size*data_step + x_offset], sizeof(float));
            memcpy(&temp_data[i].y, &recent_pointcloud.data[i*reduce_size*data_step + y_offset], sizeof(float));
            memcpy(&temp_data[i].z, &recent_pointcloud.data[i*reduce_size*data_step + z_offset], sizeof(float));
            memcpy(&temp_data[i].r, &recent_pointcloud.data[i*reduce_size*data_step + rgb_offset + 0], sizeof(uint8_t));
            memcpy(&temp_data[i].g, &recent_pointcloud.data[i*reduce_size*data_step + rgb_offset + 1], sizeof(uint8_t));
            memcpy(&temp_data[i].b, &recent_pointcloud.data[i*reduce_size*data_step + rgb_offset + 2], sizeof(uint8_t));
            //ROS_INFO("%d x=%f y=%f z=%f",i,temp_recent_pointcloud.x,temp_recent_pointcloud.y,temp_recent_pointcloud.z);
        }
        */
        int k=0;
        for(int i = reduce_size-1; i<data_height; i+=reduce_size){
            for(int j = reduce_size-1; j<data_width; j+=reduce_size){
                memcpy(&temp_data[k].x, &recent_pointcloud.data[(i*data_raw_step) + j*data_step + x_offset], sizeof(float));
                memcpy(&temp_data[k].y, &recent_pointcloud.data[(i*data_raw_step) + j*data_step + y_offset], sizeof(float));
                memcpy(&temp_data[k].z, &recent_pointcloud.data[(i*data_raw_step) + j*data_step + z_offset], sizeof(float));
                memcpy(&temp_data[k].r, &recent_pointcloud.data[(i*data_raw_step) + j*data_step + rgb_offset + 0], sizeof(uint8_t));
                memcpy(&temp_data[k].g, &recent_pointcloud.data[(i*data_raw_step) + j*data_step + rgb_offset + 1], sizeof(uint8_t));
                memcpy(&temp_data[k].b, &recent_pointcloud.data[(i*data_raw_step) + j*data_step + rgb_offset + 2], sizeof(uint8_t));
                //ROS_INFO("i=%d j=%d",i,j);
                //ROS_INFO("x=%d",(i*data_raw_step-1) + j*data_step + x_offset);
                k++;
            }
        }

        //キャスト変換
        PointCloud_cast *cast_data = new PointCloud_cast[cast_size];
        for(int i=0;i<cast_size;i++){
            cast_data[i].x = (int16_t)(temp_data[i].x * 100);
            cast_data[i].y = (int16_t)(temp_data[i].y * 100);
            cast_data[i].z = (int16_t)(temp_data[i].z * 100);
            cast_data[i].r = temp_data[i].r;
            cast_data[i].g = temp_data[i].g;
            cast_data[i].b = temp_data[i].b;
        }

        //デバック
        for(int i = 0;i<9;i++){
            //ROS_INFO("%d row r=%d g=%d b=%d",i,temp_data[i].r,temp_data[i].g,temp_data[i].b);
            //ROS_INFO("%d row x=%f cast x=%d",i,temp_data[i].x,cast_data[i].x);
            //ROS_INFO("%d x=%d y=%d z=%d",i,cast_data[i].x,cast_data[i].y,cast_data[i].z);
        }
        //ROS_INFO("x = %d y = %d z =%d",cast_data[0].x,cast_data[0].y, cast_data[0].z);


        /*
        想定している圧縮したPointCloudのデータ形式
        01 23 45 6 7 8
        x  y  z  r g b
        つまり1ピクセルにつき9byte必要
        また、m単位からcmに変更
        */
        int cast_point_step = 9;

        //データを入力
        sensor_msgs::PointCloud2 filter_data;
        filter_data.data.resize(cast_point_step*cast_size);

        //変換後のデータの情報
        int cast_x_offset = 0;
        int cast_y_offset = 2;
        int cast_z_offset = 4;
        int cast_rgb_offset = 6;
        int cast_row_step = cast_width * cast_point_step;

        for(int i=0;i<cast_size;i++){
            memcpy(&filter_data.data[i*cast_point_step + cast_x_offset], &cast_data[i].x, sizeof(int16_t));
            memcpy(&filter_data.data[i*cast_point_step + cast_y_offset], &cast_data[i].y, sizeof(int16_t));
            memcpy(&filter_data.data[i*cast_point_step + cast_z_offset], &cast_data[i].z, sizeof(int16_t));
            memcpy(&filter_data.data[i*cast_point_step + cast_rgb_offset + 0], &cast_data[i].r, sizeof(uint8_t));
            memcpy(&filter_data.data[i*cast_point_step + cast_rgb_offset + 1], &cast_data[i].g, sizeof(uint8_t));
            memcpy(&filter_data.data[i*cast_point_step + cast_rgb_offset + 2], &cast_data[i].b, sizeof(uint8_t));
        }

        //キャストしたPointCloudの情報を代入
        filter_data.width = cast_width;
        filter_data.height = cast_height;
        //headerとbigendianはそのままコピー
        filter_data.header = recent_pointcloud.header;
        filter_data.is_bigendian = recent_pointcloud.is_bigendian;
        filter_data.row_step = cast_row_step;
        filter_data.point_step = cast_point_step;

        //x,y,z,rgb4つのfield
        filter_data.fields.resize(4);
        filter_data.fields[0].name = "x";
        filter_data.fields[0].datatype = 9;
        filter_data.fields[0].offset = cast_x_offset;
        filter_data.fields[0].count = 1;

        filter_data.fields[1].name = "y";
        filter_data.fields[1].datatype = 9;
        filter_data.fields[1].offset = cast_y_offset;
        filter_data.fields[1].count = 1;

        filter_data.fields[2].name = "z";
        filter_data.fields[2].datatype = 9;
        filter_data.fields[2].offset = cast_z_offset;
        filter_data.fields[2].count = 1;

        filter_data.fields[3].name = "rgb";
        filter_data.fields[3].datatype = 9;
        filter_data.fields[3].offset = cast_rgb_offset;
        filter_data.fields[3].count = 1;

        pub_pointcloud2_filter.publish(filter_data);

        //動的に確保した構造体を削除
        delete []temp_data;
        delete []cast_data;
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