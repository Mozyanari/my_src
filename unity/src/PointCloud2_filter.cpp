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

    //PointCloudを何分の1にするか設定
    int reduce_size = 10;


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
}

//関数定義-----------------------------------------------------------------------
void PointCloud2_filter::cb_pointcloud2_filter(const sensor_msgs::PointCloud2::ConstPtr& data){
    //PointCloudの情報を取得
    int x_offset = data->fields[0].offset;
    int y_offset = data->fields[1].offset;
    int z_offset = data->fields[2].offset;
    int rgb_offset = data->fields[3].offset;
    int data_step = data->point_step;
    int data_width = data->width;
    int data_height = data->height;

    //変換後のPointCloudのサイズ
    int cast_width = data_width / reduce_size;
    int cast_height = data_height /reduce_size;
    int cast_size = cast_width * cast_height;

    //もし、cast_width、cast_heightが0なら縮小しすぎで失敗とする
    if(cast_width == 0 && cast_height == 0){
        throw;
    }

    //二次元配列を動的に確保
    PointCloud2_data *temp_data = new PointCloud2_data[cast_size];

    //データを取得
    for(int i=0;i<cast_size;i++){
        //データ取得
        memcpy(&temp_data[i].x, &data->data[i*reduce_size*data_step + x_offset], sizeof(float));
        memcpy(&temp_data[i].y, &data->data[i*reduce_size*data_step + y_offset], sizeof(float));
        memcpy(&temp_data[i].z, &data->data[i*reduce_size*data_step + z_offset], sizeof(float));
        memcpy(&temp_data[i].r, &data->data[i*reduce_size*data_step + rgb_offset + 0], sizeof(uint8_t));
        memcpy(&temp_data[i].g, &data->data[i*reduce_size*data_step + rgb_offset + 1], sizeof(uint8_t));
        memcpy(&temp_data[i].b, &data->data[i*reduce_size*data_step + rgb_offset + 2], sizeof(uint8_t));
        //ROS_INFO("%d x=%f y=%f z=%f",i,temp_data->x,temp_data->y,temp_data->z);
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
        ROS_INFO("%d row r=%d g=%d b=%d",i,temp_data[i].r,temp_data[i].g,temp_data[i].b);
        //ROS_INFO("%d row x=%f cast x=%d",i,temp_data[i].x,cast_data[i].x);
    }


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
    filter_data.header = data->header;
    filter_data.is_bigendian = data->is_bigendian;
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

    filter_data.fields[2].name = "rgb";
    filter_data.fields[2].datatype = 9;
    filter_data.fields[2].offset = cast_rgb_offset;
    filter_data.fields[2].count = 1;

    pub_pointcloud2_filter.publish(filter_data);

    //動的に確保した構造体を削除
    delete []temp_data;
    delete []cast_data;
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloud2_filter");
	PointCloud2_filter PointCloud2_filter;
	ros::spin();
	return 0;
}