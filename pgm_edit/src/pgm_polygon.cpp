#include <ros/ros.h>
#include <iostream>
#include <fstream> // ifstream
#include <sstream> // stringstream

//コンストラクタ
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Polygon.h>

//cb_polygon
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

//コンストラクタでtmpのコピーファイルを作成
//polygonのコールバックで，map_serviceを落としtmpファイルを読み込み全部コピーからの指定した場所の情報を変更

class pgm_polygon{
public:
    pgm_polygon();
private:
    //コールバック関数
    void cb_polygon(const geometry_msgs::Polygon::ConstPtr &data);
    void map_server_request(bool request);

    //ノードハンドラ作成
    ros::NodeHandle nh;
    ros::Subscriber sub_polygon;

    ros::ServiceClient service_client;

    //map_serverが読み込んだパス
    std_msgs::String map_path;

    //map.pgmのコピー先のパス
    std::string output_pgm;
    //map.yamlのコピー先のパス
    std::string output_yaml;
    //mapのコピー先に必要なパス変数
    char *username;
    std::string home = "/home/";
};

//コンストラクタ
//コンストラクタで元の地図データをコピーする
pgm_polygon::pgm_polygon(){
    //購読するトピックの定義
    sub_polygon = nh.subscribe("/map_polygon",5,&pgm_polygon::cb_polygon,this);

    //サービスの定義
    service_client = nh.serviceClient<std_srvs::SetBool>("map_server_enable");

    //map_serverが読み込んだmapのパスを取得
    nh.getParam("map_path", map_path.data);
    if(nh.hasParam("map_path")){
        //ROS_INFO("%s",map_path.data.c_str());
        //output_pathに元のmap.pgmとmap.yamlをコピー
        //usernameを取得
        username = std::getenv("USER");

         //map.pgmをコピー
        output_pgm = home + username + "/maps_temp/map.pgm";
        try {
            boost::filesystem::copy_file(map_path.data + "/map.pgm", output_pgm, boost::filesystem::copy_option::overwrite_if_exists);
        }catch (boost::filesystem::filesystem_error& ex) {
            std::cout << ex.what() << std::endl;
            throw;
        }

        //map.yamlをコピー
        output_yaml = home + username + "/maps_temp/map.yaml";
        try {
            boost::filesystem::copy_file(map_path.data + "/map.yaml", output_yaml, boost::filesystem::copy_option::overwrite_if_exists);
        }catch (boost::filesystem::filesystem_error& ex) {
            std::cout << ex.what() << std::endl;
            throw;
        }
    }else{
        //地図のパスを見つけられなかったら終了させる
        ROS_INFO("Can't find map_path!!");
        ros::shutdown();
    }
}

void pgm_polygon::cb_polygon(const geometry_msgs::Polygon::ConstPtr &data){
    //map_serverを落とすリクエストをサービス経由で出す
    //落とすのでfalseを送信
    map_server_request(false);

    //maps_tempの中の地図データを編集する
    //地図データのコピーを行う

    //maps/tempの中の地図データからmap_serverを立ち上げる
    //map_serverを起動するリクエストをサービス経由で出す
    map_server_request(true);
    
    //amclのサービスからロボット位置を新しいmapの中で認識させる
    
}

//map_serverの起動関数
void pgm_polygon::map_server_request(bool request){
    std_srvs::SetBool map_flag;
    map_flag.request.data =request;
    if(!(service_client.call(map_flag))){
        //失敗したら何もしない
        ROS_ERROR("Failed map_server request!!");
        return;
    }
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pgm_polygon");
	pgm_polygon pgm_polygon;
	ros::spin();
	return 0;
}
