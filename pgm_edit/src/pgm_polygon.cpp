#include <ros/ros.h>
#include <iostream>
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

//コンストラクタでtmpのコピーファイルを作成
//polygonのコールバックで，tmpファイルを読み込み全部コピーからの指定した場所の情報を変更

class pgm_polygon{
public:
    pgm_polygon();
private:
    //コールバック関数
    
    //ノードハンドラ作成
    //nh("~")にすることで
    ros::NodeHandle nh;

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


//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pgm_polygon");
	pgm_polygon pgm_polygon;
	ros::spin();
	return 0;
}
