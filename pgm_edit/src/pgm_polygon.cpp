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
    void map_polygon(const geometry_msgs::Polygon::ConstPtr &data);

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

    //ファイル編集変数
    FILE *fp;
    //画像の縦・横・深度
    int width,height,depth;
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
    map_polygon(data);

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

//mapのコピーと編集の関数
void pgm_polygon::map_polygon(const geometry_msgs::Polygon::ConstPtr &data){
    //地図データの読み出し
    fp = fopen(output_pgm.c_str(),"rb");

    //1.マジックナンバー
    char type[256];
    fread(type,sizeof(char),3,fp);
    type[3] = '\0';
    //mapデータのpgmのはずなので、P5の場合は実行しない
    if(strcmp("P5\n",type) != 0){
        std::cout << type << std::endl;
        return;
    }

    //2.コメント
    int pos = ftell(fp);
    while(fgetc(fp) == '#'){
        while(fgetc(fp) != '\n'){
        }
        pos = ftell(fp);
    }

    fseek(fp,pos,SEEK_SET);

    //3.データの行列の数
    //4.グレースケールの最大値
    fscanf(fp,"%d %d %d\n",&width,&height,&depth);

    //5.データ(P5は1byteずつ)
    //データの数でbufferを動的に確保
    int **buffer;
    buffer = (int**)malloc(sizeof(int*)*width);
    for(int i=0;i<width;i++) buffer[i] = (int*)malloc(sizeof(int)*height);

    //データをバッファに格納
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++)
        {
            buffer[i][j] = fgetc(fp);
        }
    }

    //読み込みで開いたファイルを閉じる
    fclose(fp);

    //書き込みモードで開く
    fp = fopen(output_pgm.c_str(),"wb");

    //1.マジックナンバー
    fprintf(fp, "P5\n");

    //3.データの行列の数
    fprintf(fp, "%d %d\n",width,height);

    //4.グレースケールの最大値
    fprintf(fp, "%d\n",depth);

    //polygonで受け取ったデータからmapデータを編集
    for(int i = 0;i< data->points.size(); i++){
        buffer[(int)data->points[i].x][(int)data->points[i].y] = (int)data->points[i].z;
    }

    //5.データ(P5は1byteずつ)
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++)
        {
            /*
            if(!(i%10)){
                fwrite(&buffer[0][0],sizeof(unsigned char),1,fp);
            }else{
                fwrite(&buffer[i][j],sizeof(unsigned char),1,fp);
            }
            */
            
            fwrite(&buffer[i][j],sizeof(unsigned char),1,fp);
        }
    }

    
    /*
    for(int i = 0;i< data->points.size(); i++){
        fwrite(&buffer[(int)data->points[i].x][(int)data->points[i].y],sizeof(unsigned char),(int)data->points[i].z,fp);
    }
    */

    //書き込みで開いたファイルを閉じる
    fclose(fp);

    //バッファを開放
    for(int i=0;i<width;i++) free(buffer[i]);
    free(buffer);
}
//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pgm_polygon");
	pgm_polygon pgm_polygon;
	ros::spin();
	return 0;
}
