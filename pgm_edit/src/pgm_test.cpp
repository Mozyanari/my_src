#include <ros/ros.h>
#include <iostream>
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <std_msgs/String.h>

#include <boost/filesystem.hpp>



class pgm_test{
public:
    pgm_test();
private:
    //コールバック関数

    //ノードハンドラ作成
    ros::NodeHandle nh;

    int row = 0, col = 0, numrows = 0, numcols = 0;
    int **array;
    int grey_max;
    FILE *fp;

    int black_count=0;
    int grey_count=0;
    int white_count=0;
};

//コンストラクタ
pgm_test::pgm_test(){
    
    //mapを読み込み
    //usernameを取得
    char *username;
    username = std::getenv("USER");
    //std::cout << username << std::endl;

    //mapのパスは/home/(username)/maps/map_test.pgmにある前提
    std::string home = "/home/";
    std::string map = "/maps/map_test.pgm";
    //パスを結合
    std::string filename = home + username + map;
    //std::cout << filename << std::endl;

    //mapをオープンして読み込み
    //pgm形式は
    //1.マジックナンバーP2 or P5
    //2.コメント
    //3.データの行列の数
    //4.グレースケールの最大値
    //5.データ(P5は1byteずつ)
    fp = fopen(filename.c_str(),"rb");
    //fp = fopen("/home/masanari/maps/map_test.pgm","rb");

    
    char type[256];
    //fpからcharバイトのデータを3つ読む
    //読みだすたびにfpの指定位置が変化する
    fread(type,sizeof(char),3,fp);
    type[3] = '\n';
    //mapデータのpgmのはずなので、P5の場合は実行しない
    if(strcmp("P5\n",type) != 0){
        return;
    }
    /*
    for(int i=0;i<3;i++){
        std::cout << type[i] << std::endl;
    }
    */
    //出力結果
    //P5
    //

    //P5の下には空白がある

    //コメント行への対処
    int pos = ftell(fp);
    while(fgetc(fp) == '#'){
        while(fgetc(fp) != '\n'){
        }
        pos = ftell(fp);
    }

    fseek(fp,pos,SEEK_SET);

    //画像の幅、高さ、色深度取得
    int width,height,depth;
    fscanf(fp,"%d %d %d¥n",&width,&height,&depth);

    std::cout << width << std::endl;
    std::cout << height << std::endl;
    std::cout << depth << std::endl;

    //データの間のwhitespaceを一つ埋める
    fseek(fp,1,SEEK_CUR);

    int **buffer;
    buffer = (int**)malloc(sizeof(int*)*width);
    for(int i=0;i<width;i++) buffer[i] = (int*)malloc(sizeof(int)*height);

    //画素値を読み込んでいく
    //白は254
    //黒は0
    //グレーは205
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++)
        {
            buffer[i][j] = fgetc(fp);
            if(buffer[i][j] == 0){
                black_count++;
                std::cout << 1;
            }
            else if(buffer[i][j] == 205){
                grey_count++;
                std::cout << 0;
            }else if(buffer[i][j] == 254){
                white_count++;
                std::cout << 2;
            }else{
                std::cout << 3;
            }
        }
        std::cout << std::endl;
    }
    std::cout << "black" << black_count <<  std::endl;
    std::cout << "white" << white_count <<  std::endl;
    std::cout << "grey" << grey_count <<  std::endl;
    //ファイルを閉じる
    fclose(fp);

    //mapをコピー
    std::string exitname = home + username + "/map_temp.pgm";
    
    try {
        boost::filesystem::copy_file(filename, exitname);
    }
    catch (boost::filesystem::filesystem_error& ex) {
        std::cout << ex.what() << std::endl;
        throw;
    }

}

//関数定義-----------------------------------------------------------------------


//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pgm_test");
	pgm_test pgm_test;
	ros::spin();
	return 0;
}
