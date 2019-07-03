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
    std::cout << username << std::endl;

    //mapのパスは/home/(username)/maps/map_test.pgmにある前提
    std::string home = "/home/";
    std::string map = "/maps/map_test.pgm";
    //パスを結合
    std::string filename = home + username + map;
    std::cout << filename << std::endl;

    //mapをオープンして読み込み
    fp = fopen(filename.c_str(),"rb");
    //fp = fopen("/home/masanari/maps/map_test.pgm","rb");

    
    char type[256];
    //fpからcharバイトのデータを3つ読む
    //読みだすたびにfpの指定位置が変化する
    fread(type,sizeof(char),3,fp);
    
    for(int i=0;i<3;i++){
        std::cout << type[i] << std::endl;
    }
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

    /*
    //どのファイルを読むか指定
    std::ifstream infile("/home/masanari/maps/map_test.pgm");
    std::stringstream ss;
    std::string inputline = "";

    //pgm形式は
    //1.マジックナンバーP2 or P5
    //2.コメント
    //3.データの行列の数
    //4.グレースケールの最大値
    //5.データ(P5は1byteずつ)

    //バージョンを吐き出す
    //infileの中の行末までを読みだす
    std::getline(infile,inputline);
    ROS_INFO("%s",inputline);
    std::cout << inputline << std::endl;

    // Second line : comment
    std::getline(infile,inputline);
    std::cout << "Comment : " << inputline << std::endl;
    */

    /*
    std::getline(infile,inputline);
    std::cout << "row and line : " << inputline << std::endl;

    std::getline(infile,inputline);
    std::cout << "data_max : " << inputline << std::endl;

    std::getline(infile,inputline);
    std::cout << "data : " << inputline << std::endl;
    */

   /*
    // Continue with a stringstream
    ss << infile.rdbuf();
    // Third line : size
    //ssは空白で区切られて書き込むから，for文みたいに増やしていく必要はない
    ss >> numcols >> numrows;
    std::cout << numcols << " columns and " << numrows << " rows" << std::endl;

    // fourth line ; data_max
    ss >> grey_max;
    std::cout << "grey max" << grey_max << std::endl;

    numcols = 10;
    numrows = 10;
    array = new int*[numrows];
    for(int i = 0; i< numrows;++i){
        array[i] = new int [numcols];
    }

    
    for(row = 0; row < numrows; ++row){
        for (col = 0; col < numcols; ++col){
            ss >> array[row][col];
            //array[row][col] = fgetc(ss);
        }
    }
    for(row = 0; row < numrows; ++row) {
        for(col = 0; col < numcols; ++col) {
            std::cout << array[row][col] << std::endl;
        }
    }
    */

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
