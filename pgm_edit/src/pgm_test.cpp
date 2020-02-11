#include <ros/ros.h>
#include <iostream>
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Point.h>



class pgm_test{
public:
    pgm_test();
private:
    //コールバック関数
    void cb_edit(const geometry_msgs::Point::ConstPtr &point);
    //ノードハンドラ作成
    ros::NodeHandle nh;

    ros::Subscriber sub_point;

    //変数
    int row = 0, col = 0, numrows = 0, numcols = 0;
    int **array;
    int grey_max;
    FILE *fp;

    int black_count=0;
    int grey_count=0;
    int white_count=0;
    int other_count=0;

    char *username;
    std::string home = "/home/";
    std::string map = "/maps/map_test.pgm";
    long comment_start;
    long comment_end;
    int width,height,depth;
};

//コンストラクタ
//コンストラクタで元の地図データをコピーする
pgm_test::pgm_test(){
    //購読するトピックの定義
    sub_point = nh.subscribe("/point",5,&pgm_test::cb_edit,this);

    //mapを読み込み
    //usernameを取得
    username = std::getenv("USER");
    //std::cout << username << std::endl;

    //mapのパスは/home/(username)/maps/map_test.pgmにある前提
    
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
    type[3] = '\0';
    //mapデータのpgmのはずなので、P5の場合は実行しない
    if(strcmp("P5\n",type) != 0){
        std::cout << type << std::endl;
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
    
    fscanf(fp,"%d %d %d\n",&width,&height,&depth);

    std::cout << width << std::endl;
    std::cout << height << std::endl;
    std::cout << depth << std::endl;

    //データの間のwhitespaceを一つ埋める
    //fseek(fp,1,SEEK_CUR);

    //
    //data_position = ftell(fp);

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
                //その他が一番最後に何故かある
                other_count++;
                //std::cout << 3;
                std::cout << buffer[i][j];
            }
        }
        std::cout << std::endl;
    }
    std::cout << "black" << black_count <<  std::endl;
    std::cout << "white" << white_count <<  std::endl;
    std::cout << "grey" << grey_count <<  std::endl;
    std::cout << "other" << other_count <<  std::endl;
    //ファイルを閉じる
    fclose(fp);

    //mapをコピー
    std::string exitname = home + username + "/map_temp.pgm";
    
    try {
        boost::filesystem::copy_file(filename, exitname,boost::filesystem::copy_option::overwrite_if_exists);
    }
    catch (boost::filesystem::filesystem_error& ex) {
        std::cout << ex.what() << std::endl;
        throw;
    }

    //bufferを開放
    for(int i=0;i<width;i++) free(buffer[i]);
    free(buffer);

}

//関数定義-----------------------------------------------------------------------
void pgm_test::cb_edit(const geometry_msgs::Point::ConstPtr &point){
    std::string filename = home + username + "/map_temp.pgm";

    //読み込みモードで開く→データをすべて吸い出す→書き込みモードで開いて同じようにすべて格納
    //読み込みモードで開く
    //fp = fopen(filename.c_str(),"rb");



    //読み込みモードで開いたファイルを閉じる
    //fclose(fp);
    //書き込みモードで開く
    fp = fopen(filename.c_str(),"wb");

    //マジックナンバー
    fprintf(fp, "P5\n");
    //fwrite("P5\n",1,4,fp);

    //コメント
    fprintf(fp, "#test\n");
    //fwrite("#test\n",1,7,fp);

    //データ数
    fprintf(fp, "200 200\n");
    //fwrite("2 2\n",1,5,fp);

    //データの深度
    fprintf(fp, "255\n");
    //fwrite("125\n",1,5,fp);

    //データの書き込み
    for(int i=0;i<20500;i++){
        //バイナリ形式で書き込む
        unsigned char a = i%255;
        //fprintf(fp, "%d ", a);
        fwrite(&a,sizeof(unsigned char),1,fp);
    }
    
    //fwrite("100",1,1,fp);

    //一度読み込んだ時のデータの位置にポインタを移動
    /*
    fseek(fp,data_position,SEEK_SET);
    std::cout << data_position << std::endl;
    for(int i=0;i<100000;i++){
        //std::cout << fputc(254,fp) << std::endl;
        fputc(254,fp);
    }
    */
    fclose(fp);
    
    return;
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pgm_test");
	pgm_test pgm_test;
	ros::spin();
	return 0;
}
