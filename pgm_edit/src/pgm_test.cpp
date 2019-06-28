#include <ros/ros.h>
#include <iostream>
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <std_msgs/String.h>



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
};

//コンストラクタ
pgm_test::pgm_test(){
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

    /*
    std::getline(infile,inputline);
    std::cout << "row and line : " << inputline << std::endl;

    std::getline(infile,inputline);
    std::cout << "data_max : " << inputline << std::endl;

    std::getline(infile,inputline);
    std::cout << "data : " << inputline << std::endl;
    */
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
