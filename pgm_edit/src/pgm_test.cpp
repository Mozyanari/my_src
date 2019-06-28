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
};

//コンストラクタ
pgm_test::pgm_test(){
    //どのファイルを読むか指定
    std::ifstream infile("/home/masanari/maps/map_test.pgm");
    std::stringstream ss;
    std::string inputline = "";

    //バージョンを吐き出す
    //infileの中の行末までを読みだす
    std::getline(infile,inputline);
    ROS_INFO("%s",inputline);
    std::cout << inputline << std::endl;
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
