#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class stm_serial{
    public:
    stm_serial();
    private:
    void cb_twist_serial(const geometry_msgs::Twist::ConstPtr &twist);
    void polling_odom(const ros::TimerEvent&);

    //ノードハンドラ作成
	ros::NodeHandle nh;
    //pub,sub定義
    ros::Publisher pub_odom;
    ros::Subscriber sub_twist;

    //ポーリングのためのタイマー
    ros::Timer timer;
    double poling_Hz = 100.0;
    
    //シリアル通信関係
    int open_serial(const char *device_name);
    void serial_callback(const std_msgs::String& serial_msg);
    int fd1;

    //保存するオドメトリ
    nav_msgs::Odometry odom;
};

//コンストラクタ
stm_serial::stm_serial(){
    //pub・sub設定
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom",1000);
    sub_twist = nh.subscribe("cmd_vel", 5, &stm_serial::cb_twist_serial,this);

    //timer設定
    timer = nh.createTimer(ros::Duration(1.0/poling_Hz), &stm_serial::polling_odom,this);

    //オドメトリの初期化
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;

    //指定した名前のシリアルポートを開く
    //char device_name[]="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF555054877567044633-if02";
    char device_name[]="/dev/ttyACM0";;
    //char device_name[]="/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM009HAY-if00-port0";
    fd1=open_serial(device_name);
    if(fd1<0){
        ROS_ERROR("Serial Fail: cound not open %s", device_name);
        printf("Serial Fail\n");
        ros::shutdown();
    }
}

//シリアルポートを開く関数
int stm_serial::open_serial(const char *device_name){
    int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd1, F_SETFL,0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd1,&conf_tio);
    //set baudrate
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON | INLCR);
    //0x0aを送るときに、0x0d,0x0aとなるのを防ぐ
    conf_tio.c_oflag &= ~(ONLCR);
    //printf("%d",conf_tio.c_oflag);
    //non blocking
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    //store configuration
    tcsetattr(fd1,TCSANOW,&conf_tio);
    return fd1;
}

//ポーリングしてstmからのオドメトリ情報を取得
void stm_serial::polling_odom(const ros::TimerEvent&){
    //オドメトリはx,y,yawの変化をcmの単位で受信する
    char buf[3]={0};
    int recv_data=read(fd1, buf, sizeof(buf));
    if(recv_data>0){
        odom.pose.pose.position.x += (double)buf[0] / 100.0;
        //odom.
        printf("recv:%03d %s\n",recv_data,buf);
        /*
        std_msgs::String serial_msg;
        serial_msg.data=buf;
        serial_pub.publish(serial_msg);
        */
    }else{
        printf("no data\n");
    }
}
//受信したtwistをserial通信で送信
void stm_serial::cb_twist_serial(const geometry_msgs::Twist::ConstPtr &twist){
    //twistの中のlinear.xとangular.zだけを送信する
    //また、単位をcmにして送信する
    int16_t v = (int16_t)(twist->linear.x * 100.0);
    int16_t w = (int16_t)(twist->angular.z * 100.0);
    char buff[4];
    //vをbuff[0],buff[2]に格納
    memcpy(&buff[0],&v,sizeof(int16_t));
    memcpy(&buff[2],&w,sizeof(int16_t));
    //printf("data[0]=%s data[1]=%s",buff[0],buff[1]);
    int rec=write(fd1,buff,sizeof(buff));
    printf("%d",sizeof(buff));
    //printf("[0]=%s [1]=%s [2]=%s [3]=%s size=%d",buff[0],buff[1],buff[2],buff[3],sizeof(buff));
    if(rec>=0)printf("send:\n");
    else{
        ROS_ERROR_ONCE("Serial Fail: cound not write");
    }
    
}



//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "stm_serial");
	stm_serial stm_serial;
	ros::spin();
	return 0;
}