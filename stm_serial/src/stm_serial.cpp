#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class stm_serial{
    public:
    stm_serial();
    private:
    void cb_twist_serial(const geometry_msgs::Twist::ConstPtr &twist);

    //シリアル通信関係
    int open_serial(const char *device_name);
    void serial_callback(const std_msgs::String& serial_msg);
    int fd1;
};

//コンストラクタ
stm_serial::stm_serial(){
    //指定した名前のシリアルポートを開く
    char device_name[]="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF555054877567044633-if02";
    fd1=open_serial(device_name);
    if(fd1<0){
        ROS_ERROR("Serial Fail: cound not open %s", device_name);
        printf("Serial Fail\n");
        ros::shutdown();
    }
    
}

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
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    //non blocking
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    //store configuration
    tcsetattr(fd1,TCSANOW,&conf_tio);
    return fd1;
}


void stm_serial::serial_callback(const std_msgs::String& serial_msg){
    int rec=write(fd1,serial_msg.data.c_str(),serial_msg.data.size());
    if(rec>=0)printf("send:%s\n",serial_msg.data.c_str());
    else{
        ROS_ERROR_ONCE("Serial Fail: cound not write");
    }
}


//受信したtwistをserial通信で送信
void stm_serial::cb_twist_serial(const geometry_msgs::Twist::ConstPtr &twist){

}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "stm_serial");
	stm_serial stm_serial;
	ros::spin();
	return 0;
}