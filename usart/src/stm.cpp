#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>
#include "std_msgs/String.h"
#include <cstring>

using namespace std;
serial::Serial ser;
int main(int argc, char* argv[]){
    ros::init(argc,argv,"stm32");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("com",10);
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    unsigned short  data_size;
    unsigned char data[1024] ;
    uint8_t Rx_buf[128]={0};
    uint8_t Rx_Count = 0;
    std_msgs::String msg;
    size_t n = ser.available();//先读取串口缓存，然后就可以清空串口之前的缓存

    while(ros::ok()){
        n = ser.available();//获取串口接收到的字符数
        if(n != 0){//字符数不为0
            ser.read(data, n);//读取串口信息
            //串口接收到完整的一帧是数据/r/n，16进制表示为0x0d 0x0a
            for(int i =0; i<n;i++){
                if (data[i] != 0x0a){
                    Rx_buf[Rx_Count++] = data[i];
                } else{
                    if (Rx_buf[Rx_Count-1] == 0x0d){
                        Rx_buf[Rx_Count] = data[i];
                        cout << Rx_buf;//输出串口通信接收到的信息
                        Rx_Count=0;//清除计数
                        memset(Rx_buf,0,sizeof(Rx_buf));//清除缓冲区中的所有字符
                    }
                }
            }

        }
        ros::spinOnce();
    }
    return 0;
}