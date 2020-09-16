//--------------------------------------------------
// UDP socket 通信により
// NUC2へステアリング出力角[rad]
// を送信する．
//--------------------------------------------------

#include <string>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

//UDP通信用Qtライブラリを使用
#include <QString>
#include <QUdpSocket>

//クラス作るのはめんどくさいし管理がだるいので、
//全部グローバルなインスタンスでいいや

//UDP socket インスタンス
QUdpSocket sock_steer;

//UDPソケットの受付IP,ポート
QString STEER_IP = "192.168.1.12";
int STEER_PORT = 7001;

int sock_init();
int sock_send();

std::string uin_topic = "/soma/uin";

void callback(std_msgs::Float32MultiArray::ConstPtr data)
{
    //Definition send data
    struct Send_t
    {
        int mode;
        float steering;
        float s;
        float isRotReset;
        float isWeeding;
    } send;

    if (data->data[1] > 0.0)
        send.mode = 1; //Foward
    else if (data->data[1] > 0.0)
        send.mode = 2; //Backward
    else
        send.mode = 3; //Stop

    send.steering = data->data[0] / M_PI * 180.0;
    send.s = 1.0;
    send.isRotReset = false;
    send.isWeeding = false;

    sock_steer.writeDatagram((char*)&send,
    sizeof(Send_t),
    QHostAddress(STEER_IP),
    STEER_PORT);

    if(!sock_steer.waitForBytesWritten(33)){
        ROS_WARN("Timeout error");
    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "soma_steer_send_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber uin_sub = nh.subscribe<std_msgs::Float32MultiArray>(uin_topic,
                                                                        3,
                                                                        callback);


    while (ros::ok())
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
}