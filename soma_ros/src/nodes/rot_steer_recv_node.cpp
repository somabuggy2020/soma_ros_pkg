//--------------------------------------------------
// UDP socket 通信により
// RaspiからRotaryEncoderによる速度計測値[m/s]と
// NUC2からステアリング出力角[rad]
// を受信する．
// 値はトピック名"/soma/Ut" 型"Float32MultiArray"の1次元2要素配列でパブリッシュする 
//--------------------------------------------------

#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

//UDP通信用Qtライブラリを使用
#include <QUdpSocket>

//クラス作るのはめんどくさいし管理がだるいので、
//全部グローバルなインスタンスでいいや

//UDP socket インスタンス
QUdpSocket sock_rot;
QUdpSocket sock_steer;

//UDPソケットの受付ポート
int ROT_PORT = 12346;
int STEER_PORT = 7002;

//proto type difinition
int sock_init();
int recv_sock_rot(float *v);
int recv_sock_steer(float *steer);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "soma_rot_steer_recv_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ros::Publisher v_pub, steer_pub
    ros::Publisher ut_pub;
    // v_pub = nh.advertise<std_msgs::Float32>("/soma/v_rot", 3);
    // steer_pub = nh.advertise<std_msgs::Float32>("/soma/steer", 3);
    ut_pub = nh.advertise<std_msgs::Float32MultiArray>("/soma/ut", 3);

    if(sock_init() == -1) return -1;

    float v, steer;
    v = steer = 0.0;

    while(ros::ok()){
       
        recv_sock_rot(&v);
        recv_sock_steer(&steer);

        std_msgs::Float32MultiArray famsgs;
        famsgs.data.resize(2);
        famsgs.data[0] = steer/180.0*M_PI; //to radians
        famsgs.data[1] = v;
        ut_pub.publish(famsgs);

        ROS_INFO("Ut=[%.2f, %.2f]", steer, v);
        ros::Duration(0.01).sleep();
    }

    ros::spin();
    return 0;
}

/*
UDP socket 初期化およびポートとのbind(対応付け)
*/
int sock_init()
{
    if(!sock_rot.bind(ROT_PORT)){
        ROS_ERROR("sock ROT bind error");
        return -1;
    }
    ROS_INFO("sock ROT bind success");

    if(!sock_steer.bind(STEER_PORT)){
        ROS_ERROR("sock STEER bind error");
        return -1;
    }
    ROS_INFO("sock STEER bind success");

    return 0;   //No error
}


/*
ロータリエンコーダ受信処理
*/
int recv_sock_rot(float *v)
{
     if(sock_rot.waitForReadyRead(33)){
	    // ROS_INFO("Rotary data comming (%d)",sock_rot.bytesAvailable());
            if(sock_rot.bytesAvailable() <= 0){
               ROS_INFO("Data not available");
                return -1;
            }

            //binary data structure
            struct Rot_recv_t{
                unsigned long pcount;
                double v;
            } data;

            while(sock_rot.bytesAvailable() > 0){
                sock_rot.readDatagram((char*)&data, sizeof(Rot_recv_t));
                *v = (float)data.v;
            }            
        }
        else{
           ROS_INFO("sock ROT timeout");
        }

    return 0;
}

/*
ステアリング出力角受信処理
*/
int recv_sock_steer(float *steer){    
        if(sock_steer.waitForReadyRead(33)){
            if(sock_steer.bytesAvailable() <= 0){
               ROS_INFO("Data not available");
                return -1;
            }

            //data structure
            struct Steer_recv_t{
                int state;
                double steer;
            } data;

            while(sock_steer.bytesAvailable() > 0){
                sock_steer.readDatagram((char*)&data, sizeof(Steer_recv_t));
                *steer = (float)data.steer;
            }
        }
        else{
            ROS_INFO("sock STEER timeout");
        }

    return 0;
}
