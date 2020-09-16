#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include <QPointF>

#include <Eigen/Dense>

#include <string>
#include <math.h>

#include "soma_ros/common.h"
#include "soma_ros/Data/Data.h"
#include "soma_ros/Behavior/StateBase/StateBase.h"
#include "soma_ros/Behavior/Stop.h"
#include "soma_ros/Behavior/MoveTo.h"
#include "soma_ros/Behavior/Home.h"
#include "soma_ros/Communicate/Communicate.h"

typedef std::string sstring;
namespace geo_msgs = geometry_msgs;

//タイマコールバックでループ処理を行う
//タイマコールバックの周期Tを設定
#define TIMER_T 0.1 //[sec]

//--------------------------------------------------
//--------------------------------------------------
//--------------------------------------------------
class Behavior
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;

    sstring frame_id, map_frame_id;
    sstring odom_topic;
    sstring cmd_topic;
    sstring cmd_vel_topic;

    //Subscriberの宣言
    ros::Subscriber clicked_point_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber odom_sub;

    //Publisherの宣言
    ros::Publisher txt_pub;
    ros::Publisher state_pub;
    ros::Publisher Pg_pub;  //
    ros::Publisher Xt_pub;  //Estimated state vector
    ros::Publisher Uin_pub; //

    Data *data;
    Communicate *comm;

    //behavior modules
    Stop *sStop;
    std::map<int, StateBase *> states;

public:
    Behavior() : nh(ros::NodeHandle()),
                 pnh(ros::NodeHandle("~"))
    {
        // topic name
        frame_id = pnh.param<sstring>("frame_id", "soma_link");
        map_frame_id = pnh.param<sstring>("map_frame_id", "map");

        odom_topic = pnh.param<sstring>("odom_topic", "/soma/odom");
        cmd_topic= pnh.param<sstring>("cmd_topic", "/soma/cmd");
        cmd_vel_topic = pnh.param<sstring>("cmd_vel_topic", "/soma/cmd_vel");

        // subscribers
        // clicked_point_sub = nh.subscribe<geo_msgs::PointStamped>("/clicked_point", 3, &Behavior::clicked_point_callback, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 3, &Behavior::odom_callback, this);
        cmd_sub = nh.subscribe<std_msgs::String>(cmd_topic, 3, &Behavior::cmd_callback, this);

        //publishers
        state_pub = nh.advertise<std_msgs::String>("/soma/state", 3);
        Pg_pub = nh.advertise<geo_msgs::PointStamped>("/soma/Pg", 3);
        Xt_pub = nh.advertise<geo_msgs::PoseStamped>("/soma/Xt", 3);
        Uin_pub = nh.advertise<std_msgs::Float32MultiArray>("/soma/uin", 3);
        txt_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/soma/txt", 3);

        data = new Data();
        comm = new Communicate();

        //make states
        sStop = new Stop();
        states[State::Stop] = sStop;

        timer = nh.createTimer(ros::Duration((double)TIMER_T), &Behavior::main, this);
    }

    ~Behavior()
    {
    }

private:
    void main(const ros::TimerEvent &e)
    {

        //
        // main process
        //

        // Finite State Machine
        int new_state = states[data->state]->Transition(data);

        if (new_state != data->state)
        {
            states[data->state]->Exit(data);
            states[new_state]->Enter(data);
            data->state = new_state;
        }
        else
            states[data->state]->Process(data);

        //--------------------------------------------------
        // finished main process
        //--------------------------------------------------


        //--------------------------------------------------
        //data publishing
        //--------------------------------------------------
        std_msgs::String smsgs;
        smsgs.data = data->state;
        state_pub.publish(smsgs);

        //Text
        jsk_rviz_plugins::OverlayText jsktxt;
        jsktxt.action = jsk_rviz_plugins::OverlayText::ADD;
        jsktxt.text += "STATE:" + State::Str.at(data->state) + "\n";
        jsktxt.text += "ACT  :" + Action::Str.at(data->action) + "\n";
        jsktxt.text += "STEER:" + std::to_string(RAD2DEG(data->Uin.lambda)) + "[deg]\n";
        jsktxt.text += "V ROT:" + std::to_string(data->Uin.v) + "[m/s]\n";
        txt_pub.publish(jsktxt);


        //Control input vector 2-d arrray
        //"/soma/uin"
        std_msgs::Float32MultiArray famsgs;
        famsgs.data.resize(2);
        famsgs.data[0] = data->Uin.lambda;
        famsgs.data[1] = data->Uin.v;
        Uin_pub.publish(famsgs);

        geo_msgs::PoseStamped posemsgs;
        posemsgs.header.frame_id = map_frame_id;
        posemsgs.header.stamp = ros::Time::now();
        posemsgs.pose = data->Xt;
        Xt_pub.publish(posemsgs);

        geo_msgs::PointStamped pmsgs;
        pmsgs.header.frame_id = map_frame_id;
        pmsgs.header.stamp = ros::Time::now();
        pmsgs.point = data->Pg;
        Pg_pub.publish(pmsgs);

        return;
    }

    // void clicked_point_callback(const geo_msgs::PointStampedConstPtr &msg)
    // {
    //     //目標座標設定(mapフレーム上でのみ許可)
    //     if (msg->header.frame_id != map_frame_id)
    //     {
    //         ROS_WARN("Check frame of clicked point");
    //         return;
    //     }

    //     data->Pg = msg->point;
    //     return;
    // }

    void cmd_callback(const std_msgs::StringConstPtr &msg)
    {
        if (msg->data == "stop")
        {
            data->action = Action::Stop;
        }

        return;
    }

    void odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        data->Xt = msg->pose.pose;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior_node");
    Behavior behavior;
    ros::spin();
    return 0;
}
