#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>

#include "definition.h"

#define TIMER_T 0.033 //[sec]

//長いので省略しておく
namespace s_msgs = std_msgs;
namespace geo_msgs = geometry_msgs;

class Behavior
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;

    std::string base_link;
    std::string map_frame_id;
    std::string odom_id;

    //Subscriber
    ros::Subscriber clicked_point_sub;
    ros::Subscriber command_sub;
    ros::Subscriber odom_sub;

    //Publisher
    ros::Publisher state_pub;
    ros::Publisher pg_pub;
    ros::Publisher xt_pub;

    //local members
    Data_t data;
    // int state;
    // geo_msgs::PointStamped _pg;
    // int command;
    // geo_msgs::PoseStamped _xt;

public:
    Behavior() : nh(ros::NodeHandle()),
                 pnh(ros::NodeHandle("~"))
    {
        base_link = pnh.param<std::string>("base_link", "soma_link");
        map_frame_id = pnh.param<std::string>("map_frame_id", "map");
        odom_id = pnh.param<std::string>("odom_id", "odom");

        //Subscriber
        clicked_point_sub = nh.subscribe<geo_msgs::PointStamped>("/clicked_point", 1,
                                                                 &Behavior::clicked_point_callback, this);
        command_sub = nh.subscribe<s_msgs::String>("/soma_command", 1,
                                                   &Behavior::command_callback, this);
        odom_sub = nh.subscribe(odom_id, 3,
                                &Behavior::odom_callback, this);

        //Publisher
        state_pub = nh.advertise<s_msgs::String>("/soma_state", 3);
        pg_pub = nh.advertise<geo_msgs::PointStamped>("/soma_pg", 3);
        xt_pub = nh.advertise<geo_msgs::PoseStamped>("/soma_xt", 3);

        // state = State::Stop;
        // command = State::Stop;

        ROS_INFO("Start timer callback");
        timer = nh.createTimer(ros::Duration((double)TIMER_T),
                               &Behavior::main,
                               this);
    }

    ~Behavior()
    {
    }

private:
    void main(const ros::TimerEvent &e)
    {
        ROS_INFO("State:%s", State::Str.at(data.state).c_str());

        //Publish
        std_msgs::String smsgs;
        smsgs.data = State::Str.at(data.state);
        state_pub.publish(smsgs);

        data._pg.header.stamp = ros::Time::now();
        pg_pub.publish(data._pg);

        data._xt.header.stamp = ros::Time::now();
        xt_pub.publish(data._xt);

        return;
    }

    void clicked_point_callback(const geometry_msgs::PointStampedConstPtr &msg)
    {
        if (msg->header.frame_id != map_frame_id)
        {
            ROS_WARN("Check global frame of /clicked_point");
            return;
        }

        ROS_INFO("Subscribe target point:%.1f,%.1f,%.1f",
                 msg->point.x, msg->point.y, msg->point.z);

        data._pg.header.frame_id = msg->header.frame_id;
        data._pg.header.stamp = ros::Time::now();
        data._pg.point = msg->point;

        return;
    }

    void command_callback(const std_msgs::StringConstPtr &msg)
    {
        ROS_INFO("Command:%s", msg->data.c_str());

        if (msg->data == "stop")
        {
            data.command = State::Stop;
        }
        else if (msg->data == "moveto")
        {
            data.command = State::Stop;
        }
        else if (msg->data == "home")
        {
            data.command = State::Home;
        }
        return;
    }

    void odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        data._xt.header.frame_id = msg->header.frame_id;
        data._xt.header.stamp = ros::Time::now();
        data._xt.pose = msg->pose.pose;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior");
    ROS_INFO("Start behavior node");
    Behavior behavior;
    ros::spin();
    return 0;
}