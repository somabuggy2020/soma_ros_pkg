#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <math.h>

#define TIMER_T 0.033 //[sec]

class Behavior
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;

    std::string base_link;
    std::string map_frame_id;

    //Subscriber
    ros::Subscriber clicked_point_sub;

    //Publisher
    ros::Publisher pg_pub;

    //local members
    geometry_msgs::PointStamped _pg;

public:
    Behavior() : nh(ros::NodeHandle()),
                 pnh(ros::NodeHandle("~"))
    {
        map_frame_id = pnh.param<std::string>("map_frame_id", "map");

        //Subscriber
        clicked_point_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1,
                                                                      &Behavior::clicked_point_callback, this);

        //Publisher
        pg_pub = nh.advertise<geometry_msgs::PointStamped>("/soma_pg", 3);

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

        //Publish
        pg_pub.publish(_pg);
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

        _pg.header.frame_id = msg->header.frame_id;
        _pg.header.stamp = ros::Time::now();
        _pg.point = msg->point;

        return;
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