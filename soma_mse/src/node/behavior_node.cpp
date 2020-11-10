#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>

#include "../definition.h"
#include "../StateBase/StateBase.h"
#include "../States/Stop.h"
#include "../States/MoveTo.h"
#include "../States/Home.h"

#define TIMER_T 0.033 //[sec]

namespace geo_msgs = geometry_msgs;

class Behavior
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;

  //frame ids
  std::string base_link;
  std::string map_frame_id;
  std::string odom_frame_id;

  //Subscriber
  ros::Subscriber clicked_point_sub;
  ros::Subscriber command_sub;
  ros::Subscriber odom_sub;

  //Publisher
  ros::Publisher state_pub;
  ros::Publisher pg_pub;
  ros::Publisher xt_pub;
  ros::Publisher ut_pub;

  //local members
  Data_t data;
  //state objects
  std::map<int, StateBase*> states;
  Stop *stop;
  MoveTo *moveto;
  Home *home;

public:
  Behavior() : nh(ros::NodeHandle()),
    pnh(ros::NodeHandle("~"))
  {

    //frame id strings
    base_link = pnh.param<std::string>("base_link", "soma_link");
    map_frame_id = pnh.param<std::string>("map_frame_id", "map");
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");

    //Subscriber
    clicked_point_sub = nh.subscribe<geo_msgs::PointStamped>("/clicked_point", 1,
                                                             &Behavior::clicked_point_callback, this);

    command_sub = nh.subscribe<std_msgs::String>("/soma_command", 1,
                                               &Behavior::command_callback, this);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 3,
                                                &Behavior::odom_callback, this);

    //Publisher
    state_pub = nh.advertise<std_msgs::String>("/soma_state", 3);
    pg_pub = nh.advertise<geo_msgs::PointStamped>("/soma_pg", 3);
    xt_pub = nh.advertise<geo_msgs::PoseStamped>("/soma_xt", 3);
    ut_pub = nh.advertise<geo_msgs::TwistStamped>("/soma_ut", 3);


    stop = new Stop();
    moveto = new MoveTo(3.0);
    home = new Home(3.0);
    states[State::Stop] = stop;
    states[State::MoveTo] = moveto;
    states[State::Home] = home;

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
    //FMS
    int new_state = states[data.state]->Transition(&data);

    if(new_state != data.state){
      states[data.state]->Exit(&data);
      states[new_state]->Enter(&data);
    }
    else{
      states[data.state]->Process(&data);
    }


    //log info
    ROS_INFO("State:%s / Command:%s",
             State::Str.at(data.state).c_str(),
             Command::Str.at(data.command).c_str());


    //Publish
    std_msgs::String smsgs;
    smsgs.data = State::Str.at(data.state);
    state_pub.publish(smsgs);

    geo_msgs::PointStamped pg_msgs;
    pg_msgs.header.frame_id = map_frame_id;
    pg_msgs.header.stamp = ros::Time::now();
    pg_msgs.point = data.pg;
    pg_pub.publish(pg_msgs);

    geo_msgs::PoseStamped xt_msgs;
    xt_msgs.header.frame_id = map_frame_id;
    xt_msgs.header.stamp = ros::Time::now();
    xt_msgs.pose = data.x_t;
    xt_pub.publish(xt_msgs);

    geo_msgs::TwistStamped ut_msgs;
    ut_msgs.header.frame_id = base_link;
    ut_msgs.header.stamp = ros::Time::now();
    ut_msgs.twist = data.u_t;
    ut_pub.publish(ut_msgs);

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

    data.pg = msg->point;
    return;
  }

  void command_callback(const std_msgs::StringConstPtr &msg)
  {
    ROS_INFO("Command:%s", msg->data.c_str());

    if (msg->data == "stop")
    {
      data.command = Command::Stop;
    }
    else if (msg->data == "moveto")
    {
      data.command = Command::MoveTo;
    }
    else if (msg->data == "home")
    {
      data.command = Command::GoHome;
    }
    return;
  }

  void odom_callback(const nav_msgs::OdometryConstPtr &msg)
  {
    data.x_t = msg->pose.pose;
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
