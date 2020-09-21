#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#define ROS_DIST 0 //0:kinetic. 1:melodic

int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_example");

#if ROS_DIST == 0
    tf::TransformListener *tfListener;
    tfListener = new tf::TransformListener(ros::Duration(100));
    costmap_2d::Costmap2DROS costmap("local_costmap_font", *tfListener);
    dwa_local_planner::DWAPlannerROS local_planner;
    local_planner.initialize("dwa_local_planner", tfListener, &costmap);
#elif ROS_DIST == 1
    tf2_ros::Buffer *tfBuffer = new tf2_ros::Buffer();
    tf2_ros::TransformListener *tfListener = new tf2_ros::TransformListener(*tfBuffer);
    costmap_2d::Costmap2DROS costmap("local_costmap_font", *tfBuffer);
    dwa_local_planner::DWAPlannerROS local_planner;
    local_planner.initialize("dwa_local_planner", tfBuffer, &costmap);
#endif

    ros::NodeHandle nh;
    costmap.start();
    ros::Rate rate(2); //Hz

    while (ros::ok())
    {
        geometry_msgs::PoseStamped fixed_start;
        geometry_msgs::PoseStamped out;
        fixed_start.header.frame_id = "foot_print";
        out.header.frame_id = "foot_print";

        fixed_start.pose.position.x = 0.0;
        fixed_start.pose.position.y = 0.0;
        fixed_start.pose.position.z = 0.0;
        fixed_start.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

        out.pose.position.x = 3.0;
        out.pose.position.y = 0.0;
        out.pose.position.z = 0.0;
        out.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

        std::vector<geometry_msgs::PoseStamped> local_path;
        local_path.push_back(fixed_start); // start pose
        local_path.push_back(out);         // goal pose
        local_planner.setPlan(local_path); // planning start to gall

        geometry_msgs::Twist _cmd_vel;
        local_planner.computeVelocityCommands(_cmd_vel);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
