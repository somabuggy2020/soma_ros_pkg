#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>

class Behavior
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;

public:
    Behavior()
    {
    }

    ~Behavior()
    {
    }
};

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior");
    Behavior behavior;
    ros::spin();
    return 0;
}