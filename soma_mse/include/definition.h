#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <map>

namespace geo_msgs = geometry_msgs;

namespace State
{
  const int Stop = 0;
  const int MoveTo = 1;
  const int Home = 2;

  std::map<int, std::string> Str = {
      {Stop, "Stop"}, {MoveTo, "MoveTo"}, {Home, "Home"}};
} // namespace State

struct Data_t
{
  int state;
  int command;
  geo_msgs::PointStamped _pg;
  geo_msgs::PoseStamped _xt;
  double _yaw_t;

  Data_t() : state(State::Stop),
             command(State::Stop){};
};