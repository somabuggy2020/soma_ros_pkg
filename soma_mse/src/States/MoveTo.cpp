#include "MoveTo.h"

MoveTo::MoveTo(double _lim_d) : lim_d(_lim_d)
{
  fixed_target.header.frame_id = "soma_link";
}

MoveTo::~MoveTo() {}

int MoveTo::_Transition(Data_t *data)
{
  // force quit
  if (data->command == Command::Stop)
    return State::Stop;

  // reach global target position?
  if (Dist(data->pg.point.x, data->x_t.position.x,
           data->pg.point.y, data->x_t.position.y) <= lim_d)
  {
    data->command = Command::Stop;
  }

  return State::MoveTo; //continue
}

int MoveTo::_Enter(Data_t *data)
{
  return 0;
}

int MoveTo::_Process(Data_t *data)
{
  data->fixed_start.header.stamp = ros::Time::now();
  data->fixed_target.header.stamp = ros::Time::now();

  //get target position in "map" frame
  fixed_target.pose.position = data->pg.point;

  //transform target point "map" to "base_link" frame
  geometry_msgs::PoseStamped out;
  tf2::doTransform(fixed_target, out, data->transform_map2base);

  //calc local plan
  std::vector<geometry_msgs::PoseStamped> local_path;
  local_path.push_back(data->fixed_start);  // start pose
  local_path.push_back(out);                // target pose
  data->local_planner->setPlan(local_path); // planning start to gall

  geometry_msgs::Twist _cmd_vel;
  data->local_planner->computeVelocityCommands(_cmd_vel);
  data->u_t = _cmd_vel;
  return 0;
}

int MoveTo::_Exit(Data_t *data)
{
  return 0;
}
