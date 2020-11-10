#include "soma_ros/Behavior/MoveTo.h"

MoveTo::MoveTo(double _lim_d) : lim_d(_lim_d)
{
  fixed_start.header.frame_id = "foot_print";
  dummy_target.header.frame_id = "foot_print";

  // Pure pursuit config
  // PpCfg.Pt = QPointF(0.0, 0.0);
  // PpCfg.Ps = QPointF(0.0, 0.0);
  // PpCfg.v_const = 0.8;
  // PpCfg.dt = 0.5;
  // PpCfg.W = (float)WHEEL_BASE;
}

MoveTo::~MoveTo() {}

int MoveTo::_Transition(Data *data)
{
  // force quit
  if (data->action == Action::Stop)
    return State::Stop;

  // reach global target position?
  if (Dist(data->Pg.x, data->Xt.position.x,
           data->Pg.y, data->Xt.position.y) <= lim_d)
  {
    data->action = Action::Stop;
    return State::Stop;
  }

  // Continue?
  // if (data->action == Action::MoveTo)
  //   return State::MoveTo;

  // if (data->action == Action::Home)
  //   return State::Home;

  return State::MoveTo; //continue
}

int MoveTo::_Enter(Data *data)
{
  // PpCfg.Pt = QPointF(data->Pg.x(), data->Pg.y());
  // PpCfg.Ps = QPointF(data->Xt.x, data->Xt.y);
  return 0;
}

int MoveTo::_Process(Data *data)
{
  // Pure pursuit algorithm
  // PurePursuit::X x;
  // x.x = data->Xt.x;
  // x.y = data->Xt.y;
  // x.theta = data->Xt.theta;

  // PurePursuit::U u;
  // u.v = data->vt;
  // u.lambda = 0.0;

  // PurePursuit::U _u;

  // //
  // if (data->state == State::MoveTo) {
  //   _u = PurePursuit::calc(x, u, PpCfg, false);  // forward calculation
  // } else if (data->state == State::Home) {
  //   _u = PurePursuit::calc(x, u, PpCfg, true);  // backward calculation
  // }

  // data->Uin.lambda = _u.lambda;
  // data->Uin.v = _u.v;

  fixed_start.header.stamp = ros::Time(0);
  dummy_target.header.stamp = ros::Time(0);

  fixed_start.header.frame_id = "foot_print";
  fixed_start.pose.position.x = 0.0;
  fixed_start.pose.position.y = 0.0;
  fixed_start.pose.position.z = 0.0;
  fixed_start.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  dummy_target.header.frame_id = "map";
  dummy_target.pose.position = data->Pg;
  dummy_target.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  geometry_msgs::PoseStamped out;
  data->tf->transformPose("foot_print", dummy_target, out);

  std::vector<geometry_msgs::PoseStamped> local_path;
  local_path.push_back(fixed_start);        // start pose
  local_path.push_back(out);                // goal pose
  data->local_planner->setPlan(local_path); // planning start to gall

  geometry_msgs::Twist _cmd_vel;
  data->local_planner->computeVelocityCommands(_cmd_vel);

  data->Uin.from(_cmd_vel.linear.x, _cmd_vel.angular.z);
  data->Uin.v = _cmd_vel.linear.x;

  // data->Xtarget = data->Xt.motion(data->Uin, 1.0);
  return 0;
}

int MoveTo::_Exit(Data *data)
{
  return 0;
}
