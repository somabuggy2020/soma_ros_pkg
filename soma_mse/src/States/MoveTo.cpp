#include "MoveTo.h"

MoveTo::MoveTo(double _lim_d) : lim_d(_lim_d)
{
  fixed_start.header.frame_id = "foot_print";
  fixed_target.header.frame_id = "foot_print";
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
  fixed_start.header.stamp = ros::Time(0);
  fixed_target.header.stamp = ros::Time(0);

  //ローカルプランナのためのスタート地点を作成(ようするに原点)
  fixed_start.header.frame_id = "foot_print";
  fixed_start.pose.position.x = 0.0;
  fixed_start.pose.position.y = 0.0;
  fixed_start.pose.position.z = 0.0;
  fixed_start.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  //ローカルプランナのためのゴール地点を作成
  //まずは大域座標の目標地点を取得
  fixed_target.header.frame_id = data->pg.header.frame_id;
  fixed_target.pose.position = data->pg.point;
  fixed_target.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  geometry_msgs::PoseStamped out;
  //  data->tf->transformPose("foot_print", dummy_target, out);

  std::vector<geometry_msgs::PoseStamped> local_path;
  local_path.push_back(fixed_start); // start pose
  local_path.push_back(out);         // goal pose
  //  data->local_planner->setPlan(local_path); // planning start to gall

  geometry_msgs::Twist _cmd_vel;
  //  data->local_planner->computeVelocityCommands(_cmd_vel);

  //  data->Uin.from(_cmd_vel.linear.x, _cmd_vel.angular.z);
  //  data->Uin.v = _cmd_vel.linear.x;
  // data->Xtarget = data->Xt.motion(data->Uin, 1.0);
  return 0;
}

int MoveTo::_Exit(Data_t *data)
{
  return 0;
}
