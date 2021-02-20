#include "MoveTo.h"

MoveTo::MoveTo(double _lim_d) : lim_d(_lim_d)
{
  fixed_start.header.frame_id = "soma_link";
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
  fixed_start.header.stamp = ros::Time(0);
  fixed_target.header.stamp = ros::Time(0);

  //ローカルプランナのためのスタート地点を作成(ようするに原点)
  fixed_start.header.frame_id = "soma_link";
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
  geometry_msgs::TransformStamped t = data->tfBuf->lookupTransform("soma_link", "map", ros::Time(0));
  tf2::doTransform(fixed_target, out, t);
  // out.pose = data->tfBuf->transform(fixed_target.pose, "base_link");
  // data->tf->transformPose("base_link", fixed_target, out);

  std::vector<geometry_msgs::PoseStamped> local_path;
  local_path.push_back(fixed_start);        // start pose
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
