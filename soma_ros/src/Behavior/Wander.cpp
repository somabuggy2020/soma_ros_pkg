#include "soma_ros/Behavior/Wander.h"

Wander::Wander()
{
  tf = new tf::TransformListener();
  local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *tf);

  //--- base local planner
  // local_planner = new base_local_planner::TrajectoryPlannerROS(
  //     "local_planner", tf, local_costmap);

  //--- dwa local planner
  local_planner = new dwa_local_planner::DWAPlannerROS();
  local_planner->initialize("dwa_local_planner", tf, local_costmap);

  // local_planner = new teb_local_planner::TebLocalPlannerROS();

  fixed_start.header.frame_id = "foot_print";
  fixed_start.header.stamp = ros::Time::now();
  fixed_start.pose.position.x = 0.0;
  fixed_start.pose.position.y = 0.0;
  fixed_start.pose.position.z = 0.0;
  tf::Quaternion tmp = tf::createQuaternionFromYaw(0.0);
  tf::quaternionTFToMsg(tmp, fixed_start.pose.orientation);

  dummy_target.header.frame_id = "foot_print";
  dummy_target.pose.position.x = 5.0;
  dummy_target.pose.position.y = 0.0;
  dummy_target.pose.position.z = 0.0;
  tmp = tf::createQuaternionFromYaw(0.0);
  tf::quaternionTFToMsg(tmp, dummy_target.pose.orientation);

  std::vector<geometry_msgs::PoseStamped> local_path;
  local_path.push_back(fixed_start);  //start pose
  local_path.push_back(dummy_target); //goal pose
  local_planner->setPlan(local_path); //pose set
}

Wander::~Wander()
{
}

int Wander::_Transition(Data *data)
{
  ROS_INFO("Np:%d", data->np);

  if (data->action == Action::Stop)
    return State::Stop;

  return State::Wander;
}

int Wander::_Enter(Data *data) { return 0; }

int Wander::_Process(Data *data)
{
  fixed_start.header.stamp = ros::Time::now();
  dummy_target.header.stamp = ros::Time::now();

  geometry_msgs::Twist _cmd_vel;
  local_planner->computeVelocityCommands(_cmd_vel);
  ROS_INFO("%f, %.2f", _cmd_vel.linear.x, _cmd_vel.angular.z);

  data->Uin.from(_cmd_vel.linear.x, _cmd_vel.angular.z);
  data->Uin.v = _cmd_vel.linear.x;

  return 0;
}

int Wander::_Exit(Data *data) {}