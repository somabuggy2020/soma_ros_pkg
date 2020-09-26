#include "soma_ros/Behavior/Home.h"

Home::Home(double _lim_d) : lim_d(_lim_d)
{
  // Pure pursuit config
  // PpCfg.Pt = QPointF(0.0, 0.0);
  // PpCfg.Ps = QPointF(0.0, 0.0);
  // PpCfg.v_const = 0.8;
  // PpCfg.dt = 0.5;
  // PpCfg.W = (float)WHEEL_BASE;
}

int Home::_Transition(Data *data)
{
  // force quit
  if (data->action == Action::Stop)
    return State::Stop;

  if (Dist(data->Pg.x, data->Xt.position.x,
           data->Pg.y, data->Xt.position.y) <= lim_d)
  {
    data->action = Action::Stop;
    return State::Stop;
  }

  return State::Home;
}

int Home::_Enter(Data *data)
{
  // PpCfg.Pt = QPointF(0.0, 0.0);
  // PpCfg.Ps = QPointF(data->Xt.position.x, data->Xt.position.y);
  return 0;
}

int Home::_Process(Data *data)
{
  // Pure pursuit algorithm
  // PurePursuit::X x;
  // x.x = data->Xt.position.x;
  // x.y = data->Xt.position.y;

  // tf::Quaternion q;
  // double roll, pitch, yaw;
  // tf::quaternionMsgToTF(data->Xt.orientation, q);
  // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // x.theta = yaw;

  // PurePursuit::U u;
  // u.v = data->Uin.v;
  // u.lambda = 0.0;

  // PurePursuit::U _u;

  // _u = PurePursuit::calc(x, u, PpCfg, true); // backward calculation

  // data->Uin.lambda = _u.lambda;
  // data->Uin.v = _u.v;

  // data->Xtarget = data->Xt.motion(data->Uin, PpCfg.dt);

  return 0;
}

int Home::_Exit(Data *data) { return 0; }
