#include "Home.h"

Home::Home(double _lim_d) : lim_d(_lim_d)
{
}

int Home::_Transition(Data_t *data)
{
  // force quit
  if (data->command == Command::Stop)
    return State::Stop;

  // stop?
  if (Dist(data->pg.point.x, data->x_t.position.x,
           data->pg.point.y, data->x_t.position.y) <= lim_d)
  {
    data->command = Command::Stop;
  }

  return State::GoHome;
}

int Home::_Enter(Data_t *data)
{
  // PpCfg.Pt = QPointF(0.0, 0.0);
  // PpCfg.Ps = QPointF(data->Xt.position.x, data->Xt.position.y);
  return 0;
}

int Home::_Process(Data_t *data)
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

int Home::_Exit(Data_t *data) { return 0; }
