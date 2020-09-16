#ifndef DWA_H
#define DWA_H

#include <iostream>
#include <qt5/QtCore/QList>
#include <qt5/QtCore/QPointF>
#include <qt5/QtCore/QRectF>
#include <qt5/QtGui/QVector2D>
#include <string>
#include <vector>
#include <math.h>
using namespace std;

namespace DWA {

struct Config {
  /* paramaters for calculation dynamic window
    max_v, min_v, max_w : fow window Vs
    max_acc, max_dw : fow window Vr
    v_reso, w_reso : sampling resolution of control input from window Vs & Vd &
    Vr

    dt :  predict time slice [sec]
    T : predict time [sec]

    weight_v : weight for velocity cost
    weight_h : weight for heading cost
    weight_c : weight for clearance cost

    base : rectangule difinition local map range
  */

  float max_v, min_v;
  float max_w;
  float max_acc, max_dw;
  float v_reso, w_reso;
  int v_sample_num, w_sample_num;

  float dt, T;

  float weight_v;
  float weight_h;
  float weight_c;

  QRectF base;
};

struct U {
  float v;
  float omega;
};

struct X {
  float x, y;
  float theta;
};

struct DynamicWindow {
  QList<float> possibleV;
  QList<float> possibleW;
};

static void createDynamicWindow(U u, Config cfg, DynamicWindow *dw) {
  float Vs[4] = {cfg.min_v, cfg.max_v, -cfg.max_w, cfg.max_w};

  // Window Vd (definition by original paper)
  float Vd[4] = {
      u.v - cfg.max_acc * cfg.dt,
      u.v + cfg.max_acc * cfg.dt,
      u.omega - cfg.max_dw * cfg.dt,
      u.omega + cfg.max_dw * cfg.dt,
  };

  // Dynamic Window
  float DW[4] = {0.0};
  DW[0] = std::max<float>(Vs[0], Vd[0]);
  DW[1] = std::min<float>(Vs[1], Vd[1]);
  DW[2] = std::max<float>(Vs[2], Vd[2]);
  DW[3] = std::min<float>(Vs[3], Vd[3]);

  int nPossibleV = (int)((DW[1] - DW[0]) / cfg.v_reso);
  int nPossibleW = (int)((DW[3] - DW[2]) / cfg.w_reso);

  /*
   *   Sampling from Dynamic Window (Control Input Space)
   */
  for (int i = 0; i <= nPossibleV; i++) {
    float _v = DW[0] + (float)i * cfg.v_reso;
    dw->possibleV.push_back(_v);
  }

  for (int i = 0; i <= nPossibleW; i++) {
    float _w = DW[2] + (float)i * cfg.w_reso;
    dw->possibleW.push_back(_w);
  }
  return;
}

static X motion(X x, U u, float dt, float T) {
  X _x = x;
  return _x;
}

static float calcVelocityCost(U u, Config cfg) { return u.v; }

static float calcHeadingCost(X term_x, QPointF goal) {}

}  // namespace DWA

#endif