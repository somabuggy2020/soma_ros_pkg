#ifndef PP_H
#define PP_H

#include <math.h>
#include <QtCore/QPointF>
#include <QtCore/QVector>
#include <QtGui/QVector2D>

namespace PurePursuit
{
    struct Config
    {
        QPointF Pt;    //target point on global coordinate
        QPointF Ps;    //start point when starting pursuit
        float v_const; //contant linear velocity
        float dt;      //predict time [sec]
        float W;       //wheel base [m]
    };

    struct U
    {
        float v;
        float lambda;
    };

    struct X
    {
        float x, y;
        float theta;
    };

    static U calc(X x, U u, Config cfg, bool backward)
    {
        //predict future state vector
        X _x;
        _x.theta = x.theta;

	if(backward){
	 _x.x = x.x + (-cfg.v_const)*cfg.dt*cos(x.theta);
	 _x.y = x.y + (-cfg.v_const)*cfg.dt*sin(x.theta);
	}
	else{
        _x.x = x.x + cfg.v_const * cfg.dt * cos(x.theta);
        _x.y = x.y + cfg.v_const * cfg.dt * sin(x.theta);
	}

        //vectorl calculation
	QVector2D ap(QPointF(_x.x, _x.y) - cfg.Ps);
	QVector2D ab(cfg.Pt - cfg.Ps);
//        QVector2D va(cfg.Pt - cfg.Ps);
 //       QVector2D vb(QPointF(_x.x, _x.y) - cfg.Ps);

        float prob = QVector2D::dotProduct(ap, ab);
        QVector2D vn = prob * ab.normalized();

        QPointF Palpha;
	if(vn.x() < cfg.Pt.x() || cfg.Ps.x() < vn.x()){
		Palpha.setX(cfg.Pt.x());
		Palpha.setY(cfg.Pt.y());
	}
	else{
	        Palpha.setX(cfg.Ps.x() + vn.x());
       		Palpha.setY(cfg.Ps.y() + vn.y());
	}

        //calc desired control input
        float alpha = atan2((Palpha.y() - x.y), (Palpha.x() - x.x)) - x.theta;
        QPointF tmp = Palpha - QPointF(x.x, x.y);
        float L = sqrt(pow(tmp.x(), 2) + pow(tmp.y(), 2));

        U _u;
        if (!backward)
            _u.v = cfg.v_const;
        else if (backward)
            _u.v = -cfg.v_const;

        if (L > 0.00001)
            _u.lambda = atan(2.0 * cfg.W * sin(alpha) / L);
        else
            _u.lambda = 0.0;

        return _u;
    }

} // namespace PurePursuit

#endif
