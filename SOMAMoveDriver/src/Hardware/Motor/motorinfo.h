#ifndef MOTORINFO_H
#define MOTORINFO_H

#include <QString>
#include <QDebug>
#include <QtMath>

namespace MotorRPM
{
  const unsigned long Default = ULONG_MAX;
}

namespace MotorInfo {
  struct Data_t
  {
    //Input data structure
    struct In_t
    {
      double pos;
      unsigned long rpm;

      In_t() : pos(0.0), rpm(MotorRPM::Default) {}
    };

    //Output data structure
    struct Out_t
    {
      double pos, trgPos;	//[degree]
      int rpm;						//[roll/min]
      short cur;					//[mA]

      Out_t() : pos(0.0), trgPos(0.0), rpm(0), cur(0) {}
    };

    In_t In;
    Out_t Out;
    double min_pos, max_pos;
  };

  /*!
   * Definition of actuator names namespace
   */
  namespace Names	{
    const QString Steering = "STEERING";
    const QString RearBrake = "REAR_BRAKE";
    const QString FrontBrake = "FRONT_BRAKE";
    const QString Accel = "ACCEL";
    const QString Pan = "PAN";
    const QString Tilt = "TILT";
  }

  /*!
   * Definition of actuator roles namespace
   */
  namespace Roles {
    const QString Drive = "DRIVE";
    const QString Vision = "VISION";
    const QString Weeding = "WEEDING";
  }

  /*!
   * Device configuration structure
   */
  struct Config_t
  {
    QString DeviceName;
    QString ProtocolStackName;
    QString InterfaceName;
    QString PortName;
    unsigned int Node;

    unsigned int MaxRPM;
    unsigned int Accel;
    unsigned int Decel;
    int EncReso;
    double GearRatio;
    double MaxPos;
    double MinPos;

    friend QDebug operator<<(QDebug dbg, const Config_t &mcfg){
			dbg << mcfg.DeviceName
					<< mcfg.ProtocolStackName
					<< mcfg.InterfaceName
					<< mcfg.PortName
					<< mcfg.Node
          << mcfg.MaxRPM
          << mcfg.Accel
          << mcfg.Decel
          << mcfg.EncReso
          << mcfg.GearRatio
          << mcfg.MaxPos
          << mcfg.MinPos;
      return dbg;
    }
  };
}

namespace MotorPos
{
  const double Home = 0.0;
  const double Max = INFINITY;
  const double Min = -INFINITY;
}

#endif // MOTORINFO_H
