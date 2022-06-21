#ifndef DATA_H
#define DATA_H
#include <Vector.h>

typedef class SendReceiveData
{
  public:
  int ID;
};

typedef class DroneData: public SendReceiveData
{
  public:
  Vector3<float> Acceleration;
  Vector3<float> AngularVelocity;
};

typedef class JoystickControllerData: public SendReceiveData
{
  public:
  Vector3<float> LeftJoystick;
  Vector3<float> RightJoystick;
  uint16_t Potentiomter; 
  uint16_t Potentiomter2; 
};
#endif