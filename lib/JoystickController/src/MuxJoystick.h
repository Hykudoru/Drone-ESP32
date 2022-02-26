#include <Vector.h>
#ifndef MUXJOYSTICK_H
#define MUXJOYSTICK_H

class MuxJoystick
{
public:
  int x = 0;// [-100, 100]
  int y = 0;// [-100, 100]
  int button = 0; // 1
  int muxPort;
  MuxJoystick(int muxPort)
  {
    this->muxPort = muxPort;
  }
  ~MuxJoystick() {}
  void Start();
  Vector3<int> Read();
};

class JoystickController
{
 public: 
  MuxJoystick *leftJoystick;
  MuxJoystick *rightJoystick;
  //char *command = "";
};
#endif