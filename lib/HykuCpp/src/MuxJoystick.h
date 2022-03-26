#include <Vector.h>
#ifndef MUXJOYSTICK_H
#define MUXJOYSTICK_H

class MuxJoystick
{
  int x = 0;// [-100, 100]
  int y = 0;// [-100, 100]
  int buttonPressed = 0; // 1
public:  
  int muxPort;
  MuxJoystick(int muxPort)
  {
    this->muxPort = muxPort;
  }
  ~MuxJoystick() {}
  void Start();
  Vector3<int> Read();
};
#endif