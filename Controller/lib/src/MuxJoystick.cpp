
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Vector.h>
#include <MuxJoystick.h>

QWIICMUX mux;
JOYSTICK rawJoystick;

void MuxJoystick::Start()
{
  //----- MUX joystick setup -------
  rawJoystick.begin();
  Wire.begin();
  mux.begin();
  //force read for potentially bad init readings
  Read();
}

Vector3<int> MuxJoystick::Read(int absMaxRadius)
{
  static int rawMidpoint = 512;
  static int absErrorDeadZoneOffset = 15;
  static long t = 0;

  // Check frame to determine if joystick values are still current; 
  if (t == millis())
  {
    return vec;
  }

  // New frame
  t = millis();

  mux.enablePort(muxPort);

  uint16_t rawX = rawJoystick.getHorizontal();// 0 - 1023
  uint16_t rawY = rawJoystick.getVertical();// 0 - 1023
  
  // Fix/reverse x axis
  if (rawX != rawMidpoint)
  {
    rawX = 1023 - rawX;
  }

  // Map X-axis Range [-100, 100] 
  if (rawX < (rawMidpoint - absErrorDeadZoneOffset))
  {
    // Left
    vec.x = -map(rawX, 0, (rawMidpoint - absErrorDeadZoneOffset), absMaxRadius, 0);

  }
  else if (rawX > (rawMidpoint + absErrorDeadZoneOffset))
  {
    // Right
    vec.x = map(rawX, (rawMidpoint + absErrorDeadZoneOffset), 1023, 0, absMaxRadius);
  }
  else {
    vec.x = 0;
  }

  // Map Y-axis Range [-100, 100] 
  if (rawY < (rawMidpoint - absErrorDeadZoneOffset))
  {
    // Down
    vec.y = -map(rawY, 0, (rawMidpoint - absErrorDeadZoneOffset), absMaxRadius, 0);

  }
  else if (rawY > (rawMidpoint + absErrorDeadZoneOffset))
  {
    // Up
    vec.y = map(rawY, (rawMidpoint + absErrorDeadZoneOffset), 1023, 0, absMaxRadius);
  }
  else {
    vec.y = 0;
  }
  
  // Invert axes if physically upside down
  if (invertH) {
    vec.x *= -1.0;
  }
  if (invertV)
  {
    vec.y *= -1.0;
  }
  //Invert button so that pressed state means 1 = true else 0;
  isPressed = !rawJoystick.getButton();
  vec.z = isPressed;

  Serial.println(String("Joystick_")+muxPort
  +" <x:"+vec.x+", y:"+vec.y+">"+"  pressed:"+(int)(isPressed)
  +" \t raw: <x:"+rawX+", y:"+rawY+">  pressed:"+rawJoystick.getButton());
  
  mux.disablePort(muxPort);

  return vec;
};