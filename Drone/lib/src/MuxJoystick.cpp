
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

Vector3<int> MuxJoystick::Read()
{
  static long t = 0;
  static int absErrorDeadZoneOffset = 15;

  // Check frame to determine if joystick values are still current; 
  if (t == millis())
  {
    Vector3<int> axis(x, y, (int)isPressed);
    return axis;
  }

  // New frame
  t = millis();

  mux.enablePort(muxPort);

  int rawX = rawJoystick.getHorizontal();// 0 - 1023
  int rawY = rawJoystick.getVertical();// 0 - 1023
  
  // Fix/reverse x axis
  if (rawX != 512)
  {
    rawX = 1023 - rawX;
  }

  // Map X-axis Range [-100, 100] 
  if (rawX < (512 - absErrorDeadZoneOffset))
  {
    // Left
    x = -map(rawX, 0, (512 - absErrorDeadZoneOffset), 100, 0);

  }
  else if (rawX > (512 + absErrorDeadZoneOffset))
  {
    // Right
    x = map(rawX, (512 + absErrorDeadZoneOffset), 1023, 0, 100);
  }
  else {
    x = 0;
  }

  // Map Y-axis Range [-100, 100] 
  if (rawY < (512 - absErrorDeadZoneOffset))
  {
    // Down
    y = -map(rawY, 0, (512 - absErrorDeadZoneOffset), 100, 0);

  }
  else if (rawY > (512 + absErrorDeadZoneOffset))
  {
    // Up
    y = map(rawY, (512 + absErrorDeadZoneOffset), 1023, 0, 100);
  }
  else {
    y = 0;
  }
  
  // Invert axes if physically upside down
  if (invertH) {
    x *= -1.0;
  }
  if (invertV)
  {
    y *= -1.0;
  }
  //Invert button so that pressed state means 1 = true else 0;
  isPressed = !rawJoystick.getButton();
  
  Serial.println(String("Joystick_")+muxPort
  +" <x:"+x+", y:"+y+">"+"  pressed:"+(int)(isPressed)
  +" \t raw: <x:"+rawX+", y:"+rawY+">  pressed:"+rawJoystick.getButton());
  //oled.println(String("JS (")+muxPort+") <"+x+","+y+"> Btn:"+buttonPressed);
  
  mux.disablePort(muxPort);

  Vector3<int> axis(x, y, (int)isPressed);
  return axis;
};