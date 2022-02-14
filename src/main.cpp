#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>


#if defined(ESP32)
  const int BAUD_RATE = 115200;
#else
  const int BAUD_RATE = 9600;
#endif

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction pfunc;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

QWIICMUX mux;
JOYSTICK js;
const int LEFT_JOYSTICK = 0;
const int RIGHT_JOYSTICK = 1;

class Joystick
{
public:
  int offsetX;
  int offsetY;
  
  int x;
  int y;
  int muxPort;
  Joystick(int muxPort)
  {
    offsetX = 0;
    offsetY = 0;
    muxPort = muxPort;
  }
  ~Joystick() {}

  void Calibrate()
  {
    calibrateJoystick(this);
  }
  void Update()
  {
    readJoystick(this);
  }
};

Joystick leftJoystick(LEFT_JOYSTICK);
Joystick rightJoystick(RIGHT_JOYSTICK);

// void calibrateJoystick(int muxPort)
// {
//     mux.enablePort(muxPort);
//     Serial.println(mux.getPort());
//     joystickZeroOffsetX = joystick.getHorizontal();
//     joystickZeroOffsetY = joystick.getVertical();
//     mux.disablePort(muxPort);
// }
void readJoystick(Joystick *joystick) 
{
  int muxPort = (*joystick).muxPort;
  mux.enablePort(muxPort); Serial.println(mux.getPort());

  uint16_t x = js.getHorizontal();//
  uint16_t y = js.getVertical();//

  // fix dir of x axis
  if (x != 514)
  {
    x = 1023 - x;
  }

  (*joystick).x = x;
  (*joystick).y = y;

  Serial.println(String("Joystick ")+muxPort+" X: "+x);
  Serial.println(String("Joystick ")+muxPort+" Y: "+y);
  Serial.println(String("Joystick ")+muxPort+" Button: "+js.getButton());

  mux.disablePort(muxPort);

  //return Vector3(x, y, joystick.getButton());
}
void calibrateJoystick(Joystick *joystick) //calculate zero offset when centered
  {
    readJoystick(&leftJoystick);//(LEFT_JOYSTICK); // force read potentially bad init readings
    readJoystick(&rightJoystick);// (RIGHT_JOYSTICK); // force read potentially bad init readings

    //calibrate
    mux.enablePort((*joystick).muxPort);
    Serial.println(mux.getPort());
    Serial.println("(Joystick calibration starting in 4 seconds...)");//joysticks[muxPort].nameID);
    delay(1000);
    Serial.println("(Joystick calibration starting in 3 seconds...)");//joysticks[muxPort].nameID);
    delay(1000);
    Serial.println("(Joystick calibration starting in 2 seconds...)");//joysticks[muxPort].nameID);
    delay(1000);
    Serial.println("(Joystick calibration starting in 1 seconds...)");//joysticks[muxPort].nameID);
    delay(1000);
    Serial.println("Calibrating Joystick...");//joysticks[muxPort].nameID);

    int nSamples = 10;
    uint16_t x = 0;
    uint16_t y = 0;
    int i = 0;
    while(i++ < nSamples)
    {
      x += js.getHorizontal();//was .xZeroOffset +=
      y += js.getVertical();// was .yZeroOffset +=
      delay(10);
    }
    // x axis
    (*joystick).offsetX = (x/nSamples);//avg
    // y axis
    (*joystick).offsetY = (y/nSamples);//avg

    mux.disablePort((*joystick).muxPort);
    Serial.println(String("Joystick calibrated. Zero offset = ")+"X:"+(*joystick).offsetX+", Y:"+(*joystick).offsetY);//joysticks[muxPort].nameID);
  }



bool Send()
{
  return 0;
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  js.begin();
  Wire.begin();
  mux.begin();

  calibrateJoystick(&leftJoystick); // calculates joystick zero offset
  calibrateJoystick(&rightJoystick); // calculates joystick zero offset
 
  delay(100);
 }


void loop() 
{
  // if (digitalRead(BUTTON_A) == 0) pfunc = &mode_1;
  // if (digitalRead(BUTTON_B) == 0) pfunc = &mode_2;
  // if (digitalRead(BUTTON_C) == 0) pfunc = &mode_3;
  
  //(*pfunc)();

  leftJoystick.Update();
  rightJoystick.Update();

  Send();

  delay(200);
}