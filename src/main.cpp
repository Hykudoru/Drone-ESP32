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
pointerFunction ptrMode;
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

QWIICMUX mux;
JOYSTICK js;
const int LEFT_JOYSTICK = 0;
const int RIGHT_JOYSTICK = 1;

class Joystick
{
public:
  int offsetX = 0;
  int offsetY = 0;
  int x = 0;
  int y = 0;
  int button = 0;
  int muxPort;
  Joystick(int muxPort)
  {
    offsetX = 0;
    offsetY = 0;
    this->muxPort = muxPort;
  }
  ~Joystick() {}
  void Calibrate();
  void Update();
};

void Joystick::Update()
{
  static int rawMidpoint = 514;
  static int deadzoneOffset = 5;

  mux.enablePort(muxPort); Serial.println(mux.getPort());
  uint16_t rawX = js.getHorizontal();// 0 - 1023
  uint16_t rawY = js.getVertical();// 0 - 1023

  // Fix/reverse x axis
  if (rawX != 514)
  {
    rawX = 1023 - rawX;
  }

  // Map X-axis Range [-100, 100] 
  if (rawX < (rawMidpoint - deadzoneOffset))
  {
    //left
    x = -map(rawX, 0, 514, 100, 0);

  }
  else if (rawX > (rawMidpoint + deadzoneOffset))
  {
    //right
    x = map(rawX, 514, 1023, 0, 100);
  }
  else {
    x = 0;
  }

  // Map Y-axis Range [-100, 100] 
  if (rawY < (rawMidpoint - deadzoneOffset))
  {
    //left
    y = -map(rawY, 0, 514, 100, 0);

  }
  else if (rawY > (rawMidpoint + deadzoneOffset))
  {
    //right
    y = map(rawY, 514, 1023, 0, 100);
  }
  else {
    y = 0;
  }
  
  
  button = !js.getButton(); //Reversed so that button pressed = 1 else 0;

  Serial.println(String("Joystick ")+muxPort+" X: "+x+", Y: "+y+", Button: "+button);
  oled.println(String("JS ")+muxPort+" X:"+x+", Y:"+y+", Button:"+button);
  mux.disablePort(muxPort);

  //return Vector3(x, y, joystick.getButton());
}

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

void calibrateJoystick(Joystick *joystick) //calculate zero offset when centered
  {
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
    Serial.println(String("Joystick calibrated. Zero offset = ")+"X: "+(*joystick).offsetX+", Y: "+(*joystick).offsetY);//joysticks[muxPort].nameID);
  }



bool Send()
{
  return 0;
}

void mode_1() {
  //oled.clearDisplay();
  oled.setCursor(0, 0);
 // oled.println("Mode 1");

  //calibrate();

  oled.display();
}
void mode_2() {
  //oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 2");

  //calibrate();

  oled.display();
}
void mode_3() {
  //oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 3");

  //calibrate();

  oled.display();
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  js.begin();
  Wire.begin();
  mux.begin();

  leftJoystick.Update();//(LEFT_JOYSTICK); // force read potentially bad init readings
  rightJoystick.Update();// (RIGHT_JOYSTICK); // force read potentially bad init readings
  calibrateJoystick(&leftJoystick); // calculates joystick zero offset
  calibrateJoystick(&rightJoystick); // calculates joystick zero offset
 
   oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();//displays initial adafruit image
  oled.clearDisplay();//clears initial adafruit image
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Setup...");
  oled.display();
  
  // pinMode(BUTTON_A, INPUT_PULLUP);
  // pinMode(BUTTON_B, INPUT_PULLUP);
  // pinMode(BUTTON_C, INPUT_PULLUP);
  
  ptrMode = &mode_1;  
  
  delay(500);
  oled.clearDisplay();

  delay(100);
 }


void loop() 
{
  oled.clearDisplay();
//   if (digitalRead(BUTTON_A) == 0) pfunc = &mode_1;
//   if (digitalRead(BUTTON_B) == 0) pfunc = &mode_2;
//   if (digitalRead(BUTTON_C) == 0) pfunc = &mode_3;
  
 

  leftJoystick.Update();
  rightJoystick.Update();
  Send();

  (*ptrMode)();
  
  oled.display();
  delay(2);
}