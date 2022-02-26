#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Vector.h>
#include <MuxJoystick.h>

#if defined(ESP32)
  const int BAUD_RATE = 115200;
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#endif
#if defined(__AVR_ATmega32U4__)
  const int BAUD_RATE = 9600;
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
#endif

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

const int LEFT_JOYSTICK_MUX_PORT = 1;
const int RIGHT_JOYSTICK_MUX_PORT = 0;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT);


// void calibrateJoystick(int muxPort)
// {
//     mux.enablePort(muxPort);
//     Serial.println(mux.getPort());
//     joystickZeroOffsetX = joystick.getHorizontal();
//     joystickZeroOffsetY = joystick.getVertical();
//     mux.disablePort(muxPort);
// }

// void calibrateJoystick(MuxJoystick *joystick) //calculate zero offset when centered
//   {
//     //calibrate
//     // Serial.println("(Joystick calibration starting in 4 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     // Serial.println("(Joystick calibration starting in 3 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     // Serial.println("(Joystick calibration starting in 2 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     // Serial.println("(Joystick calibration starting in 1 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     Serial.println("Calibrating...");
//   }


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();//displays initial adafruit image
  oled.clearDisplay();//clears initial adafruit image
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Setup...");
  oled.display();
  delay(500);
  oled.clearDisplay();

  leftJoystick.Start();
  rightJoystick.Start();

  oled.display();

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  // ptrMode = &mode_1;  

  delay(1000);
 }

bool Send()
{
  
  return 0;
}

//char *mode = "mode_1";

void loop() 
{
  oled.clearDisplay();
  // if (digitalRead(BUTTON_A) == 0) mode = "mode_1";
  // if (digitalRead(BUTTON_B) == 0) mode = "mode_2";
  // if (digitalRead(BUTTON_C) == 0) mode = "mode_3";
  
  Vector3<int> l = leftJoystick.Read();
  Vector3<int> r = rightJoystick.Read();
  Send();

  //(*ptrMode)();
  
  oled.display();
  delay(20);
}