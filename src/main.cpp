#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Vector.h>
#include <MuxJoystick.h>
#include <BleKeyboard.h>
#include <BleMouse.h>

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
  #define BUTTON_D 12C 
  #define BUTTON_MOUSE_LEFT 0xA0
  #define BUTTON_MOUSE_RIGHT 0xA1
  #define BUTTON_SWAP_JOYSTICKS 0xA2
  
  #define ANALOG_PIN_3 0xA3
  #define ANALOG_PIN_4 0xA4
  #define ANALOG_PIN_5 0xA5

  #define LED_GREEN 8
#endif

BleKeyboard bleKeyboard;
//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

const int LEFT_JOYSTICK_MUX_PORT = 1;
const int RIGHT_JOYSTICK_MUX_PORT = 0;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT);

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
pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);


  leftJoystick.Start();
  rightJoystick.Start();
  bleKeyboard.begin();

  oled.display();
 }

int maxSpeed = 40;
bool mouseActive = true;
Vector3<int> mouse;
Vector3<int> scroll;

void loop() 
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  
if (mouseActive) 
{ 
  mouse = leftJoystick.Read();
  scroll = rightJoystick.Read();
  
  if (mouse.x != 0 || mouse.y != 0)
  {
    int dirX = mouse.x/abs(mouse.x);// -1 or 1
    int dirY = mouse.y/abs(mouse.y);// -1 or 1
    mouse.x = dirX * map(abs(mouse.x), 0, 100, 0, maxSpeed);
    mouse.y = dirY * map(abs(mouse.y), 0, 100, 0, maxSpeed);

    if (bleKeyboard.isConnected())
    {
      
    }
  }
}
  
  oled.display();
  //delay(2);
}