#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Vector.h>
#include <MuxJoystick.h>
#include <Mouse.h>
#include <Keyboard.h>

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

  leftJoystick.Start();
  rightJoystick.Start();

  Mouse.begin();
  Keyboard.begin();

  oled.display();

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  pinMode(LED_GREEN, OUTPUT);

  // ptrMode = &mode_1;  

  digitalWrite(LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(LED_GREEN, LOW);
 }

void CopyCommand()
{
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press('c');
  Keyboard.releaseAll();
}

void PasteCommand()
{
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press('v');
  Keyboard.releaseAll();
} 

void mouseLoop()
{
  static int maxSpeed = 40;
  static bool mouseActive = true;

  Vector3<int> mouse = leftJoystick.Read();
  Vector3<int> scroll = rightJoystick.Read();

  if (Serial.read() == 'p' || digitalRead(BUTTON_A) == HIGH)
  {
    mouseActive = !mouseActive;
  }
  
  if (mouseActive) 
  { 
    if (mouse.x != 0 || mouse.y != 0)
    {
      int dirX = mouse.x/abs(mouse.x);// -1 or 1
      int dirY = mouse.y/abs(mouse.y);// -1 or 1
      mouse.x = dirX * map(abs(mouse.x), 0, 100, 0, maxSpeed);
      mouse.y = dirY * map(abs(mouse.y), 0, 100, 0, maxSpeed);
      
      Mouse.move(mouse.x, -mouse.y, scroll.y);

      Serial.println("-----------------------");
      Serial.print(String("Mouse \t x:")+mouse.x+", y:"+ mouse.y);
      Serial.println(String("\t dirX:")+dirX+" dirY:"+dirY);
      Serial.println("-----------------------");
    }
    if (mouse.z)
    {
      CopyCommand();
    }
    if (scroll.z) {
      PasteCommand();
    }
      //Keyboard.write(KEY_RETURN);

    //delay(5);
  }
    // if (Keyboard.)
}

void Send()
{
}

void loop() 
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  // if (digitalRead(BUTTON_A) == 0) ptrMode = mode_1;
  // if (digitalRead(BUTTON_B) == 0) ptrMode = mode_2;
  // if (digitalRead(BUTTON_C) == 0) ptrMode = mode_3;
  
  mouseLoop();
  
  //(*ptrMode)();
  
  oled.display();
  //delay(2);
}