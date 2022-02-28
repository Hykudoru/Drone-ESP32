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

#define LSTICK_LEFT_LED 10
#define LSTICK_RIGHT_LED 11
#define LSTICK_DOWN_LED 12
#define LSTICK_UP_LED 13

#define RSTICK_LEFT_LED 9//10
#define RSTICK_RIGHT_LED 9//11
#define RSTICK_DOWN_LED 9//12
#define RSTICK_UP_LED 9//13

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

  pinMode(LSTICK_LEFT_LED, OUTPUT);
  pinMode(LSTICK_RIGHT_LED, OUTPUT);
  pinMode(LSTICK_DOWN_LED, OUTPUT);
  pinMode(LSTICK_UP_LED, OUTPUT);
  pinMode(RSTICK_LEFT_LED, OUTPUT);
  pinMode(RSTICK_RIGHT_LED, OUTPUT);
  pinMode(RSTICK_DOWN_LED, OUTPUT);
  pinMode(RSTICK_UP_LED, OUTPUT);
  // ptrMode = &mode_1;  

  digitalWrite(LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(LED_GREEN, LOW);
 }

    
void mouseLoop()
{
  static MuxJoystick *mouse = &leftJoystick;
  static MuxJoystick *scroll = &rightJoystick;
  static int maxSpeed = 40;
  static bool mouseActive = true;
    
  if (Serial.read() == 'p' || digitalRead(BUTTON_A) == HIGH)
  {
    mouseActive = !mouseActive;
  }

  if (mouseActive) 
  { 
    if (mouse->x != 0 || mouse->y != 0)
    {
      int dirX = mouse->x/abs(mouse->x);// -1 or 1
      int dirY = mouse->y/abs(mouse->y);// -1 or 1
      int mouseX = dirX * map(abs(mouse->x), 0, 100, 0, maxSpeed);
      int mouseY = dirY * map(abs(mouse->y), 0, 100, 0, maxSpeed);
      
      Mouse.move(mouseX, -mouseY, scroll->y);

      Serial.println("-----------------------");
      Serial.print(String("Mouse \t x:")+mouse->x+", y:"+ mouse->y);
      Serial.println(String("\t dirX:")+dirX+" dirY:"+dirY);
      Serial.println("-----------------------");
      
      
    }
    if (mouse->button)
    {
      Keyboard.press(KEY_LEFT_CTRL);
      Keyboard.press('c');
      Keyboard.releaseAll();
    }
    if (scroll->button) {
      Keyboard.press(KEY_LEFT_CTRL);
      Keyboard.press('v');
      Keyboard.releaseAll();
    }
      //Keyboard.write(KEY_RETURN);

    delay(5);
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
  
  leftJoystick.Read();
  rightJoystick.Read();
  mouseLoop();
  // LEFT JOYSTICK LEDS

  // // LEFT
  // if (leftJoystick.x < 0) {
  //   analogWrite(LSTICK_LEFT_LED, map(abs(leftJoystick.x), 0, 100, 0, 255));
  //   digitalWrite(LSTICK_RIGHT_LED, LOW);
  // } // RIGHT
  // else if(leftJoystick.x > 0) {
  //   analogWrite(LSTICK_RIGHT_LED, map(abs(leftJoystick.x), 0, 100, 0, 255));
  //   digitalWrite(LSTICK_LEFT_LED, LOW);
  // } // ZERO
  // else {
  //    digitalWrite(LSTICK_LEFT_LED, LOW);
  //   digitalWrite(LSTICK_RIGHT_LED, LOW);
  // }
  // // DOWN
  // if (leftJoystick.y < 0) {
  //   analogWrite(LSTICK_DOWN_LED, map(abs(leftJoystick.y), 0, 100, 0, 255));
  //   digitalWrite(LSTICK_UP_LED, LOW);
  // }// UP
  // else if(leftJoystick.y > 0) {
  //   analogWrite(LSTICK_UP_LED, map(abs(leftJoystick.y), 0, 100, 0, 255));
  //   digitalWrite(LSTICK_DOWN_LED, LOW);
  // } // ZERO
  // else {
  //   digitalWrite(LSTICK_DOWN_LED, LOW);
  //   digitalWrite(LSTICK_UP_LED, LOW);
  // }

  // // RIGHT JOYSTICK LEDS
  
  // // LEFT
  // if (rightJoystick.x < 0) {
  //   analogWrite(RSTICK_LEFT_LED, map(abs(rightJoystick.x), 0, 100, 0, 255));
  //   digitalWrite(RSTICK_RIGHT_LED, LOW);
  // } // RIGHT
  // else if(rightJoystick.x > 0) {
  //   analogWrite(RSTICK_RIGHT_LED, map(abs(rightJoystick.x), 0, 100, 0, 255));
  //   digitalWrite(RSTICK_LEFT_LED, LOW);
  // }// ZERO
  // else {
  //   digitalWrite(RSTICK_LEFT_LED, LOW);
  //   digitalWrite(RSTICK_RIGHT_LED, LOW);
  // } 

  //   // DOWN
  // if (rightJoystick.y < 0) {
  //   analogWrite(RSTICK_DOWN_LED, map(abs(rightJoystick.y), 0, 100, 0, 255));
  //   digitalWrite(RSTICK_UP_LED, LOW);
  // }// UP
  // else if(rightJoystick.y > 0) {
  //   analogWrite(RSTICK_UP_LED, map(abs(rightJoystick.y), 0, 100, 0, 255));
  //   digitalWrite(RSTICK_DOWN_LED, LOW);
  // } // ZERO
  // else {
  //   digitalWrite(RSTICK_DOWN_LED, LOW);
  //   digitalWrite(RSTICK_UP_LED, LOW);
  // }
  
  Send();

  
  //(*ptrMode)();
  
  oled.display();
  //delay(2);
}