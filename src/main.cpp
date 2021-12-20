#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiScan.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Drone.h>

#if defined(ESP32)
  const int BAUD_RATE = 115200;
#else
  const int BAUD_RATE = 9600;
#endif
const int I2C_ADDR = 0x3C; // Can be shared with other I2C devices
const int BUTTON_A = 15;//GPIO 15 or A8
const int BUTTON_B = 32;
const int BUTTON_C = 14;

int ACCEL_GYRO_ADDR = 0x68;

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction pfunc;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_MPU6050 mpu;
Drone drone;


void print(String str, boolean appendEmptyNewLine = false)
{
  Serial.print(str);
  display.print(str);

  if (appendEmptyNewLine) {
    Serial.println("");
    display.println("");
  }
}

void println(String str, boolean appendEmptyNewLine = false)
{
  Serial.println(str);
  display.println(str);

  if (appendEmptyNewLine) {
    Serial.println("");
    display.println("");
  }
}

void mode_1() {
  static int i = 0;
  if (i++ == 0) Serial.println("Mode 1");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 1");

  delay(100);
  yield();
  display.display();
}

void mode_2() {
  display.clearDisplay();
  display.setCursor(0, 0);
  println("Mode 2", true);


  delay(10);
  yield();
  display.display();
}

void mode_3() {
  display.clearDisplay();
  display.setCursor(0, 0);
  println("Mode 3", true);
  
  
  delay(10);
  yield();
  display.display();
}

void setup() 
{
  Serial.begin(BAUD_RATE);
  Serial.println("Setup...");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();//displays initial adafruit image
  display.clearDisplay();//clears initial adafruit image
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Setup...");
  display.display();

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  pfunc = &mode_1;  
  
  //mpuSetup();
  drone.setup();

  delay(500);
  display.clearDisplay();//clears initial adafruit image
}


void loop() 
{
  if (digitalRead(BUTTON_A) == 0) pfunc = &mode_1;
  if (digitalRead(BUTTON_B) == 0) pfunc = &mode_2;
  if (digitalRead(BUTTON_C) == 0) pfunc = &mode_3;
  
  //(*pfunc)();

  drone.loop();
  //mpuLoop();
}