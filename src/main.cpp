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

const int I2C_ADDR = 0x3C; // Can be shared with other I2C devices
const int BUTTON_A = 15;//GPIO 15 or A8
const int BUTTON_B = 32;
const int BUTTON_C = 14;
int ACCEL_GYRO_ADDR = 0x68;

#if defined(ESP32)
  const int BAUD_RATE = 115200;
#else
  const int BAUD_RATE = 9600;
#endif

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction pfunc;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_MPU6050 mpu;

void setupMPU(Adafruit_MPU6050 mpu) {
  if (!mpu.begin()) {
    Serial.println("MPU not detected!");
    while (1)
    {
      yield();
    }
  }
  else {
    Serial.println("MPU detected!");
  }
}

void loopMPU(Adafruit_MPU6050 mpu) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}

void mode_1() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.write("Mode 1");


  delay(100);
  yield();
  display.display();
}

void mode_2() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.write("Mode 1");


  delay(10);
  yield();
  display.display();
}

void mode_3() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.write("Mode 1");


  delay(10);
  yield();
  display.display();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial.println("");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Setup...");
  display.display();
  delay(500);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  pfunc = &mode_1;  
  setupMPU(mpu);
}


void loop() {

  if (digitalRead(BUTTON_A) == 0) pfunc = &mode_1;
  if (digitalRead(BUTTON_B) == 0) pfunc = &mode_2;
  if (digitalRead(BUTTON_C) == 0) pfunc = &mode_3;
  
  (*pfunc)();

  loopMPU(mpu);
}