#include "Arduino.h"
#include "WiFi.h"
#include "SPI.h"
#include "Wire.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

#if defined(ESP32)
  const int BAUD_RATE = 115200;
  #define BUTTON_A = 15;//GPIO 15 or A8
  #define BUTTON_B = 32;
  #define BUTTON_C = 14;
#else
  const int BAUD_RATE = 9600;
#endif

const int I2C_ADDR = 0x3C; // Can be shared with other I@C devices
int t = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial.println("");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Time elapsed: ");
  Serial.println(t++);
  delay(1000);
}
