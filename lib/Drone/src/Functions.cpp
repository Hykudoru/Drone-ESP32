
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>
#include <Drone.h>
#include <Vector.h>
#include "Functions.h"


int clamp(int &val, int min, int max) {
  if (val > max) 
  {
    val = max;
  }
  else if (val < min) {
    val = min;
  }
  
  return val;
}

void print(Vector3<float> vec, char header[])
{
    Serial.println("");

    Serial.print(header); 
    Serial.print(vec.x); Serial.print(", ");
    Serial.print(vec.y); Serial.print(", ");
    Serial.println(vec.z); 
}