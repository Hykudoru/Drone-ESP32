#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>
#include <esp_now.h>
#include <WiFi.h>
//Alex Lib
#include <Functions.h>
#include <Matrix.h>
#include <WirelessData.h>
#include "../lib/src/Drone.h"//#include <Drone.h>



void loop() 
{
  Matrix3x3 rotMatrix;
  rotMatrix = Multiply(Multiply(RotZ(45*PI/180).matrix, RotY(45*PI/180).matrix).matrix, RotX(45*PI/180).matrix);
}