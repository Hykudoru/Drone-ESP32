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

// template <typename T>
// void Print(Vector3<T> vec, String header = "");
int clamp(int &val, int min, int max);
void print(Vector3<float> vec, char header[]);