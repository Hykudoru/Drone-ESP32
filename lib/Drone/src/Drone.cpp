#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "Drone.h"

// Vector3 Drone::GetPosition() 
// {
//     return position;
// }

// Vector3 Drone::GetRotation() 
// {
//     return rotation;
// }
Drone::Drone(/* args */)
{
    Serial.println("Drone()");
}

Drone::~Drone()
{
    Serial.println("~Drone()");
}

void Drone::calibrate() 
{
    Serial.println("Calibrate()");
}

void Drone::init() 
{
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

void Drone::update() 
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}