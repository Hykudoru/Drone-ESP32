#include "Drone.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Vector3 Drone::GetPosition() {
    return position;
}

Vector3 Drone::GetRotation() {
    return rotation;
}

void Drone::Calibrate() {
    
}

void Drone::Setup() {
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

void Drone::Loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}