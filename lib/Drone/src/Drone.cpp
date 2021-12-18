#include <Arduino.h>
#include <Adafruit_MPU6050.h>
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

void Drone::setup() 
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

void Drone::loop() 
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Serial.printf("Temperature: %sC ", temp);
  
  Serial.println("");

  Serial.println("Acceleration"); 
  Serial.print("x:"); Serial.println(a.acceleration.x);
  Serial.print("y:"); Serial.println(a.acceleration.y);
  Serial.print("z:"); Serial.println(a.acceleration.z);
  //Serial.printf("a.data: %d ", a.data);
  //Serial.printf("a.current: %d ", a.current);
  
  Serial.println("");

  Serial.println("Rotation"); 
  Serial.print("x:"); Serial.println(g.gyro.x);
  Serial.print("y:"); Serial.println(g.gyro.y);
  Serial.print("z:"); Serial.println(g.gyro.z);

  Serial.printf("g.gyro.v: %d ", g.gyro.v);
  Serial.printf("g.gyro.status: %d ", g.gyro.status);
  Serial.printf("g.gyro.heading: %d ", g.gyro.heading);
  Serial.printf("g.gyro.roll: %d ", g.gyro.roll);
  Serial.printf("g.gyro.pitch: %d ", g.gyro.pitch);
 
  delay(500);
}