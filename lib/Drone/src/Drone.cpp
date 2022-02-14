#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "Drone.h"
#include <Functions.h>

//Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
// Adafruit_MotorShield motorShield = Adafruit_MotorShield();
// Adafruit_DCMotor *m1 = motorShield.getMotor(1);
// Adafruit_DCMotor *m2 = motorShield.getMotor(2);
// Adafruit_DCMotor *m3 = motorShield.getMotor(3);
// Adafruit_DCMotor *m4 = motorShield.getMotor(4);

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
  acceleration = Vector3<float>();
  angularVelocity = Vector3<float>();
  accelZeroOffset = Vector3<float>();
  gyroZeroOffset = Vector3<float>();
  m1 = motorShield.getMotor(1);
  m2 = motorShield.getMotor(2);
  m3 = motorShield.getMotor(3);
  m4 = motorShield.getMotor(4);
  Serial.println("Drone()");
}

Drone::~Drone()
{
    Serial.println("~Drone()");
}

Vector3<float> Drone::GetAcceleration()
{
  return acceleration;
}

Vector3<float> Drone::GetAngularVelocity()
{
  return angularVelocity;
}

void Drone::calibrate() 
{
  Vector3<float> accelSamples = Vector3<float>(0.0, 0.0, 0.0);
  Vector3<float> gyroSamples = Vector3<float>(0.0, 0.0, 0.0);
  Vector3<float> accelError;
  Vector3<float> gyroError;
  int nSamples = 1000;
  
  Serial.println("Calibrating...");
  mpu.getEvent(&a, &g, &temp);
  for (size_t i = 0; i < nSamples; i++) 
  {
    if (mpu.getEvent(&a, &g, &temp))
    { 
      accelSamples += a.acceleration.v;
      gyroSamples += g.gyro.v;
    }
    delay(2);
  }

  // Vector average calculated from sampled vector sum
  accelZeroOffset.x = accelSamples.x / nSamples;
  accelZeroOffset.y = accelSamples.y / nSamples;
  accelZeroOffset.z = accelSamples.z / nSamples;

  gyroZeroOffset.x = gyroSamples.x / nSamples;
  gyroZeroOffset.y = gyroSamples.y / nSamples;
  gyroZeroOffset.z = gyroSamples.z / nSamples;
  //error = Vector3<float>(abs(avg.x), abs(avg.y), abs(avg.z-9.81));
  // accelCal = Vector3<float>(a.acceleration.v) - zeroOffsetAccel;
  // gyroCal = Vector3<float>(g.gyro.v) - zeroOffsetGyro;

  print(accelZeroOffset, "Zero Offset Accel: ");
  print(gyroZeroOffset, "Zero Offset Gyro: ");
}

void Drone::Init()
{   
  while (!mpu.begin())
  {
    Serial.println("MPU not detected!...");;
    delay(1000);
  }
  Serial.println("MPU detected!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.getAccelerometerSensor()->printSensorDetails();
  mpu.getGyroSensor()->printSensorDetails();
  
  delay(1000);

  calibrate();

  motorShield.begin();
    
  delay(1000);
}

void Drone::Update() 
{
  if (mpu.getEvent(&a, &g, &temp)) {
    acceleration = Vector3<float>(a.acceleration.v) - accelZeroOffset;
    angularVelocity = Vector3<float>(g.gyro.v) - gyroZeroOffset;
    /*...
    */
  }
}