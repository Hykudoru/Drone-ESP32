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


Drone::Drone(/* args */)
{
  position = Vector3<float>();
  rotation = Vector3<float>();
  velocity = Vector3<float>();
  angularVelocity = Vector3<float>();

  prevPosition = Vector3<float>();
  prevRotation = Vector3<float>();
  prevVelocity = Vector3<float>();
  prevAngularVelocity = Vector3<float>();

  accelZeroOffset = Vector3<float>();
  gyroZeroOffset = Vector3<float>();

  motorShield = Adafruit_MotorShield();
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
    delay(4);// 250Hz sample rate. 1000ms/250 = 4ms
  }

  // Vector average calculated from sampled vector sum
  accelZeroOffset.x = accelSamples.x / (float) nSamples;
  accelZeroOffset.y = accelSamples.y / (float) nSamples;
  accelZeroOffset.z = accelSamples.z / (float) nSamples;

  gyroZeroOffset.x = gyroSamples.x / (float) nSamples;
  gyroZeroOffset.y = gyroSamples.y / (float) nSamples;
  gyroZeroOffset.z = gyroSamples.z / (float) nSamples;
  //error = Vector3<float>(abs(avg.x), abs(avg.y), abs(avg.z-9.81));

  print(accelZeroOffset, "Zero Offset Accel: ");
  print(gyroZeroOffset, "Zero Offset Gyro: ");
}

void Drone::Init()
{
  if (!mpu.begin())
  {
    Serial.println("MPU not detected!...");
    delay(1000);
  }
  else {
    Serial.println("MPU detected!");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);// Sensitivity Scale Factor: raw 2,048 = 1g = (2^15)/16
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);// Sensitivity Scale Factor: raw ~65.5 = 1 deg/s = (2^15)/500
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.getAccelerometerSensor()->printSensorDetails();
    mpu.getGyroSensor()->printSensorDetails();
    switch (mpu.getCycleRate())
    {
    case MPU6050_CYCLE_40_HZ:
      Serial.println("MPU6050_CYCLE_40_HZ");
      break;
    case MPU6050_CYCLE_20_HZ:
      Serial.println("MPU6050_CYCLE_20_HZ");
      break;
    case MPU6050_CYCLE_5_HZ:
      Serial.println("MPU6050_CYCLE_5_HZ");
      break;
    case MPU6050_CYCLE_1_25_HZ:
      Serial.println("MPU6050_CYCLE_1_25_HZ");
      break;
    default:
      Serial.println("Cycle: ...");
      break;
    }

    calibrate();  
  }

  motorShield.begin();
    
  delay(1000);
}

// const int MAX_SIZE_MPU_BUFFER = 10;
// std::vector<> mpuBuffer;//heap
unsigned long delayRead = 10000;// 1/250 Microseconds
unsigned long lastStartTime = 0;
unsigned long mpuLastTimeSampled = 0;

void Drone::Update(JoystickControllerData* input) 
{
  if (micros() - lastStartTime > delayRead) {
    lastStartTime = micros();

    if (mpu.getEvent(&a, &g, &temp)) 
    {
      //===================================================
      //                    PHYSICS
      //===================================================
      // Time difference between now and last sample
      float mpuDeltaTime = ((float)(millis() - mpuLastTimeSampled))/1000.0;
      mpuLastTimeSampled = millis();

      prevPosition = position;
      prevVelocity = velocity;
      prevRotation = rotation;
      prevAngularVelocity = angularVelocity;   

      // =============== GYRO RAD/S ================
      Vector3<float> gyroAngularVel = Vector3<float>(g.gyro.v) - gyroZeroOffset;// rad/s

      angularVelocity.x = RadToDeg(gyroAngularVel.x);
      angularVelocity.y = RadToDeg(gyroAngularVel.y);
      angularVelocity.z = RadToDeg(gyroAngularVel.z);
      rotation += angularVelocity * mpuDeltaTime; // delta rotate n degrees
        
      // =============== ACCELEROMETER M/S^2 ================
      Vector3<float> gyroAccel = (Vector3<float>(a.acceleration.v) - accelZeroOffset); // m/s/s
      
      velocity += gyroAccel * mpuDeltaTime;
      position += gyroAccel * (mpuDeltaTime * mpuDeltaTime);

        // --------------Buffer Storage------------------
        // if (mpuBuffer.size() >= MAX_SIZE_MPU_BUFFER) {
        //   mpuBuffer.clear();
        // }
        // mpuBuffer.push_back();
        
        // Serial.println("------------------");
        // Serial.println(String("Heading: ")+g.gyro.heading);
        // Serial.println(String("Roll: ")+g.gyro.roll);
        // Serial.println(String("Pitch: ")+g.gyro.pitch);
        // Serial.println(mpuDeltaTime, 4);
        // Serial.println("------------------");
    }

  }
  //clamp range
  clamp(m1Speed, motorMinSpeed, motorMaxSpeed);
  clamp(m2Speed, motorMinSpeed, motorMaxSpeed);
  clamp(m3Speed, motorMinSpeed, motorMaxSpeed);
  clamp(m4Speed, motorMinSpeed, motorMaxSpeed);
  m1->setSpeed(m1Speed);
  m2->setSpeed(m2Speed);
  m3->setSpeed(m3Speed);
  m4->setSpeed(m4Speed);
  m1->run(FORWARD);
  m2->run(FORWARD);
  m3->run(FORWARD);
  m4->run(FORWARD);
}