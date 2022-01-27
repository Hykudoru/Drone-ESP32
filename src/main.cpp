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
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>
#include <Drone.h>

double totalTime = 0.0; // seconds
double deltaTime = 0.0;//time difference (in seconds) between each loop;
void _timeLoop() {
  static unsigned long prevTime = 0.0;
  
  totalTime = millis()/1000.0; // seconds
  deltaTime = (millis() - prevTime)/1000.0;// delta seconds
  prevTime = millis();//update time

  Serial.print("deltaTime (sec): ");Serial.println(deltaTime);
  Serial.print("Time (sec): ");Serial.println(totalTime);
}

void delay(double milliSec, void(*callback)(void))
{
  static unsigned long t = 0.0;
  //delay
  if ((millis() - t) >= milliSec)
  {
    t = millis(); //reset timer
    callback();
    Serial.println(String("----------------------CALLBACK : t:")+ t/1000.0);
  }
}

#if defined(ESP32)
  const int BAUD_RATE = 115200;
#else
  const int BAUD_RATE = 9600;
#endif
const int I2C_ADDR = 0x3C; // Can be shared with other I2C devices
const int BUTTON_A = 15;//GPIO 15 or A8
const int BUTTON_B = 32;
const int BUTTON_C = 14;

int ACCEL_GYRO_ADDR = 0x68;

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *m1 = motorShield.getMotor(1);
Adafruit_DCMotor *m2 = motorShield.getMotor(2);
Adafruit_DCMotor *m3 = motorShield.getMotor(3);
Adafruit_DCMotor *m4 = motorShield.getMotor(4);

int m1Speed = 0;
int m2Speed = 0;
int m3Speed = 0;
int m4Speed = 0;
const int motorMinSpeed = 0;
const int motorMaxSpeed = 255;

Drone drone = Drone();

void printVector(Vector3<float> vec, String header = "") {
  Serial.println("");

  Serial.print(header); 
  Serial.print(vec.x); Serial.print(", ");
  Serial.print(vec.y); Serial.print(", ");
  Serial.println(vec.z); 

  display.print(header); 
  display.print(vec.x); display.print(", ");
  display.print(vec.y); display.print(", ");
  display.println(vec.z); 

  display.display();
}
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

Vector3<float> accelCal;
Vector3<float> gyroCal;
Vector3<float> zeroOffsetAccel;
Vector3<float> zeroOffsetGyro;
void calibrate() {
  Vector3<float> avgAccel = Vector3<float>(0.0, 0.0, 0.0);
  Vector3<float> avgGyro = Vector3<float>(0.0, 0.0, 0.0);
  Vector3<float> errorAccel;
  Vector3<float> errorGyro;
  int nSamples = 1000;
  
  Serial.println("Calibrating...");
  mpu.getEvent(&a, &g, &temp);
  for (size_t i = 0; i < nSamples; i++) 
  {
    if (mpu.getEvent(&a, &g, &temp))
    { 
      avgAccel.x += a.acceleration.x;
      avgAccel.y += a.acceleration.y;
      avgAccel.z += a.acceleration.z;

      avgGyro += g.gyro.v;
    }
    delay(2);
  }

  // Vector average calculated from sampled vector sum
  avgAccel.x /= (double)nSamples;
  avgAccel.z /= (double)nSamples;
  avgAccel.y /= (double)nSamples;
  
  zeroOffsetAccel.x = avgAccel.x;
  zeroOffsetAccel.y = avgAccel.y;
  zeroOffsetAccel.z = avgAccel.z;

  avgGyro /= (double)nSamples;
  zeroOffsetGyro = avgGyro;

  //error = Vector3<float>(abs(avg.x), abs(avg.y), abs(avg.z-9.81));
  // accelCal = Vector3<float>(a.acceleration.v) - zeroOffsetAccel;
  // gyroCal = Vector3<float>(g.gyro.v) - zeroOffsetGyro;

  printVector(zeroOffsetAccel, "Zero Offset Accel: ");
  printVector(zeroOffsetGyro, "Zero Offset Gyro: ");
}

void mpuSetup() 
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
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.getAccelerometerSensor()->printSensorDetails();
    mpu.getGyroSensor()->printSensorDetails();
  }
  delay(1000);

  calibrate();
  
  delay(1000);
}

void _mpuLoop() 
{
  display.clearDisplay();
  display.setCursor(0, 0);
//delay(1000.0, calibrate);
  if (mpu.getEvent(&a, &g, &temp)) {
    accelCal = Vector3<float>(a.acceleration.v) - zeroOffsetAccel;
    gyroCal = Vector3<float>(g.gyro.v) - zeroOffsetGyro;
    
  }
}

void mode_1() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 1");

  printVector(accelCal, "Acceleration (m/s^2)");
  printVector(gyroCal, "Gyro (rad/s)");
          
  display.display();
}

void mode_2() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 2");

  // printVector(Vector3<float>(g.gyro.v), "Gyro (rad/s)");

  display.display();
}

void mode_3() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 3");

  calibrate();

  display.display();
  delay(5000);
}

void _inputLoop()
{
  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &mode_3;

  if(Serial.available())
  {
    char c = Serial.read();
    Serial.println(c);
    
    if (c == '+' || c == '-')
    {
      if (c == '+') {
        ++m1Speed;
        ++m2Speed;
        ++m3Speed;
        ++m4Speed;
      }
      else {
        --m1Speed;
        --m2Speed;
        --m3Speed;
        --m4Speed;
      }
      //clamp range
      clamp(m1Speed, motorMinSpeed, motorMaxSpeed);
      clamp(m2Speed, motorMinSpeed, motorMaxSpeed);
      clamp(m3Speed, motorMinSpeed, motorMaxSpeed);
      clamp(m4Speed, motorMinSpeed, motorMaxSpeed);

      Serial.println("-----------");
      Serial.println(String("Motor 1: ")+m1Speed);
      Serial.println(String("Motor 2: ")+m2Speed);
      Serial.println(String("Motor 3: ")+m3Speed);
      Serial.println(String("Motor 4: ")+m4Speed);
      Serial.println("-----------");
      delay(10);
    }
  }
}

bool DEBUGGING = true;
void setup() 
{
  Serial.begin(BAUD_RATE);
  if (DEBUGGING) {
    while (!Serial)
    {
      ;
    }
  }
  
  Serial.println("Setup...");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();//displays initial adafruit image
  display.clearDisplay();//clears initial adafruit image
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Setup...");
  display.display();
  
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  ptrMode = &mode_1;  
  
  mpuSetup();//drone.setup();
  motorShield.begin();
  delay(500);
  display.clearDisplay();
}

Vector3<float> v; 
void loop() 
{
  _timeLoop();
  _inputLoop();
  _mpuLoop();
  uint8_t motorSpeed = 255/2;//test

  if (!m1) {
    m1 = motorShield.getMotor(1);
    if (!m1) {
      //pause();
      return;
    }
  }

/*
for (size_t i = 0; i <= 5; i++)
{
  m1->setSpeed(51 * i);
  m1->run(FORWARD);
  delay(1000);

  // m1->setSpeed(0);
  // //m1->run(BACKWARD);
  // delay(1000);

  m1->setSpeed(51 * i);
  m1->run(BACKWARD);
  delay(1000);
}
*/
 m1->setSpeed(m1Speed);
  m1->run(BACKWARD);
 
  if (ptrMode) {
    (*ptrMode)();
  }
}