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
#include <Functions.h>

double totalTime = 0.0; // seconds
double deltaTime = 0.0;//time difference (in seconds) between each loop;
void timeUpdate() {
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

#if defined(oled)
#else
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);
#endif
void duelPrint(Vector3<float> vec, String header = "")
{
    Serial.println("");

    Serial.print(header); 
    Serial.print(vec.x); Serial.print(", ");
    Serial.print(vec.y); Serial.print(", ");
    Serial.println(vec.z); 

    oled.print(header); 
    oled.print(vec.x); oled.print(", ");
    oled.print(vec.y); oled.print(", ");
    oled.println(vec.z); 

    oled.display();
}

Drone drone = Drone();


void mode_1() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 1");

  duelPrint(drone.GetAcceleration(), "Acceleration (m/s^2)");
  duelPrint(drone.GetAngularVelocity(), "Gyro (rad/s)");
          
  oled.display();
}

void mode_2() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 2");

  // printVector(Vector3<float>(g.gyro.v), "Gyro (rad/s)");

  oled.display();
}

void mode_3() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 3");

  //calibrate();

  oled.display();
  delay(5000);
}

void inputUpdate()
{
  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &mode_3;

  if(Serial.available())
  {
    char ch = Serial.read();
    Serial.println(ch);
    
    // Pressing 1 - 4 starts or stops coresponding motor
    switch (ch)
    {
    case '1':
      drone.m1Speed = (byte)(drone.m1Speed != drone.motorMaxSpeed) * drone.motorMaxSpeed;
      drone.m1->setSpeed(drone.m1Speed);
      break;
    case '2':
      drone.m2Speed = (byte)(drone.m2Speed != drone.motorMaxSpeed) * drone.motorMaxSpeed;
      drone.m2->setSpeed(drone.m2Speed);
      break;
    case '3':  
      drone.m3Speed = (byte)(drone.m3Speed != drone.motorMaxSpeed) * drone.motorMaxSpeed;
      drone.m3->setSpeed(drone.m3Speed);
      break;
    case '4':  
      drone.m4Speed = (byte)(drone.m4Speed != drone.motorMaxSpeed) * drone.motorMaxSpeed;
      drone.m4->setSpeed(drone.m4Speed);
      break;
    default:
      break;
    }

    if (ch == '+' || ch == '-')
    {
      if (ch == '+') {
        ++drone.m1Speed;
        ++drone.m2Speed;
        ++drone.m3Speed;
        ++drone.m4Speed;
      }
      else {
        --drone.m1Speed;
        --drone.m2Speed;
        --drone.m3Speed;
        --drone.m4Speed;
      }
      //clamp range
      clamp(drone.m1Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
      clamp(drone.m2Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
      clamp(drone.m3Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
      clamp(drone.m4Speed, drone.motorMinSpeed, drone.motorMaxSpeed);

      Serial.println("-----------");
      Serial.println(String("Motor 1: ")+drone.m1Speed);
      Serial.println(String("Motor 2: ")+drone.m2Speed);
      Serial.println(String("Motor 3: ")+drone.m3Speed);
      Serial.println(String("Motor 4: ")+drone.m4Speed);
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
      delay(100);
    }
  }
  
  Serial.println("Setup...");

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();//displays initial adafruit image
  oled.clearDisplay();//clears initial adafruit image
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Setup...");
  oled.display();
  
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  ptrMode = &mode_1;  
  
  drone.Init();
  delay(500);
  oled.clearDisplay();
}

void loop() 
{
  timeUpdate();
  inputUpdate();
  drone.Update();

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

  drone.m1->setSpeed(drone.m1Speed);
  drone.m1->setSpeed(drone.m2Speed);
  drone.m1->setSpeed(drone.m3Speed);
  drone.m1->setSpeed(drone.m4Speed);
  drone.m1->run(FORWARD);
  drone.m2->run(FORWARD);
  drone.m3->run(FORWARD);
  drone.m4->run(FORWARD);
 
  if (ptrMode) {
    (*ptrMode)();
  }

  delay(2);
}