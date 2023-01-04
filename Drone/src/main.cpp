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
const int BAUD_RATE = 115200;
void setup()
{
  Serial.begin(BAUD_RATE);
}

//Matrix3x3 rotation = Matrix3x3(IDENTITY3x3);
Matrix3x3 rotation = IDENTITY3x3;

void loop() 
{
  Serial.println("");
  for (size_t i = 0; i < 3; i++)
  {
    
      Vector3<float> row = rotation.matrix[i];
      String pipe = String("");//String(" | ");
      Serial.println(pipe+row.x+", "+row.y+", "+row.z+pipe);
  }
    
  delay(1000);

  rotation *= YPR(90*PI/180, 90*PI/180, 90*PI/180);
  
/* UNDO Euler rotation:
    rotation = rotation * YPR(180*PI/180, 45*PI/180, 90*PI/180);
    rotation = rotation * RPY(-180*PI/180, -45*PI/180, -90*PI/180);
*/
  //rotation = YPR(180*PI/180, 45*PI/180, 90*PI/180);
  //rotation = rotation * RotZ(PI/2.0);
  //rz;//rotation = Multiply(rotation.matrix, RotZ(PI/2.0).matrix);// Multiply(Multiply(RotZ(45*PI/180).matrix, RotY(45*PI/180).matrix).matrix, RotX(45*PI/180).matrix);
  // rotation = Multiply(rotation.matrix, rotationMatrix.matrix);
}