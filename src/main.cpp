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
#include <Drone.h>

unsigned long totalTime = 0;
unsigned long deltaTime = 0;//time difference (in seconds) between each loop;

void timeLoop() {
  static unsigned long prevTime = 0;
  
  totalTime = millis()/1000.0;
  deltaTime = (millis() - prevTime)/1000.0;

  Serial.print("deltaTime:");Serial.println(deltaTime);
  Serial.print("Time:");Serial.println(totalTime);

  prevTime = millis();//update time
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
Drone drone = Drone();
sensors_event_t a, g, temp;

void printVector(float x, float y, float z, String header = "") {
  Serial.println("");

  Serial.println(header); 
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.println(z); 

  display.println(header); 
  display.print(x); display.print(", ");
  display.print(y); display.print(", ");
  display.println(z); 

  display.display();
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
    Serial.println(mpu.getAccelerometerRange());
    Serial.println(mpu.getGyroRange());
    Serial.println(mpu.getFilterBandwidth());
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  //mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}
                            static Vector3 vector;
void mpuLoop() 
{
  static int count = 0;
  display.clearDisplay();
  display.setCursor(0, 0);

  if (mpu.getEvent(&a, &g, &temp)) {
    count++;
    Serial.printf("mpu.getEvent %d times \n", count);

                            vector.x += a.acceleration.x;
                            vector.y += a.acceleration.y;
                            vector.z += a.acceleration.z;
  }
}

void mode_1() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 1");

  printVector(a.acceleration.x, a.acceleration.y, a.acceleration.z, "Acceleration (m/s^2)");

  display.display();
}

void mode_2() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 2");

  printVector(g.gyro.x, g.gyro.y, g.gyro.z, "Gyro (rad/s)");

  display.display();
}


void mode_3() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 3");

  display.display();
}

void setup() 
{
  Serial.begin(BAUD_RATE);
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

  delay(500);
  display.clearDisplay();
}

void loop() 
{
  timeLoop();

  static float t = 0;
  static float delayPeriod = 1.0;// sec
  
  //delay
  if ((totalTime - t) >= delayPeriod)
  {
    t = totalTime; //reset timer
  }

  mpuLoop();

  //input
  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &mode_3;

  (*ptrMode)();

  delay(10);
}