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

double totalTime = 0.0;
double deltaTime = 0.0;//time difference (in seconds) between each loop;
void timeLoop() {
  static unsigned long prevTime = 0.0;
  
  totalTime = millis()/1000.0;
  deltaTime = (millis() - prevTime)/1000.0;

  Serial.print("deltaTime: ");Serial.println(deltaTime);
  Serial.print("Time: ");Serial.println(totalTime);

  prevTime = millis();//update time
}

void delay(double interval, void(*callback)(void))
{
  static unsigned long t = 0.0;
  //delay
  if ((totalTime - t) >= interval/1000.0)
  {
    t = totalTime; //reset timer
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
Drone drone = Drone();
sensors_event_t a, g, temp;


void printVector(double x, double y, double z, String header = "") {
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

Vector3 zeroOffsetAccel;
Vector3 acceleration() {
  return Vector3(a.acceleration.x - zeroOffsetAccel.x, a.acceleration.y - zeroOffsetAccel.y, a.acceleration.z - zeroOffsetAccel.z);
}

void calibrate() {
  double x, y, z = 0.0;
  int nSamples = 100;

  Serial.println("Calibrating...");
  for (size_t i = 0; i < nSamples; i++) 
  {
    if (mpu.getEvent(&a, &g, &temp))
    {
      Serial.print(mpu.getEvent(&a, &g, &temp));
      x += a.acceleration.x;
      y += a.acceleration.y;
      z += a.acceleration.z;
    }
    
    delayMicroseconds(10);
  }

  x /= (float)nSamples;
  y /= (float)nSamples;
  z /= (float)nSamples;
  
  zeroOffsetAccel.x = x-0.0;
  zeroOffsetAccel.y = y-0.0;
  zeroOffsetAccel.z = z-9.81;

  Vector3 calib = acceleration();
  printVector(zeroOffsetAccel.x, zeroOffsetAccel.y, zeroOffsetAccel.z, "Zero Offset");
  printVector(a.acceleration.x, a.acceleration.y, a.acceleration.z, "Before: ");
  printVector(calib.x, calib.y, calib.z, "Calibrated: ");
  delay(5000);
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
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    //mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

Vector3 accel;
void mpuLoop() 
{
  static int count = 0;
  display.clearDisplay();
  display.setCursor(0, 0);

  if (mpu.getEvent(&a, &g, &temp)) {
    count++;
    Serial.printf("mpu.getEvent %d times \n", count);
    accel = acceleration();
    //accel.x *= deltaTime;
    //accel.y *= deltaTime;
    //accel.z *= deltaTime;
  }
}

void mode_1() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 1");

  printVector(a.acceleration.x, a.acceleration.y, a.acceleration.z, "Acceleration (m/s^2)");
  printVector(accel.x, accel.y, accel.z, "Calibrated Acceleration");
  printVector(accel.x*deltaTime, accel.y*deltaTime, accel.z*deltaTime, "Calibrated Acceleration * delta time");
          
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

  calibrate();

  display.display();
}


void debug() {
  unsigned long ul = millis();
  long l = millis();
  float f = millis();
  double d = millis();
  Serial.println(ul);
  Serial.println(l);
  Serial.println(f);
  Serial.println(d);
  Serial.println(String("millis()/1000.0: ")+millis()/1000.0);
  Serial.println(String("millis()/1000.0: ")+millis()/1000.0);

  Serial.println(String("Status: ")+a.acceleration.status);
  Serial.println(String("Heading: ")+a.acceleration.heading);
  Serial.println(String("Voltage: ")+a.voltage);
  Serial.println(String("Current: ")+a.current);
  Serial.println(String("Distance: ")+a.distance+" cm");
  Serial.println(String("Clock: ")+mpu.getClock()+"");
  Serial.println(String("Cycle Rate: ")+mpu.getCycleRate()+"");
  Serial.println(String("Sample Rate Divisor: ")+mpu.getSampleRateDivisor()+"");
  Serial.println(String("Filter Bandwidth: ")+mpu.getFilterBandwidth()+"");
  Serial.println(String("Accelerometer Range: ")+mpu.getAccelerometerRange()+"");
  delay(100.0);
}
void test()
{
  Serial.println("Test");
}

void inputLoop()
{
  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &mode_3;

  if(Serial.available())
  {
    char c = Serial.read();
    Serial.print("Serial.read() : "); 
    Serial.println(c);
    
    if (c == 'd')
    {
      ptrMode = &debug;// or debug(); delay(1000);
    }
    if (c == 'c')
    {
      ptrMode = &calibrate;
    }
    if (c == 'p') {
      delay(5000);
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
  calibrate();

  delay(500);
  display.clearDisplay();
}
 
void loop() 
{
  static int count = 0;
  count++;
  Serial.printf("-----------loop() count:  %d times \n", count);
  
  timeLoop();
  mpuLoop();
  inputLoop();

  (*ptrMode)();

  delay(500, test);
  delay(500, test);
  delay(500, test);
}