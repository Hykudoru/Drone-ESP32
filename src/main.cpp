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
#include <esp_now.h>
#include <WiFi.h>


double deltaTimeMillis = 0.0;//time difference (in milliseconds) between each loop;
double deltaTimeMicros = 0.0;//time difference (in microseconds) between each loop
double deltaTimeSec = 0.0;//time difference (in seconds) between each loop;

void TimeUpdate() {
  
  static unsigned long prevMillisTime = 0.0;
  static unsigned long prevMicrosTime = 0.0;

  deltaTimeMillis = (millis() - prevMillisTime);
  prevMillisTime = millis();

  deltaTimeMicros = (micros() - prevMicrosTime);
  prevMicrosTime = micros();

  deltaTimeSec = deltaTimeMillis/1000.0;// delta seconds

  Serial.println(String("Time: ")+millis()/1000.0+"s \t"+millis()+"ms \t"+micros()+"micros");
  Serial.println(String("Time Delta")+deltaTimeSec+"s \t"+deltaTimeMillis+"ms \t"+deltaTimeMicros+"micros");
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


void duelPrint(Vector3<int> vec, String header = "")
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

typedef struct DroneData
{
  String info;
};

typedef struct JoystickData
{
  Vector3<int> leftJoystick;
  Vector3<int> rightJoystick;
};

DroneData outgoingData;
JoystickData incomingData;
JoystickData* ptrData = NULL;
uint8_t selfMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; //Drone MAC address = 94:B9:7E:5F:51:40
uint8_t broadcastMACAddress[] {0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34}; // controller MAC address
esp_now_peer_info_t peerInfo;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
  static int count = 0;
  if (data != NULL)
  {
    Serial.println(++count);
    memcpy(&incomingData, data, sizeof(data));
  }
  ptrData = &incomingData;
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
    Serial.println(String("STATUS: ")+(status == ESP_NOW_SEND_SUCCESS));
}

void SetupESPNOW()
{
  WiFi.mode(WIFI_MODE_STA);
  Serial.println("MAC Address: "+WiFi.macAddress());
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP_NOW failed to init");
    return;
  }
  esp_now_register_recv_cb(OnDataReceived);

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("ESP_NOW failed to add peer");
    return;
  }
}

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

  oled.display();
}

void mode_3() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 3");

  //calibrate();

  oled.display();
}

void InputUpdate()
{
  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &mode_3;

  if(Serial.available())
  {
    char ch = Serial.read();
    Serial.println(ch);
    

    if (ch == 'a') ptrMode = &mode_1;
    if (ch == 'b') ptrMode = &mode_2;
    if (ch == 'c') ptrMode = &mode_3;

    // Pressing 1 - 4 starts or stops coresponding motor
    switch (ch)
    {
    case '1':
      drone.m1Speed = ((byte)(drone.m1Speed != drone.motorMaxSpeed)) * drone.motorMaxSpeed;
      break;
    case '2':
      drone.m2Speed = ((byte)(drone.m2Speed != drone.motorMaxSpeed)) * drone.motorMaxSpeed;
      break;
    case '3':  
      drone.m3Speed = ((byte)(drone.m3Speed != drone.motorMaxSpeed)) * drone.motorMaxSpeed;
      break;
    case '4':  
      drone.m4Speed = ((byte)(drone.m4Speed != drone.motorMaxSpeed)) * drone.motorMaxSpeed;
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
      
      Serial.println("-----------");
      Serial.println(String("Motor 1: ")+drone.m1Speed);
      Serial.println(String("Motor 2: ")+drone.m2Speed);
      Serial.println(String("Motor 3: ")+drone.m3Speed);
      Serial.println(String("Motor 4: ")+drone.m4Speed);
      Serial.println("-----------");
    }
    //delay(10);
  }
}

bool DEBUGGING = false;
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
  
  SetupESPNOW();

  drone.Init();
  
  delay(500);
  oled.clearDisplay();
}

pointerFunction procedureQueue[] = {TimeUpdate, InputUpdate};
void loop() 
{
  for (size_t i = 0; i < sizeof(procedureQueue)/sizeof(pointerFunction); i++)
  {
   procedureQueue[0]();
  }

  if (ptrData != NULL)
  {
    duelPrint((*ptrData).leftJoystick, "LEFT JOYSTICK");
    duelPrint((*ptrData).rightJoystick, "RIGHT JOYSTICK");
    drone.Update();
    ptrData = NULL;
  }

  //clamp range
  clamp(drone.m1Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
  clamp(drone.m2Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
  clamp(drone.m3Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
  clamp(drone.m4Speed, drone.motorMinSpeed, drone.motorMaxSpeed);
  drone.m1->setSpeed(drone.m1Speed);
  drone.m2->setSpeed(drone.m2Speed);
  drone.m3->setSpeed(drone.m3Speed);
  drone.m4->setSpeed(drone.m4Speed);
  drone.m1->run(FORWARD);
  drone.m2->run(FORWARD);
  drone.m3->run(FORWARD);
  drone.m4->run(FORWARD);
 
  if (ptrMode) {
    (*ptrMode)();
  }

  String msg = "Drone Acceleration: <";
  msg += drone.GetAcceleration().x;
  msg += ", ";
  msg += drone.GetAcceleration().y;
  msg += ", ";
  msg += drone.GetAcceleration().z;
  msg += ">";

  // Assign values
  outgoingData.info = msg;
  // Send data
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
  if (result == ESP_OK)
  {
    Serial.println("Data sent!");
    oled.println(String(sizeof(outgoingData))+" bytes sent. ");
  } else {
    Serial.println("Error sending data.");
    oled.println("Error sending data.");
  }
}