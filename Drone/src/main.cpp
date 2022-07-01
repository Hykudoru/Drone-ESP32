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
#include <WirelessData.h>
#include "../lib/src/Drone.h"//#include <Drone.h>


unsigned long deltaTimeMillis = 0.0;//time difference (in milliseconds) between each loop;
unsigned long deltaTimeMicros = 0.0;//time difference (in microseconds) between each loop

void Time() {
  
  static unsigned long prevMillisTime = millis();
  static unsigned long prevMicrosTime = micros();

  deltaTimeMillis = (millis() - prevMillisTime);
  prevMillisTime = millis();

  deltaTimeMicros = (micros() - prevMicrosTime);
  prevMicrosTime = micros();
}

void delay(unsigned long milliSec, void(*callback)(void))
{
  static unsigned long t = 0.0;
  //delay
  if ((millis() - t) >= milliSec)
  {
    t = millis(); //reset timer
    callback();
    Serial.println("----------------------CALLBACK : t:" + (t/1000UL));
  }
}

#if defined(ESP32)
  const int BAUD_RATE = 115200;
  const uint16_t ADC_RESOLUTION = 4095; // 0 - 4095
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

// #if defined(oled)
// #else
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);
// #endif

Drone drone = Drone();

//================ ESPNOW WIRELESS COMMUNICATION DATA VARS ================

const int MAX_DATA_BUFFER_SIZE = 10;
const unsigned long INCOMING_DATA_LIFETIME = 50UL;
uint8_t selfMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; //Drone MAC address = 94:B9:7E:5F:51:40
uint8_t broadcastMACAddress[] {0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34}; // controller MAC address
esp_now_peer_info_t peerInfo;
DroneData outgoingData;
JoystickControllerData incomingData;
JoystickControllerData* ptrInput = NULL;
WirelessData incomingDataBuffer[MAX_DATA_BUFFER_SIZE];
int outgoingSuccessCount = 0;
int outgoingFailCount = 0;
int outgoingCount = 0;
int incomingCount = 0;
unsigned long timeSinceLastIncoming = 0;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
  // static int count = -1;
  // count = (count + 1) > MAX_DATA_BUFFER_SIZE ? 0 : count + 1;
  timeSinceLastIncoming = 0;
  incomingCount++;
  memcpy(&incomingData, data, sizeof(incomingData));
  ptrInput = &incomingData;

  Serial.println("------INCOMING------");
  // outgoingData.Acceleration = drone.GetAcceleration();
  // outgoingData.AngularVelocity = drone.GetAngularVelocity();
  // esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS) {
    outgoingSuccessCount++;
  }
  else if (status == ESP_NOW_SEND_FAIL) {
    outgoingFailCount++;
  }
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

//--------------MODES-------------
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
  /**/
  oled.display();
}

void DebugMode() 
{
  Serial.println(String("Total Time: ")+millis()/1000UL+"s \t"+millis()+" milliseconds \t"+micros()+" microseconds");
  Serial.println(String("Delta Time (since last frame): ")+"\t"+deltaTimeMillis+" milliseconds \t"+deltaTimeMicros+" microseconds");
  Serial.println(String("Attempts:")+outgoingCount);
  Serial.println(String("Sent:")+outgoingSuccessCount+", Failed:"+outgoingFailCount);
  Serial.println(String("Received:")+incomingCount);
}

void Input()
{

  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &DebugMode;
  
  byte value = (byte) map(ptrInput->Potentiomter, 0, ADC_RESOLUTION, 0, 255);
  drone.m1Speed = value; 
  drone.m2Speed = value;
  drone.m3Speed = value;
  drone.m4Speed = value;

//---------SERIAL INPUT-------------
  if(Serial.available())
  {
    char ch = Serial.read();
    Serial.println(ch);
    
    if (ch == 'a') ptrMode = &mode_1;
    if (ch == 'b') ptrMode = &mode_2;
    if (ch == 'c') ptrMode = &DebugMode;

    // Pressing 1, 2, 3, 4 toggles corresponding motor at full speed or 0
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
    // Pressing - or + increases or decreases motor speed;
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
      
      
    }
    //delay(10);
  }
    Serial.println("-----------");
    Serial.println(String("Motor 1: ")+drone.m1Speed);
    Serial.println(String("Motor 2: ")+drone.m2Speed);
    Serial.println(String("Motor 3: ")+drone.m3Speed);
    Serial.println(String("Motor 4: ")+drone.m4Speed);
    Serial.println("-----------");
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
   if (DEBUGGING) {
    ptrMode = &DebugMode;
  }

  SetupESPNOW();

  drone.Init();
  
  delay(500);
  oled.clearDisplay();
}

pointerFunction procedureQueue[] = {
  Time, 
  Input
};

void loop() 
{
  for (size_t i = 0; i < sizeof(procedureQueue)/sizeof(pointerFunction); i++)
  {
    procedureQueue[i]();
  }

  //----------REMOTE INPUT-------------
  // Checks for stale data. Simulates continuous input if the last valid input state becomes stale/old from waiting too long for new incoming data.
  timeSinceLastIncoming += deltaTimeMillis;
  if (timeSinceLastIncoming >= INCOMING_DATA_LIFETIME) {
    ptrInput->LeftJoystick = Vector3<float>(0,0,0);
    ptrInput->RightJoystick = Vector3<float>(0,0,0);
    timeSinceLastIncoming = 0;
  }

  duelPrint(incomingData.LeftJoystick, "LEFT JOYSTICK ");
  duelPrint(incomingData.RightJoystick, "RIGHT JOYSTICK ");
  
  drone.Update(*ptrInput);//drone.Update(input);
 
 static unsigned long timer = 0;
 timer += deltaTimeMillis;
 if (timer >= 500UL)
 {
   timer = 0;
    if (ptrMode) {
      (*ptrMode)();
    }
 }
}