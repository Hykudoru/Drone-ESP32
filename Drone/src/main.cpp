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


float deltaTimeSeconds = 0.0;//time difference (in seconds) between each loop;
unsigned long deltaTimeMillis = 0U;//time difference (in milliseconds) between each loop;
unsigned long deltaTimeMicros = 0U;//time difference (in microseconds) between each loop

void Time() {
  
  static unsigned long prevMillisTime = millis();
  static unsigned long prevMicrosTime = micros();

  deltaTimeMillis = (millis() - prevMillisTime);
  prevMillisTime = millis();

  deltaTimeMicros = (micros() - prevMicrosTime);
  prevMicrosTime = micros();

  deltaTimeSeconds = deltaTimeMillis/1000.0;
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


#define DEBUGGING
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
const int LED_1 = LED_BUILTIN;
int ACCEL_GYRO_ADDR = 0x68;
//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);
Drone drone = Drone();

//==================================================================
//                  ESPNOW WIRELESS COMMUNICATION  
//==================================================================
const int MAX_DATA_BUFFER_SIZE = 10;
const unsigned long INCOMING_DATA_LIFETIME = 50UL;
uint8_t selfMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; //Drone MAC address = 94:B9:7E:5F:51:40
uint8_t broadcastMACAddress[] {0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34}; // controller MAC address
esp_now_peer_info_t peerInfo;
DroneData outgoingData = DroneData();
JoystickControllerData incomingData = JoystickControllerData();
JoystickControllerData* ptrInput = NULL;
WirelessData incomingDataBuffer[MAX_DATA_BUFFER_SIZE];
unsigned int outgoingSuccessCount = 0;
unsigned int outgoingFailCount = 0;
unsigned int outgoingCount = 0;
unsigned int incomingCount = 0;
unsigned long timeSinceLastIncoming = 0;
bool newInputReceived = false;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
  // static int count = -1;
  // count = (count + 1) > MAX_DATA_BUFFER_SIZE ? 0 : count + 1;
  timeSinceLastIncoming = 0;
  newInputReceived = true;
  incomingCount++;
  memcpy(&incomingData, data, sizeof(incomingData));
  ptrInput = &incomingData;

  //Serial.println("------INCOMING------");
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

  switch (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
  case ESP_ERR_ESPNOW_EXIST:
    Serial.println("ESP_NOW Error: Peer already existed!");
    delay(2000);
  break;
  case ESP_OK:
    Serial.println("ESP_NOW: Peer added!");
    delay(2000);
    break;
  default:
    break;
  }
  // if (esp_now_add_peer(&peerInfo) != ESP_OK)
  // {
  //   Serial.println("ESP_NOW failed to add peer");
  //   return;
  // }
}

//==================================================================
//                          DEBUGGING
//==================================================================
//==================================================================
//                            MODES 
//==================================================================
//--------------MODES-------------
void mode_0()
{

}

void mode_1() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 1");

  Serial.println("-----Keys [1,2,3,4,+,-]------");
  Serial.println(String("Motor 1: ")+drone.m1Speed);
  Serial.println(String("Motor 2: ")+drone.m2Speed);
  Serial.println(String("Motor 3: ")+drone.m3Speed);
  Serial.println(String("Motor 4: ")+drone.m4Speed);
  Serial.println("-----------");
          
  oled.display();
}

void mode_2() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Mode 2");
  
  byte value = (byte) map(ptrInput->Potentiometer, 0, ADC_RESOLUTION, 0, 255);
  drone.m1Speed = value; 
  drone.m2Speed = value;
  drone.m3Speed = value;
  drone.m4Speed = value;

  Serial.println("-----Potentiometer------");
  Serial.println(String("Motor 1: ")+drone.m1Speed);
  Serial.println(String("Motor 2: ")+drone.m2Speed);
  Serial.println(String("Motor 3: ")+drone.m3Speed);
  Serial.println(String("Motor 4: ")+drone.m4Speed);
  Serial.println("-----------");
  
  oled.display();
}

void mode_3() 
{
  Serial.println(String("Total Time: ")+millis()/1000UL+"s \t"+millis()+" milliseconds \t"+micros()+" microseconds");
  Serial.println(String("Delta Time (since last frame): ")+"\t"+deltaTimeSeconds+" seconds \t"+"\t"+deltaTimeMillis+" milliseconds \t"+deltaTimeMicros+" microseconds");
  Serial.println(String("Attempts:")+outgoingCount);
  Serial.println(String("Sent:")+outgoingSuccessCount+", Failed:"+outgoingFailCount);
  Serial.println(String("Received:")+incomingCount);

  duelPrint(incomingData.LeftJoystick, "LEFT JOYSTICK ");
  duelPrint(incomingData.RightJoystick, "RIGHT JOYSTICK ");
  
  // Serial.println("-----------");
  // Serial.println(String("Motor 1: ")+drone.m1Speed);
  // Serial.println(String("Motor 2: ")+drone.m2Speed);
  // Serial.println(String("Motor 3: ")+drone.m3Speed);
  // Serial.println(String("Motor 4: ")+drone.m4Speed);
  // Serial.println("-----------");

  duelPrint(drone.GetVelocity(), "Velocity:");
  duelPrint(drone.GetPosition(), "Position:");
  duelPrint(drone.GetRotation(), "Rotation:");  
  
  
}

void Pause() 
{
  Serial.println("Paused... Press 'p' to resume.");
  while (Serial.read() != 'p') 
  {
    while (Serial.read() == 'd') 
    {
      Serial.println("Holding 'd'.");
      esp_now_send(broadcastMACAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
    }
  }
}

void Input()
{
  if (digitalRead(BUTTON_A) == 0) ptrMode = &mode_1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = &mode_2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = &mode_3;

//---------SERIAL INPUT-------------
  if(Serial.available())
  {
    char ch = Serial.read();
    Serial.println(ch);
    
    if (ch == 'a') ptrMode = &mode_1;
    if (ch == 'b') ptrMode = &mode_2;
    if (ch == 'c') ptrMode = &mode_3;

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
    
    //------Paused--------
    if (ch == 'p') 
    {
      Pause();
    }
  }
}

//=======================================================================================================
//                                            SETUP 
//=======================================================================================================

void setup() 
{
  Serial.begin(BAUD_RATE);
  #if defined(DEBUGGING)
    while (!Serial)
    {
      delay(100);
    }
  #endif

  Serial.println("Setup...");

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();//displays initial adafruit image
  oled.clearDisplay();//clears initial adafruit image
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Setup...");
  oled.display();
  
  // INPUT_PULLUP button MUST be connected to GND
  // INPUT_PULLDOWN button MUST be connected to VCC
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(LED_1, OUTPUT);

  ptrMode = &mode_0;  
  // #if defined(DEBUGGING)
  //   ptrMode = &DebugMode;
  // #endif
    
  SetupESPNOW();
  ptrInput = &incomingData;
  
  //===========================================================
  // AWAITING COMMAND/INPUT: LED BLINKS slowly.
  // Note: Must place drone on flat surface before calibrating.
  //===========================================================
  Serial.println("Press & hold down both joysticks to begin calibrating drone.");
  while((Serial.read() != 'p') && (incomingData.LeftJoystick.z && incomingData.RightJoystick.z) == false )
  {
    digitalWrite(LED_1, HIGH);
    delay(500);
    digitalWrite(LED_1, LOW);
    delay(500);
  }

  //============================================
  // BEGIN CALIBRATING: LED ON. 
  // Note: Do not touch drone while calibrating.
  //============================================
  digitalWrite(LED_1, HIGH);
  drone.Init();

  //=====================================================
  // DONE CALIBRATING: LED BLINKS rapidly then OFF.
  //=====================================================
  { 
    for (size_t i = 0; i < 4; i++)
    {
      digitalWrite(LED_1, HIGH);
      delay(100);
      digitalWrite(LED_1, LOW);
      delay(100);
    }
  }

  oled.clearDisplay();
}

pointerFunction procedureQueue[] = {
  Time, 
  Input
};

//=======================================================================================================
//                                            MAIN LOOP 
//=======================================================================================================
void loop() 
{
  static bool init = false;
  if (!init){
    init = true;
    Serial.println("Running...");
  }

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
  
  drone.Update(&incomingData);//drone.Update(input);
 
 static unsigned long timer = 0;
 timer += deltaTimeMillis;
 if (timer >= 100UL)
 {
   timer = 0;
    if (ptrMode) {
      (*ptrMode)();
    }
 }
}