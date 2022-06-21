#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Vector.h>
#include <MuxJoystick.h>
#include <Data.h>
#include <Functions.h>
#include <esp_now.h>
#include <WiFi.h>

#if defined(ESP32)
  const int BAUD_RATE = 115200;
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
  #define BUTTON_1 27
  #define BUTTON_2 4
  
  #define POTENTIOMETER_1 0b100111// A3 (39)
  #define POTENTIOMETER_2 0b100100// A4 (36)
  const uint16_t ADC_RESOLUTION = 4095; // 0 - 4095
  uint16_t potentiometerValue = 0;
  uint16_t potentiometerValue2 = 0;
#endif
#if defined(__AVR_ATmega32U4__)
  const int BAUD_RATE = 9600;
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
#endif
const int LEFT_JOYSTICK_MUX_PORT = 0;
const int RIGHT_JOYSTICK_MUX_PORT = 7;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT, false, false);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT, false, false);
//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

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


uint8_t selfMACAddress[] {0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34}; //Controller MAC = 0C:DC:7E:CA:D2:34
uint8_t broadcastMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; //Drone MAC = 94:B9:7E:5F:51:40
esp_now_peer_info_t peerInfo;
JoystickControllerData outgoingData;
const int MAX_DATA_BUFFER_SIZE = 10;
DroneData incomingData;
DroneData incomingDataBuffer[255];
int outgoingSuccessCount = 0;
int outgoingFailCount = 0;
int outgoingCount = 0;
int incomingCount = 0;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
    Serial.println("Received: "+ (++incomingCount));
    memcpy(&incomingData, data, sizeof(data));
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
    delay(5000);
    return;
  }
}

void DisplayMode1() 
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(String("(")+leftJoystick.muxPort+") X:"+outgoingData.LeftJoystick.x+" Y:"+outgoingData.LeftJoystick.y+" Z:"+outgoingData.LeftJoystick.z);
  oled.println(String("(")+rightJoystick.muxPort+") X:"+outgoingData.RightJoystick.x+" Y:"+outgoingData.RightJoystick.y+" Z:"+outgoingData.RightJoystick.z);
  oled.display();
}
void DisplayMode2() 
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(String("Potentiometer: ")+outgoingData.Potentiomter);
  oled.println(String("Potentiometer 2: ")+outgoingData.Potentiomter2);
  // oled.println("DRONE");
  // oled.println(String("Acceleration: ")+"<"+incomingData.Acceleration.x+","+incomingData.Acceleration.y+","+incomingData.Acceleration.z+">");
  // oled.println(String("Angular Velocity: ")+"<"+incomingData.AngularVelocity.x+","+incomingData.AngularVelocity.y+","+incomingData.AngularVelocity.z+">");
  oled.display();
}
void DisplayMode3() 
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(String("Attempts:")+outgoingCount);
  oled.println(String("Sent:")+outgoingSuccessCount+", Failed:"+outgoingFailCount);
  oled.println(String("Received:")+incomingCount);
  oled.display();
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();//displays initial adafruit image
  oled.clearDisplay();//clears initial adafruit image
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Setup...");
  oled.display();
  delay(100);
  oled.clearDisplay();

  // INPUT_PULLUP button MUST be connected to GND
  // INPUT_PULLDOWN button MUST be connected to VCC
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_1, INPUT_PULLDOWN);
  pinMode(BUTTON_2, INPUT_PULLDOWN);
  
  leftJoystick.Start();
  rightJoystick.Start();
  ptrMode = &DisplayMode1;

  SetupESPNOW();

  oled.display();
  delay(100);
 }

void loop() 
{
  static unsigned long time = millis();
  static unsigned long sendDelay = 0;
  static int modeIndex = 0;
  static pointerFunction modes[] = {DisplayMode1, DisplayMode2, DisplayMode3};

  if (digitalRead(BUTTON_1) == 1) modeIndex--;//ptrMode = &DisplayMode1;
  if (digitalRead(BUTTON_2) == 1) modeIndex++; //ptrMode = &DisplayMode2;
  constrain(modeIndex, 0, 2); 
  ptrMode = modes[modeIndex];

potentiometerValue = analogRead(POTENTIOMETER_1);
potentiometerValue2 = analogRead(POTENTIOMETER_2);

  // Update values
  outgoingData.ID = (outgoingData.ID + 1) > sizeof(outgoingData.ID) ? 0 : outgoingData.ID + 1;
  outgoingData.Potentiomter = potentiometerValue;
  outgoingData.Potentiomter2 = potentiometerValue2;
  outgoingData.LeftJoystick = leftJoystick.Read(100);//.Normalize();
  outgoingData.RightJoystick = rightJoystick.Read(100);//.Normalize();
  
  time += millis();
  if (millis() - time > sendDelay)
  {
    time = millis();
    // Send data
    esp_now_send(broadcastMACAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
    outgoingCount++;
  }

  if (ptrMode) {
    (*ptrMode)();
  }

  //Wait 2 milliseconds for analog-to-digital converted to settle after last reading 
  delay(2);//delay(20);
}