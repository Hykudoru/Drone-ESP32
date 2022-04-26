#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Vector.h>
#include <MuxJoystick.h>

#include <esp_now.h>
#include <WiFi.h>

#if defined(ESP32)

  const int BAUD_RATE = 115200;
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
 #define BUTTON_TOGGLE 27
 #define BUTTON_TOGGLE2 4
#endif
#if defined(__AVR_ATmega32U4__)
  const int BAUD_RATE = 9600;
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
#endif

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

const int LEFT_JOYSTICK_MUX_PORT = 0;
const int RIGHT_JOYSTICK_MUX_PORT = 7;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT, false, false);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT, false, false);

typedef struct DroneData
{
  String info;
};

typedef struct JoystickData
{
  Vector3<int> leftJoystick;
  Vector3<int> rightJoystick;
};

DroneData incomingData;
JoystickData outgoingData;
uint8_t selfMACAddress[] {0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34}; 
uint8_t broadcastAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; //Drone Mac = 94:B9:7E:5F:51:40
esp_now_peer_info_t peerInfo;

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
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("ESP_NOW failed to add peer");
    return;
  }
}

void Mode1() 
{

}
void Mode2() 
{

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
  delay(500);
  oled.clearDisplay();

  leftJoystick.Start();
  rightJoystick.Start();
  ptrMode = &Mode1;

  oled.display();
  delay(1000);

  // INPUT_PULLUP button MUST be connected to GND
  // INPUT_PULLDOWN button MUST be connected to VCC
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_TOGGLE, INPUT_PULLDOWN);
  pinMode(BUTTON_TOGGLE2, INPUT_PULLDOWN);

  SetupESPNOW();

  oled.display();
  delay(1000);
 }

// pointerFunction ptrModes[] = {mode1, mode2};
// bool m = false;

void loop() 
{
  unsigned int count = 0;
  oled.clearDisplay();

  // if (digitalRead(BUTTON_A) == 0) ptrMode = mode1;
  // if (digitalRead(BUTTON_B) == 0) ptrMode = mode2;
  // if (digitalRead(BUTTON_C) == 0) ptrMode = mode3; 
  if (digitalRead(BUTTON_TOGGLE) == 1) ptrMode = &Mode1;
  if (digitalRead(BUTTON_TOGGLE2) == 1) ptrMode = &Mode2;

  (*ptrMode)();

  // Assign values
  outgoingData.leftJoystick = leftJoystick.Read();
  outgoingData.rightJoystick = rightJoystick.Read();
  // Send data
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
  if (result == ESP_OK)
  {
    Serial.println("Data sent!");
    oled.println(String(sizeof(outgoingData))+" bytes sent. "+count);
  } else {
    Serial.println("Error sending data.");
    oled.println("Error sending data.");
  }

  oled.setCursor(0, 0);
  oled.println(String("LS: (")+leftJoystick.muxPort+") <"+outgoingData.leftJoystick.x+","+outgoingData.leftJoystick.y+","+outgoingData.leftJoystick.z+">");
  oled.println(String("RS: (")+rightJoystick.muxPort+") <"+outgoingData.rightJoystick.x+","+outgoingData.rightJoystick.y+","+outgoingData.rightJoystick.z+">");
  oled.display();

  delay(20);
}