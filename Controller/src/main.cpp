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
  Vector3<float> Acceleration;
  Vector3<float> AngularVelocity;
};

typedef struct JoystickData
{
  Vector3<int> leftJoystick;
  Vector3<int> rightJoystick;
};

DroneData incomingData;
JoystickData outgoingData;
uint8_t selfMACAddress[] {0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34}; 
uint8_t broadcastMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; //Drone Mac = 94:B9:7E:5F:51:40
esp_now_peer_info_t peerInfo;

int outgoingSuccessCount = 0;
int outgoingFailCount = 0;
int incomingCount = 0;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
  if (data != NULL)
  {
    Serial.println(++incomingCount);
    memcpy(&incomingData, data, sizeof(data));
  }
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    outgoingSuccessCount++;
  }
  else {
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

void DisplayMode1() 
{
  oled.setCursor(0, 0);
  oled.println("CONTROLLER");
  oled.println(String("LS: (")+leftJoystick.muxPort+") <"+outgoingData.leftJoystick.x+","+outgoingData.leftJoystick.y+","+outgoingData.leftJoystick.z+">");
  oled.println(String("RS: (")+rightJoystick.muxPort+") <"+outgoingData.rightJoystick.x+","+outgoingData.rightJoystick.y+","+outgoingData.rightJoystick.z+">");
  oled.println(String("Sent:")+outgoingSuccessCount+", Received:"+incomingCount);
  oled.display();
}
void DisplayMode2() 
{
  oled.setCursor(0, 0);
  oled.println("DRONE");
  oled.println(String("Acceleration: ")+"<"+incomingData.Acceleration.x+","+incomingData.Acceleration.y+","+incomingData.Acceleration.z+">");
  oled.println(String("Angular Velocity: ")+"<"+incomingData.AngularVelocity.x+","+incomingData.AngularVelocity.y+","+incomingData.AngularVelocity.z+">");
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
  delay(500);
  oled.clearDisplay();

  leftJoystick.Start();
  rightJoystick.Start();
  ptrMode = &DisplayMode1;

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
  if (digitalRead(BUTTON_TOGGLE) == 1) ptrMode = &DisplayMode1;
  if (digitalRead(BUTTON_TOGGLE2) == 1) ptrMode = &DisplayMode2;

  // Assign values
  outgoingData.leftJoystick = leftJoystick.Read();
  outgoingData.rightJoystick = rightJoystick.Read();
  // Send data
  esp_err_t result = esp_now_send(broadcastMACAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
  if (result == ESP_OK)
  {
    Serial.println("Data sent!");
    oled.println(String(sizeof(outgoingData))+" bytes sent. "+count);
  } else {
    Serial.println("Error sending data.");
    oled.println("Error sending data.");
  }

  (*ptrMode)();

  delay(20);
}