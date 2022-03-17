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

const int LEFT_JOYSTICK_MUX_PORT = 1;
const int RIGHT_JOYSTICK_MUX_PORT = 0;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT);


// void calibrateJoystick(int muxPort)
// {
//     mux.enablePort(muxPort);
//     Serial.println(mux.getPort());
//     joystickZeroOffsetX = joystick.getHorizontal();
//     joystickZeroOffsetY = joystick.getVertical();
//     mux.disablePort(muxPort);
// }

// void calibrateJoystick(MuxJoystick *joystick) //calculate zero offset when centered
//   {
//     //calibrate
//     // Serial.println("(Joystick calibration starting in 4 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     // Serial.println("(Joystick calibration starting in 3 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     // Serial.println("(Joystick calibration starting in 2 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     // Serial.println("(Joystick calibration starting in 1 seconds...)");//joysticks[muxPort].nameID);
//     // delay(1000);
//     Serial.println("Calibrating...");
//   }

void mode0()
{

}

void mode1() 
{

}

void mode2()
{

}

void mode3()
{

}

typedef struct Data
{
  Vector3<int> leftJoystick;
  Vector3<int> rightJoystick;
};
Data dataSending;
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

  oled.display();
  delay(1000);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  ptrMode = &mode0;  

  SetupESPNOW();

  oled.display();
  delay(1000);
 }

void loop() 
{
  oled.clearDisplay();
  if (digitalRead(BUTTON_A) == 0) ptrMode = mode1;
  if (digitalRead(BUTTON_B) == 0) ptrMode = mode2;
  if (digitalRead(BUTTON_C) == 0) ptrMode = mode3;
  
  (*ptrMode)();
  
  // Assign values
  dataSending.leftJoystick = leftJoystick.Read();
  dataSending.rightJoystick = rightJoystick.Read();
  // Send data
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataSending, sizeof(dataSending));
  if (result == ESP_OK)
  {
    Serial.println("Data sent!");
  } else {
    Serial.println("Error sending data.");
  }
  
  oled.setCursor(0, 0);
  oled.println(String("Joystick_")+leftJoystick.muxPort+" <"+leftJoystick.Read().x+","+leftJoystick.Read().y+","+leftJoystick.Read().z+">");
  oled.println(String("Joystick_")+rightJoystick.muxPort+" <"+rightJoystick.Read().x+","+rightJoystick.Read().y+","+rightJoystick.Read().z+">");
  oled.display();
  
  delay(20);
}