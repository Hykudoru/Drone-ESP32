#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiScan.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int I2C_ADDR = 0x3C; // Can be shared with other I2C devices
const int BUTTON_A = 15;//GPIO 15 or A8
const int BUTTON_B = 32;
const int BUTTON_C = 14;

#if defined(ESP32)
  const int BAUD_RATE = 115200;
#else
  const int BAUD_RATE = 9600;
#endif

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction pfunc = &loop;
//typedef void (*loop_func)();

int t = 0;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial.println("");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Setup...");
  display.display();
  delay(500);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);//Stationary mode
  WiFi.disconnect();//Disconnect incase was connected to AP  
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Time elapsed: ");
  Serial.println(t);
  
  display.clearDisplay();
  display.setCursor(0,32/2);
  display.print("Time elapsed: ");
  display.print(t);
  display.print("s");
  yield();
  display.display();
  delay(1000);
  t++;

  /*
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("0 networks found");
  }
  else {
    for (byte i = 0; i < n; ++i)
    {
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID(i));
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI(i));
      Serial.print("encryptionType: ");
      Serial.println(WiFi.encryptionType(i));
      delay(10);
      if (WiFi.isConnected()){
        Serial.println("You're connected to WiFi");
        break;
      }
    }
  }

  if (WiFi.isConnected()){
    Serial.println("You're connected to WiFi");
    //pfunc = someNewProcedure;
    return;
  }

  //default delay rescan
  delay(5000);
  */
}

//pLoop* = function();
