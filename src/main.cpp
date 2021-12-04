#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiScan.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

#if defined(ESP32)
  const int BAUD_RATE = 115200;
  #define BUTTON_A = 15;//GPIO 15 or A8
  #define BUTTON_B = 32;
  #define BUTTON_C = 14;
#else
  const int BAUD_RATE = 9600;
#endif

const int I2C_ADDR = 0x3C; // Can be shared with other I2C devices
int t = 0;

//wifi client
#define wifi_mode = WIFI_STA;

void wifiSetup() {
  char ssid[] = "";//ssid
  char password[] = "";
  wl_status_t status = WiFi.status();
  static String macAddr = WiFi.macAddress();
  int ip = WiFi.broadcastIP();
  int localIP = WiFi.localIP();
  
}

void broadcastWifi()
{

}

void scanWifi() {

  
}

int test() {

}

int test2();// {}

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);//{&test};
pointerFunction pfunc = &loop;
//typedef void (*loop_func)();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial.println("");

  Serial.println(WiFi.status());
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.localIPv6());

  WiFi.mode(WIFI_STA);//Stationary mode
  WiFi.disconnect();//Disconnect incase was connected to AP 
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Time elapsed: ");
  Serial.println(t++);
  delay(1000);

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
}

//pLoop* = function();