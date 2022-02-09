#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiScan.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

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
pointerFunction pfunc;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

QWIICMUX mux;
JOYSTICK joystick;
int joystickZeroOffsetX = 0;
int joystickZeroOffsetY = 0;
// JOYSTICK joystick_1;
// JOYSTICK joystick_2;

//timer
void mode_1() {
  static double t = 0;

  display.clearDisplay();

  display.setCursor(0, 0);
  display.write("Mode 1");

  Serial.print("Time elapsed: ");
  Serial.println(t);
  
  display.setCursor(0,32/2);
  display.print("Time elapsed: ");
  display.print(t);
  display.print("s");
  display.println("");
  
  t += 0.1;
  delay(100);
  yield();
  display.display();
}

void mode_2(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Mode 2: Scanning WiFi");

  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    display.println("0 networks found");
  }
  else {
    for (byte i = 0; i < n; ++i)
    {
      display.print("SSID: ");
      display.println(WiFi.SSID(i));
      display.print("RSSI: ");
      display.println(WiFi.RSSI(i));
      display.print("encryptionType: ");
      display.println(WiFi.encryptionType(i));
      delay(10);
      if (WiFi.isConnected()){
        display.println("You're connected to WiFi");
        break;
      }
    }
  }

  if (WiFi.isConnected()){
    display.clearDisplay();
    Serial.println("You're connected to WiFi");
    display.display();
    delay(500);
    pfunc = &mode_1;
    return;
  }

  display.display();
  //default delay rescan
  delay(5000);
  yield();
  
}

void mode_3(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.write("Mode 3");

  delay(10);
  yield();
  display.display();
}

void calibrateJoystick(int muxPort)
{
    mux.enablePort(muxPort);
    Serial.println(mux.getPort());
    joystickZeroOffsetX = joystick.getHorizontal();
    joystickZeroOffsetY = joystick.getVertical();
    mux.disablePort(muxPort);
}
void muxJoystick(int muxPort) {
  
  mux.enablePort(muxPort);
  Serial.println(mux.getPort());

  int x = joystick.getHorizontal() - joystickZeroOffsetX;
  int y = joystick.getVertical() - joystickZeroOffsetY;

  // fix dir of x axis
  if (x != 514)
  {
    x = 1023 - x;
  }

  Serial.println(String("Joystick ")+muxPort+" X: "+x);
  Serial.println(String("Joystick ")+muxPort+" Y: "+y);
  Serial.println(String("Joystick ")+muxPort+" Button: "+joystick.getButton());

  mux.disablePort(muxPort);
}

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

  pfunc = &mode_1;  

  // while(!joystick.begin())
  // {
  //   Serial.println("Joystick not detected.");
  //   delay(1000);
  // }

  joystick.begin();
  Wire.begin();
  mux.begin();
  muxJoystick(1); // force read potentially bad init readings
  muxJoystick(2); // force read potentially bad init readings

  //calibrateJoystick(1); // calculates joystick zero offset
  //calibrateJoystick(2); // calculates joystick zero offset

  delay(100);
 }



void loop() {

  if (digitalRead(BUTTON_A) == 0) pfunc = &mode_1;
  if (digitalRead(BUTTON_B) == 0) pfunc = &mode_2;
  if (digitalRead(BUTTON_C) == 0) pfunc = &mode_3;
  
  //(*pfunc)();

  muxJoystick(1);
  muxJoystick(2);

  delay(200);
}