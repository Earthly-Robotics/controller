#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C //nodig voor oled
#include <Wire.h>
#endif

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // oled scherm definitie

/* WiFi network name and password */
const char * ssid = "Earthly_Robotics_AP";// dit is het naam van het gebruikte wifi netwerk
const char * pwd = "brobotica";//dit is het wachtwoord van het gebruikte wifi netwerk


const char * udpAddress = "192.168.50.1"; // het ip adres waar udp berichten naar toe wordt gestuurd
const int udpPort = 8080; // de poort waar udp berichten naar gestuurd worden.
const int buttonPin1 = 19;// knop 1 Profiel knop
const int buttonPin2 = 5;//knop 2 Nood knop
const int buttonPin3 = 17;//knop 3 Variabele knop 
int L_JoyStick_X = 35; // Analog Pin  X
int L_JoyStick_Y = 34; // // Analog Pin  Y
int L_JoyStick_button = 32; // IO Pin 
int R_JoyStick_X = 14; // Analog Pin  X
int R_JoyStick_Y = 27; // // Analog Pin  Y
int R_JoyStick_button = 12; // IO Pin 
int profile=0;//integer voor profile switch
int i;
int j=0;
int LJ_x, LJ_y,RJ_x,RJ_y, button;


const int DEBOUNCE_DELAY = 10;   // the debounce time; increase if the output flickers

// Variables will change:
int lastSteadyState = LOW;       // the previous steady state from the input pin
int lastFlickerableState = LOW;  // the previous flickerable state from the input pin
int currentState;                // the current reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

const int DEBOUNCE_DELAY3 = 40;   // the debounce time; increase if the output flickers

// Variables will change:
int lastSteadyState3 = LOW;       // the previous steady state from the input pin
int lastFlickerableState3 = LOW;  // the previous flickerable state from the input pin
int currentState3;                // the current reading from the input pin

unsigned long lastDebounceTime3 = 0;  // the last time the output pin was toggled


//create UDP instance
WiFiUDP udp;

void setup(){
  Serial.begin(115200);               // start de seriele communicatie
  pinMode(buttonPin1, INPUT_PULLDOWN);
  pinMode(buttonPin2, INPUT_PULLDOWN);
  pinMode(buttonPin3, INPUT_PULLDOWN);
  pinMode(L_JoyStick_X, INPUT);
  pinMode(L_JoyStick_Y, INPUT);
  pinMode(L_JoyStick_button, INPUT_PULLUP);
  pinMode(R_JoyStick_X, INPUT);
  pinMode(R_JoyStick_Y, INPUT);
  pinMode(R_JoyStick_button, INPUT_PULLUP);
  u8g2.begin();//begin voor OLED scherm
 
}
void connect(){
  //Connect to the WiFi network
  Serial.println("");// spacer
  Serial.print("Connecting");
  WiFi.begin(ssid, pwd);//maak de wifi verbinding.
  i=0;
  
  while ((WiFi.status() != WL_CONNECTED) && (i<20)) {// hier wordt gekeken of er verbinding is met het wifi en dat i nog kleiner is dan 10
    delay(500);
    Serial.print(".");
    i++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  }

}

void u8g2_prepare(void) {   //benodigheden voor het scherm.
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void showonoled() {
  u8g2.clearBuffer();
  u8g2_prepare();
  
  u8g2.drawStr(0,36, "Profile:");
  u8g2.setCursor(70, 36);//selecteer de positie
  u8g2.print(profile);//display de waarde van het profile

  u8g2.sendBuffer();  
}

void Startup() {
  u8g2.clearBuffer();
  u8g2_prepare();
  u8g2.drawStr(0,36, "Connecting....");
  u8g2.sendBuffer();  
}


void loop(){

  while (WiFi.status() != WL_CONNECTED){
    
       u8g2.firstPage();  
    do {
      u8g2_prepare();
      Startup();
      }
      while( u8g2.nextPage() );
    
   connect();
  }
  
  LJ_x = analogRead(L_JoyStick_X); //  X
  LJ_y = analogRead(L_JoyStick_Y); //  Y
  LJ_x = map(LJ_x, 0, 4095, -2047, 2047);
  LJ_y = map(LJ_y, 0, 4095, -2047, 2047);

  RJ_x = analogRead(R_JoyStick_X); //  X
  RJ_y = analogRead(R_JoyStick_Y); //  Y
  RJ_x = map(RJ_x, 0, 4095, -2047, 2047);//1023
  RJ_y = map(RJ_y, 0, 4095, -2047, 2047);
  String LJ= "{\"MT\":\"LJ\", \"p\" :" + String(profile) + ", \"x\":" + String(LJ_x) +  ", \"y\" :" + String(LJ_y) + "}";//"{\"MessageType\":\"LeftJoystick\", \"profile\" : profile, \"x-as\":" + String(x) +  ", \"y-as\" :" + String(y) + "}";
  String RJ= "{\"MT\":\"RJ\", \"p\" :" + String(profile) + ", \"x\":" + String(RJ_x) +  ", \"y\" :" + String(RJ_y) + "}";//"{\"MessageType\":\"RightJoystick\", \"profile\" : profile, \"x-as\":" + String(x) +  ", \"y-as\" :" + String(y) + "}";
  sendMessage(LJ);
  sendMessage(RJ);

  if (digitalRead(L_JoyStick_button)){
    String LJB = "{\"MT\":\"LJB\", \"p\" :" + String(profile) + "}";       //"{\"MessageType\":\"LeftJoystickbutton\", \"Profile\" : profile}";
    sendMessage(LJB);
  }
  if (digitalRead(R_JoyStick_button)){
    String RJB = "{\"MT\":\"RJB\", \"p\" :" + String(profile) + "}";       //"{\"MessageType\":\"LeftJoystickbutton\", \"Profile\" : profile}";
    sendMessage(RJB);
  }

  currentState = digitalRead(buttonPin1);
  if (currentState != lastFlickerableState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the the last flickerable state
    lastFlickerableState = currentState;
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (lastSteadyState == HIGH && currentState == LOW){
      Serial.println("The button is released");
      profile++;
    }
    else if (lastSteadyState == LOW && currentState == HIGH)
      Serial.println("The button is pressed");

    // save the the last steady state
    lastSteadyState = currentState;
  
    
    
    if (profile>1){
      profile=0;
    }
    String Button1 = "{\"MT\":\"PB\", \"p\" :" + String(profile) + "}";           //"{\"MessageType\":\"ProfileButton\", \"Profile\" : profile}";
    sendMessage(Button1);
    
  }

  if (digitalRead(buttonPin2)){
    String Button2 = "{\"MT\":\"EB\", \"p\" :" + String(profile) + "}";                 //"{\"MessageType\":\"EmergencyButton\" \"Profile\" : profile}";
    sendMessage(Button2);
  }

  currentState3 = digitalRead(buttonPin3);
  if (currentState3 != lastFlickerableState3) {
    // reset the debouncing timer
    lastDebounceTime3 = millis();
    // save the the last flickerable state
    lastFlickerableState3 = currentState3;
  }
  if ((millis() - lastDebounceTime3) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (lastSteadyState3 == HIGH && currentState3 == LOW){
      Serial.println("The button is released");
         String Button3 = "{\"MT\":\"VB\", \"p\" :" + String(profile) + "}";                 //"{\"MessageType\":\"VariableButton\" \ "Profile\" : profile}";
    sendMessage(Button3);
    }
    else if (lastSteadyState3 == LOW && currentState3 == HIGH)
      Serial.println("The button is pressed");

    // save the the last steady state
    lastSteadyState3 = currentState3;
 
  }
   
  u8g2.firstPage();  
  do {
    u8g2_prepare();
    showonoled();
    } 
    while( u8g2.nextPage() );

 
}

void sendMessage(String message){
  DynamicJsonDocument doc(100);
  deserializeJson(doc, message);
  udp.beginPacket(udpAddress, udpPort);
  serializeJson(doc, udp);
  udp.endPacket();
  doc.clear();
}
