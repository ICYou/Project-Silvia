#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Bugtton.h>


#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//pins:
const int HX711_dout_L = 6; //mcu > HX711 no 1 dout pin
const int HX711_sck_L = 7; //mcu > HX711 no 1 sck pin
const int HX711_dout_R = 8; //mcu > HX711 no 2 dout pin
const int HX711_sck_R = 9; //mcu > HX711 no 2 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC scaleL(HX711_dout_L, HX711_sck_L); //HX711 1
HX711_ADC scaleR(HX711_dout_R, HX711_sck_R); //HX711 2

const int calVal_eepromAdress_L = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_R = 4; // eeprom adress for calibration value load cell 2 (4 bytes)


const int serialPrintInterval = 100; //increase value to slow down serial print activity
unsigned long t = 0;


//Define Oled
#define I2C_ADDRESS 0x3C// 0X3C+SA0 - 0x3C or 0x3D
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;


//Define RotaryEncoder
#define encoder0PinA 3
#define encoder0PinB 2
#define encoder0Btn 4
int valRotary,lastValRotary;
int encoder0Pos = 0;

//Buttons
//#define btn1 7
//#define btn2 8
//unsigned long btn2Time = 0;

const uint8_t buttonCount = 2;
const uint8_t buttonPins[buttonCount] = {-22, -23}; // pin5 with pull down resistor
Bugtton buttons(buttonCount, buttonPins, 25);


unsigned long startTime = 0;
unsigned long shotTime = 0;
bool timerStart = false;
bool timerPaused = false;

float totalWeight = 0;


void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");

  float calibValL; // calibration value load cell 1
  float calibValR; // calibration value load cell 2

  calibValL = 1705.31; // uncomment this if you want to set this value in the sketch
  calibValR = 1809.97; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress_R, calibValL); // uncomment this if you want to fetch the value from eeprom
  //EEPROM.get(calVal_eepromAdress_R, calibValR); // uncomment this if you want to fetch the value from eeprom

  scaleL.begin();
  scaleR.begin();
  //scaleL.setReverseOutput();
  //scaleR.setReverseOutput();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte scaleL_rdy = 0;
  byte scaleR_rdy = 0;
  while ((scaleL_rdy + scaleR_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!scaleL_rdy) scaleL_rdy = scaleL.startMultiple(stabilizingtime, _tare);
    if (!scaleR_rdy) scaleR_rdy = scaleR.startMultiple(stabilizingtime, _tare);
  }
  if (scaleL.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (scaleR.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  scaleL.setCalFactor(calibValL); // user set calibration value (float)
  scaleR.setCalFactor(calibValR); // user set calibration value (float)

  //OLED SETUP
  Wire.begin();
  Wire.setClock(400000L);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);
  oled.set2X();


  //SET rotaryEncoder
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder0Btn, INPUT_PULLUP);
  attachInterrupt(0, doEncoder, CHANGE);

  startTime = 0;
  shotTime = 0;
  
  Serial.println("Startup is complete");
}

void loop() {
  buttons.update();

  updateScale(); // this is also where oled is updated
  checkForSerialCommands();  
  handleRotary();
  handleButtons();
  updateTimer();
  sendSerialData();

  if (millis() > t + serialPrintInterval) {
      t = millis();
  }
}

void sendSerialData(){
  if (millis() > t + serialPrintInterval) {
    Serial.println("{g:" + String(totalWeight,1) +", s:" + String((shotTime/1000.0), 1)  +"}");
  }
}
 

void checkForSerialCommands(){
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      tareScale("serial");
    }
  }
}

void updateScale(){
  
  static boolean newDataReady = 0;
  
  // check for new data/start next conversion:
  if (scaleL.update() || scaleR.update()){
    newDataReady = true;
  }

  //get smoothed value from data set
    if (millis() > t + serialPrintInterval) {
      float a = scaleL.getData();
      float b = scaleR.getData();
      if(a+b >= -0.3 && a+b <= 0.2){
        totalWeight = 0.0;
      }else{
        totalWeight = a+b;
      }
      newDataReady = 0;
      updateOled(a, b);

      t = millis();
    
  }
}

bool isTaring(){
  return (scaleL.getTareStatus() == true || scaleR.getTareStatus() == true);
}


bool longpressFlag = 0;

void handleButtons(){
  //Handle Buttons
  if(buttons.rose(0)){
    tareScale("button");
  } 

  bool button1LongPress = buttons.heldUntil(1, 1300);
  
  if(button1LongPress){
    Serial.println("Button longpressed");
    timerStart = false;
    startTime = 0;
    shotTime = 0;
    timerPaused = false;
    longpressFlag = true;
  }else if(buttons.rose(1)){
    Serial.println("button rose");

    if (!longpressFlag)
    {
    if (!timerStart){ //if timer has not started, start
      timerStart = true;
      Serial.println("timer started");
    
    }else if(timerPaused){ // if paused, unpause
      startTime = millis() - shotTime;
      timerPaused = false;
      Serial.println("timer paused");
    
    }else{ // if started, pause
      timerPaused = true;
      Serial.println("timer resumed");
    }
  }
    longpressFlag = false;
  }


}

void updateOled(float a, float b){
  oled.setCursor(0,0);
  
  if(!isTaring()){
    oled.print(String(a) + " ");
    oled.println(String(b) + "   ");
    oled.print(totalWeight, 1);
    oled.println("G     ");
  }else{
    oled.println("taring");
    oled.println("...");
  }
  
  oled.println(String((shotTime/1000.0), 1) + "s   ");
}

void updateTimer(){
   if (timerStart == true){
//      Serial.println("Timer started");
      if (startTime == 0)   // Since 0==false, this is the same as (StartTime != 0)
      {
        startTime = millis();
        Serial.println("startTime" + startTime);
      }
      if(!timerPaused){
        shotTime = millis() - startTime;
      }
  }
}

void handleRotary(){
   //handleEncoder
//    int btn = digitalRead(encoder0Btn);
//  if (btn == 0){
////    btnClicks++;
//    tareScale("rotary encoder button");
//    
//  }
//  if(valRotary>lastValRotary)
//  {
////  Serial.println("  CW");
//    timerStart = true;
////    Serial.print("Timer button pressed");
//      
//  }
//  if(valRotary < lastValRotary) {
//    tareScale("rotary rotate");
////  Serial.println("  CCW");
//  }
  lastValRotary = valRotary;
}

void tareScale(String source){
    Serial.println("tare scale from " + source);

    scaleL.tareNoDelay();
    scaleR.tareNoDelay();
}


void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
  {
  encoder0Pos--;
  }
  else
  {
  encoder0Pos++;
  }
  valRotary = encoder0Pos/2.5;
}
