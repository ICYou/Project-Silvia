#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
   

//#include <Adafruit_MAX31865.h>
//#include <Bugtton.h>

//#####################TSIC 306 temp sensor##########################
#include "TSIC.h"

TSIC tempSensor(37); 
TSIC tempSensor2(38); 
float boilerTemp = 0;
float boilerTemp2 = 0;


//#####################HX711 Weight##########################
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

//const int calVal_eepromAdress_L = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
//const int calVal_eepromAdress_R = 4; // eeprom adress for calibration value load cell 2 (4 bytes)

float totalWeight = 0;



//##################### Oled ##########################
//Small oled 128x64
#define I2C_ADDRESS 0x3C// 0X3C+SA0 - 0x3C or 0x3D 

//Large oled 128x128
//#define I2C_ADDRESS 0x78// 0X3C+SA0 - 0x3C or 0x3D 

// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;


//Buttons
#define btn1 0
#define btn2 1
unsigned long btn2Time = 0;



unsigned long startTime = 0;
unsigned long shotTime = 0;
bool timerStart = false;
bool timerPaused = false;




//#####################TEMPERATURE##########################
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(33, 34, 35, 36);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
//#define RREF      430.0

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
//#define RNOMINAL  100.0
//#define RNOMINAL  103.7

//float boilerTemp = 0;
//#####################TEMPERATURE END##########################



//#####################Other stuff##########################
const int interval50 = 200; //increase value to slow down serial print activity
unsigned long t = 0;
unsigned long loopTime = 0;

//#####################SSR Pins##########################
#define pumpSSR 30
#define solenoidSSR 31
#define boilerSSR 32



void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");
  
  pinMode(4, OUTPUT); 
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);

  float calibValL; // calibration value load cell 1
  float calibValR; // calibration value load cell 2

  calibValL = 1705.31; // uncomment this if you want to set this value in the sketch
  calibValR = 1779.86; // uncomment this if you want to set this value in the sketch
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

  //set Max31865 pt100 temp sensor
//  thermo.begin(MAX31865_2WIRE); 


  startTime = 0;
  shotTime = 0;
  
  Serial.println("Startup is complete");
}

void loop() {
  loopTime = millis();
//  buttons.update();

  updateScale(); // this is also where oled is updated
  checkForSerialCommands();  
//  handleRotary();
  handleButtons();
  updateTimer();
  

  if (loopTime > t + interval50) {
    readTemperature();
    sendSerialData();
    t = millis();
  }
}

void readTemperature(){
  uint16_t temperature = 0;
  uint16_t temperature2 = 0;
  boilerTemp = 0;
  boilerTemp2 = 0;
  if (tempSensor.getTemperature(&temperature)) {
    boilerTemp = tempSensor.calc_Celsius(&temperature);
  }
  if (tempSensor2.getTemperature(&temperature2)) {
    boilerTemp2 = tempSensor2.calc_Celsius(&temperature2);
  }
}



void sendSerialData(){
//  if (millis() > t + interval50) {
    Serial.println("{\"g\":" + String(totalWeight,1) +", \"s\":" + String((shotTime/1000.0), 1)  + ", \"t\":" + String((boilerTemp), 1) + ", \"t2\":" + String((boilerTemp2), 1)  +"}");
//  }
}
 

void checkForSerialCommands(){
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      tareScale("serial");
    }else if(inByte == 's'){
      startTimer();
    }else if(inByte == 'p'){
      stopTimer();
    }else if(inByte == 'r'){
      resetTimer();
    }
  }
}

void startStopTimer(){
  if(!timerStart){
    startTimer(); 
  }else{
    stopTimer();
  }
}

void startTimer(){
  timerStart = true;
  startTime = millis() - shotTime;
}

void stopTimer(){
  timerStart = false;
}

void resetTimer(){
  timerStart = false;
  startTime = 0;
  shotTime = 0;
}

void updateTimer(){
  if (timerStart){
    shotTime = millis() - startTime;
  }
}

void updateScale(){
  static boolean newDataReady = 0;
  // check for new data/start next conversion:
//  Serial.println("newDataREADY L: " + String(scaleL.update()) );
//  Serial.println("newDataREADY R: " + String(scaleR.update()) );
//  Serial.println("calibval L: "+ String(scaleL.getCalFactor()));
//  Serial.println("calibval R: "+ String(scaleR.getCalFactor())); 
  if (scaleL.update() || scaleR.update()){
    newDataReady = true;
  }
  //get smoothed value from data set
  if (newDataReady) {
    float a = scaleL.getData();
    float b = scaleR.getData();

    calculateWeight(a,b);
    
    newDataReady = 0;
    updateOled(a, b);

//      t = millis();
    
  }
}

const int weightLen = 20;
float weightVals[weightLen];
int weightIndex = 0;

void calculateWeight(float a, float b){

  
//  if (weightindex >= weighLen)
//    weightIndex = 0;
//
//  float currentWeight = a+b;
//  float weightAverage = average(weightVals, weightLen); //get average before new value is added
//  
//  weightVals[weightIndex] = currentWeight;
//
//  if(currentWeight - weightAverage > 0.25)
//    clearList(weightVals, weightLen);

  
  if(a+b >= -0.3 && a+b <= 0.2){
    totalWeight = 0.0;
  }else{
    totalWeight = a+b;
  }

//  weightIndex++;
}

float average (float * array, int len)  // assuming array is int.
{
  float sum = 0.0;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

float clearList (float * array, int len)  // assuming array is int.
{
  for (int i = 0 ; i < len ; i++)
    array [i] = 0;
}


bool isTaring(){
  return (scaleL.getTareStatus() == true || scaleR.getTareStatus() == true);
}


//void readTemp(){
//  thermo.readRTD();
//  boilerTemp = thermo.temperature(RNOMINAL, RREF);
//
//    uint8_t fault = thermo.readFault();
//  if (fault) {
//    Serial.print("Fault 0x"); Serial.println(fault, HEX);
//    if (fault & MAX31865_FAULT_HIGHTHRESH) {
//      Serial.println("RTD High Threshold"); 
//    }
//    if (fault & MAX31865_FAULT_LOWTHRESH) {
//      Serial.println("RTD Low Threshold"); 
//    }
//    if (fault & MAX31865_FAULT_REFINLOW) {
//      Serial.println("REFIN- > 0.85 x Bias"); 
//    }
//    if (fault & MAX31865_FAULT_REFINHIGH) {
//      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
//    }
//    if (fault & MAX31865_FAULT_RTDINLOW) {
//      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
//    }
//    if (fault & MAX31865_FAULT_OVUV) {
//      Serial.println("Under/Over voltage"); 
//    }
//    thermo.clearFault();
//  }
//}


bool longpressFlag = 0;

void handleButtons(){
  int btn1State = digitalRead(btn1);
  int btn2State = digitalRead(btn2);
  if (btn1State == HIGH) {
//    Serial.println("tare scale by button");
    scaleL.tareNoDelay();
    scaleR.tareNoDelay();
  } 
    
  if (btn2State == HIGH){ //button is pressed    
    if(btn2Time == 0){ //save initial press time
      btn2Time = millis();
    }else if((millis() - btn2Time) > 1200){ //handle 1.2s press
//      Serial.println("button longpress");
      stopTimer();
      resetTimer();
    }
  }else{ //buttonState is low
    if(btn2Time > 0 && millis() - btn2Time <= 1200){ //handle button released after long press - Reset button
//      Serial.println("button released");
      startStopTimer();
      }
      btn2Time = 0;
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


void tareScale(String source){
    Serial.println("tare scale from " + source);

    scaleL.tareNoDelay();
    scaleR.tareNoDelay();
}
