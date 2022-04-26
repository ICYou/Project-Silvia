/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/


//#####################TSIC 306 temp sensor##########################
#include "TSIC.h"

TSIC tempSensor1(37); 
TSIC tempSensor2(38); 
float boilerTemp1 = 0;
float boilerTemp2 = 0;

#include <PID_v1.h>

//#define PIN_INPUT 0
#define RELAY_PIN 32

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=40, Ki=0, Kd=20;
//double Kp=4.5, Ki=64, Kd=16;

// Shades of Coffee Gaggia suggested values
//Set 1: P=5.2 , I=28, D=7
//Set 2: P=4.5, I=64, D=16
//Set 3: P=4.8, I=80, D=20


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 1000;
unsigned long windowStartTime;

void setup()
{
  Serial.begin(115200); delay(10);
  
  pinMode(RELAY_PIN, OUTPUT);
  
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 87;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(50, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  //get Temp
  uint16_t temperature1 = 0;
  uint16_t temperature2 = 0;
  boilerTemp1 = 0;
  boilerTemp2 = 0;
  
  if (tempSensor1.getTemperature(&temperature1)) {
    boilerTemp1 = tempSensor1.calc_Celsius(&temperature1);
  }
  if (tempSensor2.getTemperature(&temperature2)) {
    boilerTemp2 = tempSensor2.calc_Celsius(&temperature2);
  }
  bool temp1Read = boilerTemp1 > 0 && boilerTemp1 < 250;
  if (temp1Read){
    Input = boilerTemp1;
  }


  
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime = millis();
    Serial.println("Set new window with output: " + String(Output, 3));
    Serial.println("temperature  1: " + String(boilerTemp1, 1) + " temperature  2: " + String(boilerTemp2, 1));
  }

//  if (Output < 100){
//    if (Output > 0) Output = 100;
//    else Output = 0;
//  }
  
  if (Output > millis() - windowStartTime){ 
    digitalWrite(RELAY_PIN, HIGH);
//    Serial.println("output: " + String(Output, 3));
  }
  else {digitalWrite(RELAY_PIN, LOW);
  }

}
