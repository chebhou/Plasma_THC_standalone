

/*/Pin allocation
   A0 = arcVoltageInput// voltage input from voltage divider in plasma machine.
   A2 = Stepper speed, ie, period between pulses.
   A3 = setPointInput //pot to adjust voltage setpoint.



   D4 = arcOkInput //Arc OK signal from plasma machine.

   D5 = dirPin  // connected to stepper driver.
   D9 = stepPin // connected to stepper driver.

   D6  = HighLedPin  //high led pin
   D8  = lowLedPin   //low led pin
   D10 = arcOkLedPin // independant led pin

   D2 = interruptPin //  pulses from controller.
   D7 = grblDirPin   //  dir  from controller.

  



*/
#include "Arduino.h"
#include <TimerOne.h>

namespace
{
constexpr int arcVoltageInput = A0;
constexpr int pulseIntervalInput = A2;
constexpr int setPointInput = A3;
constexpr int interruptPin = 2;
constexpr int arcOkInput = 4;
constexpr int dirPin = 5;
constexpr int highLedPin = 6;
constexpr int grblDirPin = 7;
constexpr int lowLedPin = 8;
constexpr int stepPin = 9;
constexpr int arcOkLedPin = 10;


constexpr unsigned deadband = 5;
int arcVoltage;
int setPoint;
long pulseInterval = 800;
bool arcOk;
bool cw = true;
bool arcVHigh;
bool arcVLow;

}

void readInputs()
{
  arcVoltage = analogRead(arcVoltageInput);
  setPoint = map(analogRead(setPointInput), 474, 1024, 778, 614);//Will need conversion to arcVoltage range.
  pulseInterval = map(analogRead(pulseIntervalInput), 0, 1024, 0, 10000);//In microseconds.
  arcOk = !digitalRead(arcOkInput);//Switches THC on only when arcOk signal from plasma machine.
}


void setup() {
  pinMode(arcOkInput, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(grblDirPin, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(highLedPin, OUTPUT);
  pinMode(lowLedPin, OUTPUT);
  pinMode(arcOkLedPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), passThrough, FALLING);
  Timer1.initialize();
  //Timer1.pwm(stepPin, 126);

  Serial.begin(115200);
  Serial.println("Stand alone THC starting");
                    
}

void loop() {
  readInputs();
  arcOk = !digitalRead(arcOkInput);//Switches THC on only when arcOk signal from plasma machine.
  arcVHigh = arcVoltage > setPoint + deadband;
  arcVLow = arcVoltage < setPoint - deadband;


    
  if (arcOk) {
    digitalWrite(arcOkLedPin, HIGH);
    
    if (arcVHigh) {
      digitalWrite(dirPin, HIGH);
      digitalWrite(highLedPin, HIGH);
      digitalWrite(lowLedPin, LOW);
      Timer1.pwm(stepPin, 512, pulseInterval);
    }
    else if (arcVLow) {
      digitalWrite(dirPin, LOW);
      digitalWrite(lowLedPin, HIGH);
      digitalWrite(highLedPin, LOW);
      Timer1.pwm(stepPin, 512, pulseInterval);
    }
    else {
      //analogWrite(stepPin, 255);
      digitalWrite(lowLedPin, LOW);
      digitalWrite(highLedPin, LOW);
      Timer1.pwm(stepPin, 1024, pulseInterval);
    }
  }

  else {      

    stop();
    digitalWrite(arcOkLedPin, LOW);
  }

}



void passThrough()
{
  digitalWrite(stepPin, LOW);
  digitalWrite(stepPin, HIGH);
}

void stop()
{
  Timer1.stop();
  Timer1.disablePwm(stepPin);
  digitalWrite(dirPin, digitalRead(grblDirPin));
  digitalWrite(lowLedPin, LOW);
  digitalWrite(highLedPin, LOW);  
}


