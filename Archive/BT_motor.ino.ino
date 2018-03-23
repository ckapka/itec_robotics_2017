//****===================LIBRARIES INCLUDED===============****//
#include <PS3BT.h>
#include <usbhub.h> //why is this included?
#include <math.h>
#include <Servo.h>  //for when we build in servo control

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
//****===================LIBRARIES INCLUDED===============****//^


//****===================ELECTRONIC SETTINGS==============****//
USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
//PS3BT PS3(&Btd); // This will just create the instance
PS3BT PS3(&Btd, 0x00, 0x19, 0x0E, 0x18, 0xBC, 0xC3); // This is the dongles adress, Mat approved wat sity is it in, zip code?
//****===================ELECTRONIC SETTINGS==============****//^


//****===================DEFINITIONS======================****//

//Batery Level Sensing
int const battSensePin = A3;  //pin for sensing battery voltage on a voltage divider. !!Potential for damage to board if divider is not configured correctly. We should get a ~5V zener to clamp the voltage to a safe level if we're worried about it.
int const battLEDPin = 2;
int battWarn1 = 639;  //level for first warning, sized for a 15K/6.8K divider
int battWarn2 = 576;  //level for second warning
int const warnInt1 = 1500;  //speed of first warning blink
int const warnInt2 = 250;  //speed of second warning blink
unsigned long prevMillis = 0;
int battLEDState = LOW;

//Controller settings
int ltAnalogXDeadZone = 10;  //set deazone ranges each axis on each stick. Values will probably be less than 10
float ltAnalogXScale = 0;
int ltAnalogYDeadZone = 10;
int rtAnalogXDeadZone = 10;
int rtAnalogYDeadZone = 10;

//Driving settings
int steeringTrim = 0; //sets trim for steering in degrees, negative for left bias, positive for right bias
float throttleScale = 1.4142; //scales throttle input. Sane values are between .5 and 1.4142 (sqrt(2)). 1.4142 will maximize forward speed, and lowering it will sacrifice top speed to increase steering response
int brakePower = 255;  //sets aggressiveness of braking. values are between 0 (no braking) and 255 (full brakes)

//initialize controller buttons
boolean tri = false;    //Start button in the off position
boolean sqr = false;    //Start button in the off position
boolean circ= false;    //Start button in the off position
boolean cross= false;   //Start button in the off position
//should we initialize all the buttons here? How about the axes?

//Output pins for motor driver input
int const enA=44;   //Lt wheel PWM
int const in1=46;   //Lt wheel forward enable
int const in2=48;   //Lt wheel reverse enable

int const enB=45;   //Rt wheel PWM
int const in4=47;   //Rt wheel forward enable
int const in3=49;   //Rt wheel reverse enable

/*//LED testbed setup on an Arduino Uno
int const enA=6;    //Lt wheel PWM
int const in1=4;   //Lt wheel forward enable
int const in2=5;   //Lt wheel reverse enable

int const enB=9;    //Rt wheel PWM
int const in3=7;   //Rt wheel forward enable
int const in4=8;   //Rt wheel reverse enable

int const yPotPin = A0;
int const xPotPin = A1;*/


int midVal = 127;  //midpoint value for input range, 127 for inputs of 0-255, 511 for inputs of 0-1023, etc. Provide here to simplify potential use with higher resolution inputs.
 

//****===================DEFINITIONS======================****//^


void setup() {
  Serial.begin(115200);
  #if !defined(__MIPSEL__) 
    while(!Serial); //Wait for serial port to connect - used on Leonardo, Teensy, and otehr boards with built in USB CDC serial connection
    //^^^is this necessary for arduino then?
  #endif
  if (Usb.Init() == -1) {  //presumably, this if statement stops the boot if something doesnt start.
    Serial.print(F("\r\nOSC did not start"));  //what is the "F" in the argument here? and what is \nOSC?
    while (1); //halt  //is the purpose of this just to hang the board if the usb device isn't initialized?
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(battLEDPin, OUTPUT);
}


void loop() {
  Usb.Task();
  


//****===================BATTERY WARNING======================****//

/*  int battLevel = analogRead(battSensePin);
  unsigned long currentMillis = millis();
  if(battLevel < battWarn2){
    if(currentMillis - prevMillis >= warnInt2){
      prevMillis = currentMillis;
      if(battLEDState == LOW){
        battLEDState = HIGH;
      }else{
        battLEDState = LOW;
      }
      digitalWrite(battLEDPin,battLEDState);
    }
  }else if(battLevel < battWarn1){
    if(currentMillis - prevMillis >= warnInt1){
      prevMillis = currentMillis;
      if(battLEDState == LOW){
        battLEDState = HIGH;
      }else{
        battLEDState = LOW;
      }
      digitalWrite(battLEDPin,battLEDState);
    }
  }else{
      battLEDState = LOW;
      digitalWrite(battLEDPin,battLEDState);
    }
    */
//****===================BATTERY WARNING======================****//^

  if (PS3.PS3Connected) {

//****===================DRIVE CONTROL======================****//
  
  int ltAnalogX = PS3.getAnalogHat(LeftHatX) - midVal;  // get right stick X and Y positions and center on zero
  int ltAnalogY = map(PS3.getAnalogHat(LeftHatY),0,255,255,0) - midVal;

  /*//LED testbed inputs
  int ltAnalogX = map(analogRead(xPotPin),0,1023,0,255) - midVal;
  int ltAnalogY = map(analogRead(yPotPin),0,1023,0,255) - midVal;*/
  
  Serial.print("Axis values before deadzone: X: ");
  Serial.print(ltAnalogX);
  Serial.print("\tY: ");
  Serial.println(ltAnalogY);

  //apply deadzones
  if(ltAnalogX >=0){
    ltAnalogX = constrain(ltAnalogX,ltAnalogXDeadZone,midVal);
    ltAnalogX = map(ltAnalogX,ltAnalogXDeadZone,midVal,0,midVal);
  }else{
    ltAnalogX = constrain(ltAnalogX,-midVal,-ltAnalogXDeadZone);
    ltAnalogX = map(ltAnalogX,-midVal,-ltAnalogXDeadZone,-midVal,0);
  }
  if(ltAnalogY >=0){
    ltAnalogY = constrain(ltAnalogY,ltAnalogYDeadZone,midVal);
    ltAnalogY = map(ltAnalogY,ltAnalogYDeadZone,midVal,0,midVal);
  }else{
    ltAnalogY = constrain(ltAnalogY,-midVal,-ltAnalogYDeadZone);
    ltAnalogY = map(ltAnalogY,-midVal,-ltAnalogYDeadZone,-midVal,0);
  }  

  Serial.print("Axis values after deadzone: X: ");
  Serial.print(ltAnalogX);
  Serial.print("\tY: ");
  Serial.println(ltAnalogY);
  
  // convert to polar cooridnates
  float steeringTheta = atan2(ltAnalogX,ltAnalogY); //the atan2() func returns a double datatype, which in arduino is the same size as a float. Since I'm feeding the function integers, do I need to convert the integers to floats?
  steeringTheta = steeringTheta + (steeringTrim * .01745); //apply steering trim, converted to radians
  float driveR = sqrt(square(ltAnalogX)+square(ltAnalogY));
  driveR = driveR * throttleScale;
  /*Serial.print("Steering Angle: \t");
  Serial.print(steeringTheta);
  Serial.print("\t Velocity Magnitude: \t");
  Serial.println(driveR);*/
  steeringTheta = steeringTheta + .7854;  // rotate 45 degrees (pi/4)
  
  // convert to cartesian and store as wheel speed
  int LWS = (int) ((driveR * sin(steeringTheta))+.5);
  int RWS = (int) ((driveR * cos(steeringTheta))+.5);
  Serial.print("Wheel Speeds prior to mapping: LT: ");
  Serial.print(LWS);
  Serial.print("\tRT: ");
  Serial.println(RWS);
  
  //set appropriate enable pins for each wheel and map input ranges
  if(LWS>0){
    digitalWrite(in1,HIGH); //set enable pins for forward movement
    digitalWrite(in2,LOW);
    LWS = constrain(LWS,0,midVal); //constrain values to posative side, and clip large values
    LWS = map(LWS,0,midVal,0,255); //expand range to appropriate PWM input range
    analogWrite(enA,LWS);
  }else if(LWS < 0){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    LWS = constrain(LWS,-midVal,0); //constrain values to negative side, and clip large values
    LWS = -LWS;  //make the negative values positive
    LWS = map(LWS,0,midVal,0,255); //expand range to appropriate PWM input range
    analogWrite(enA,LWS);
  }else{
    digitalWrite(in1,HIGH);  //apply brakes
    digitalWrite(in2,HIGH);
    analogWrite(enA,brakePower);
  }
  if(RWS>0){
    digitalWrite(in3,HIGH); //set enable pins for forward movement
    digitalWrite(in4,LOW);
    RWS = constrain(RWS,0,midVal); //constrain values to posative side, and clip large values
    RWS = map(RWS,0,midVal,0,255); //expand range to appropriate PWM input range
    analogWrite(enB,RWS);
  }else if(RWS < 0){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    RWS = constrain(RWS,-midVal,0); //constrain values to negative side, and clip large values
    RWS = -RWS;  //make the negative values positive
    RWS = map(RWS,0,midVal,0,255); //expand range to appropriate PWM input range
    analogWrite(enB,RWS);
  }else {
    digitalWrite(in3,HIGH);
    digitalWrite(in4,HIGH);
    analogWrite(enB,brakePower);
  }
  Serial.print("Wheel Speed PWM to motors: LT: ");
  Serial.print(LWS);
  Serial.print("\tRT: ");
  Serial.println(RWS);
  Serial.println();

  //Function to pause on button press so serial output can be read more carefully
  while(digitalRead(3)==HIGH){
    //do nothing - wait for button press
  }
//****===================DRIVE CONTROL======================****//^
  }
}

