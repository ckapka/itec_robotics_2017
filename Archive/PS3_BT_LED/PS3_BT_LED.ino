// for connecting, PS button then plug in arduino with Dongle already in.

#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

int Reverse=0;
int Foreward=0;
int led1=12;
int led2=13;

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}
void loop() {
  Usb.Task();

  
if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      if (PS3.getAnalogButton(R2) > 10 && PS3.getAnalogButton(L2) < 10){
        Foreward = (PS3.getAnalogButton(R2));
        analogWrite(led1,Foreward);       
        delay(10);
     }
     if (PS3.getAnalogButton(L2) > 10 && PS3.getAnalogButton(R2) < 10){
        Reverse = (PS3.getAnalogButton(L2));
        analogWrite(led2,Reverse);
        delay(10);
     }

       Reverse=0;
       Foreward=0;
       analogWrite(led1,0);
       analogWrite(led2,0);
  }
  
}
