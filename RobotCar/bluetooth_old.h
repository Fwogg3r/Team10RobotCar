#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

int bluetoothData;

/*void setup() {
  pinMode(Rx, INPUT);     //sets Rx as an input
  pinMode(Tx, OUTPUT);    //sets Tx as an output
  bluetooth.begin(BLUETOOTH_BAUD_RATE);     //starts bluetooth communication
}*/
/*String readCommand()
{
  if (bluetooth.available()>0)    //sends data only after recieved data
  { incomingByte = bluetooth.read();    //reads the incoming byte(s)

    bluetooth.print("check");       //confirmation that it was recieved
    bluetooth.println(incomingByte, DEC);
  }
}*/
static String readCommand(SoftwareSerial &bluetooth, LiquidCrystal_I2C lcd) {
  String asciiData = "";
  if (bluetooth.available()) {
    //lcd.print("Bluetooth DETECTED ON bluetooth.h...");
    //lcd.clear();
    while(bluetooth.available())
    {
      lcd.print("Bluetooth Reading...");
      bluetoothData = bluetooth.read();     //reads the bluetooth data
      if( bluetoothData != 13 && bluetoothData != 10)   //ignores certain unwnated values
      {
        asciiData += (char)bluetoothData;
      }
    }
  }
  if(asciiData != "")
  {
    lcd.clear();
    lcd.print(asciiData);
  }
  return asciiData;
}

/*void loop() 
{
  if (bluetooth.available()>0)    //sends data only after recieved data
  { incomingByte = bluetooth.read();    //reads the incoming byte(s)

    bluetooth.print("check");       //confirmation that it was recieved
    bluetooth.println(incomingByte, DEC);
  }
}*/
#endif