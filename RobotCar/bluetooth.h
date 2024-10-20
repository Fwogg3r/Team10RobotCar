#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#define BLUETOOTH_BAUD_RATE 38400

#define Rx 15  //sets transmit pin on the bluetooth to the Rx pin on the arduino
#define Tx 14  //sets recieve pin on the bluetooth to the Tx pin on the arduino

SoftwareSerial bluetooth(Rx,Tx);
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
static String readCommand() {
  String asciiData = "";
  if (bluetooth.available()) {
    //lcd.clear();
    while(bluetooth.available())
    {
      bluetoothData = bluetooth.read();     //reads the bluetooth data
      if( bluetoothData != 13 && bluetoothData != 10)   //ignores certain unwnated values
      {
        asciiData += (char)bluetoothData;
      }
    }
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