#include <SoftwareSerial.h>
#include <LCD_I2C.h>
LCD_I2C lcd(0x27, 16, 2);

#define BLUETOOTH_BAUD_RATE 9600

#define Rx 15   //sets transmit pin on the bluetooth to the Rx pin on the arduino
#define Tx 14  //sets recieve pin on the bluetooth to the Tx pin on the arduino

SoftwareSerial bluetooth(Rx,Tx);
int bluetoothData;

void setup() {
  pinMode(Rx, INPUT);
  pinMode(Tx, OUTPUT);
  bluetooth.begin(BLUETOOTH_BAUD_RATE);
}
String readCommand() {
  String asciiData = "";
  if (bluetooth.available()) {
    lcd.clear();
    while(bluetooth.available())
    {
      bluetoothData = bluetooth.read();
      if( bluetoothData != 13 && bluetoothData != 10)
      {
        asciiData += (char)bluetoothData;
      }
    }
  }
  return asciiData;
}
void loop() {
  // put your main code here, to run repeatedly:

}
