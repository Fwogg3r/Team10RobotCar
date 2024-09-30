#include <SoftwareSerial.h>

#define BLUETOOTH_BAUD_RATE 9600

#define Rx 52   //sets transmit pin on the bluetooth to the Rx pin on the arduino
#define Tx 51  //sets recieve pin on the bluetooth to the Tx pin on the arduino

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
