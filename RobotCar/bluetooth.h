#include <SoftwareSerial.h>


#define LED_GREEN 12
#define LED_RED 13

void setup() {
  // Set up the LEDs as output
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // Start Serial Communication for both the monitor and the HC-05
  Serial.begin(38400);  // Monitor
  Serial1.begin(38400); // HC-05 
  
  // make sure LEDs are off
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  
  Serial.println("Setup complete, waiting for commands..."); //Open monitor and watch for all this stuff
  // Monitor was my best friend trying to figure this **** out
}

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n'); // Read command from HC-05 using serial 1 which is pins 18 and 19
    command.trim(); // Remove any leading/trailing spaces, etc. This was an issue when I was sending the
    //color codes. I made them "red, green, blue" on my phone and serial monitor would show they were
    //received but not understood. I just changed these to right and left
    
    Serial.print("Received: ");
    Serial.println(command);
    
    // Handle the received command
    if (command.equalsIgnoreCase("right")) {
      Serial.println("Turning on green LED");
      turnOnLED(LED_GREEN);
    } else if (command.equalsIgnoreCase("left")) {
      Serial.println("Turning on red LED");
      turnOnLED(LED_RED);
    } else {
      Serial.println("Unknown command"); // for when you push something that the board has no idea what it is
    }
  }
}
//This is just for testing below, we can change this whenever
void turnOnLED(int ledPin) {
 digitalWrite(ledPin, HIGH);  // Turn on the LED
  delay(500);                 // Wait for .5 seconds
  digitalWrite(ledPin, LOW);   // Turn off the LED
 delay(500);                 // Wait for .5 seconds
  digitalWrite(ledPin, HIGH);  // Turn on the LED
  delay(500);                 // Wait for .5 seconds
  digitalWrite(ledPin, LOW);   // Turn off the LED
  delay(500);                 // Wait for .5 seconds
  digitalWrite(ledPin, HIGH);  // Turn on the LED
  delay(500);                 // Wait for .5 seconds
  digitalWrite(ledPin, LOW);   // Turn off the LED
  delay(500);                 // Wait for .5 seconds
}
