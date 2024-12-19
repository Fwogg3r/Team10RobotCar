#include <string.h>
#define MotorPWM_A 46  //left motor
#define MotorPWM_B 44  //right motor
using namespace std;

#define ENCODER 2
static volatile int16_t count = 0;
float RPMA = 0;
float RPMB = 0;
float rotation = 3.125;

#define MotorPWM_A 5  // Left motor PWM pin
#define MotorPWM_B 4  // Right motor PWM pin
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36
#define R_S A7
#define M_S A6
#define L_S A5  //7

//----------------------------------Servo-------------------------------
#define echo A0       // Ultrasonic echo pin
#define trigger A1    // Ultrasonic trigger pin
#define servoPin 10    // Pin for controlling the servo
Servo myServo;  // Create a servo object
//----------------------------------------------------------------------

class Motor {
public:
  static String DetermineMotorSpeeds(int countA, int countB) {
    RPMA = countA * rotation;
    RPMB = countB * rotation;
    return "RPM A: {}\nRPM B: {}\n", String(RPMA), String(RPMB);
  }

  static void Forward(int speed) {
    analogWrite(MotorPWM_A, speed);  // Sets left motor speed
    analogWrite(MotorPWM_B, speed);  // Sets right motor speed

    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  }

  static void Left(int speed) {
    analogWrite(MotorPWM_A, speed / 2);  // Slow left motor
    analogWrite(MotorPWM_B, speed);      // Full speed on right motor

    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  }

  static void Right(int speed) {
    analogWrite(MotorPWM_A, speed);      // Full speed on left motor
    analogWrite(MotorPWM_B, speed / 2);  // Slow right motor

    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  }

  static void Reverse(int speed) {
    analogWrite(MotorPWM_A, speed);  // Sets left motor speed
    analogWrite(MotorPWM_B, speed);  // Sets right motor speed

    digitalWrite(INA1A, HIGH);
    digitalWrite(INA2A, LOW);

    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
  }

  static void Stop() {
    analogWrite(MotorPWM_A, 0);
    analogWrite(MotorPWM_B, 0);
  }

  static void pathfinding() {
    int leftSensorRead = analogRead(L_S);    // Read left sensor
    int middleSensorRead = analogRead(M_S);  // Read middle sensor
    int rightSensorRead = analogRead(R_S);   // Read right sensor

    const int lineThreshold = 70;  // Line detection threshold
    const int baseSpeed = 150;     // Base speed for motors

    if (middleSensorRead <= leftSensorRead && middleSensorRead <= rightSensorRead && middleSensorRead < lineThreshold) {
      // Middle sensor detects the line
      Forward(baseSpeed);
    } else if (leftSensorRead < middleSensorRead && leftSensorRead < rightSensorRead && leftSensorRead < lineThreshold) {
      // Line is more on the left
      Left(baseSpeed);
    } else if (rightSensorRead < middleSensorRead && rightSensorRead < leftSensorRead && rightSensorRead < lineThreshold) {
      // Line is more on the right
      Right(baseSpeed);
    } else {
      // Stop if no clear line is detected
      Stop();
    }
  }

  static void Drive(int x, int y)
  {
    if(x > 0) //go right
    {
      if(y > 0)
      {
        analogWrite(MotorPWM_A, max((y - x), 0));  // Sets left motor speed
      }
      else
      {
        analogWrite(MotorPWM_A, min((y + x), 510));  // Sets left motor speed
      }
      analogWrite(MotorPWM_B, y);  // Sets right motor speed
    }
    else //go left
    {
      if(y > 0 && Ultrasonic_read() <= 20) //only go forward if ultrasonic pulse is < 20. ADDED SERVO HERE!
      {
        analogWrite(MotorPWM_A, max((y + x), 0));  // Sets left motor speed
      }
      else
      {
        analogWrite(MotorPWM_A, min((y - x), 510));  // Sets left motor speed
      }
      analogWrite(MotorPWM_B, y);  // Sets right motor speed
    }
    if(y > 0) //go forward
    {
      digitalWrite(INA1A, LOW);
      digitalWrite(INA1B, LOW);
      digitalWrite(INA2A, HIGH);
      digitalWrite(INA2B, HIGH);
    }
    else //go backward
    {
      digitalWrite(INA1A, HIGH);
      digitalWrite(INA1B, HIGH);
      digitalWrite(INA2A, LOW);
      digitalWrite(INA2B, LOW);
    }
  }

  void RemoteMode(string command) {
    if (command.equalsIgnoreCase("remoteModeEnd")) {
      remoteMode = false;
    } else {
      int x = 0;
      int y = 0 if (command[0] == 'Y') {
        command = command.substr(1, command.length());  //TODO: get both at the same time from sender.
        y = stoi(command);
      }
      if (command[0] == 'X') {
        command = command.substr(1, command.length());
        x = stoi(command);
      }
      Drive(x, y);
    }
  }
  };
