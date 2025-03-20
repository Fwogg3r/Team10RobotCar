#include "LiquidCrystal_I2C.h"
#include <string.h>
#define MotorPWM_A 46  //left motor
#define MotorPWM_B 44  //right motor
using namespace std;

#define encoderA 2
#define encoderB 3
static volatile int16_t count = 0;
float RPMA = 0;
float RPMB = 0;
float rotation = 3.125;

#define MotorPWM_A 4  // Left motor PWM pin
#define MotorPWM_B 5  // Right motor PWM pin
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36
#define R_S A7
#define M_S A6
#define L_S A5  //7

/*
//----------------------------------Servo-------------------------------
#define echo A0       // Ultrasonic echo pin
#define trigger A1    // Ultrasonic trigger pin
#define servoPin 10    // Pin for controlling the servo
Servo myServo;  // Create a servo object
//----------------------------------------------------------------------
*/

class Motor {
public:

  static void Forward(int speed, float delta_pwm, float max_pwm) {
    analogWrite(MotorPWM_A, constrain(speed + delta_pwm, 0, max_pwm));  // Sets left motor speed
    analogWrite(MotorPWM_B, constrain(speed - delta_pwm, 0, max_pwm));  // Sets right motor speed

    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  }

    static void DepreciatedForward(int speed) {
    analogWrite(MotorPWM_A, speed);  // Sets left motor speed
    analogWrite(MotorPWM_B, speed);  // Sets right motor speed

    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  }

  static void Left(int speed, float delta_pwm) {
    analogWrite(MotorPWM_A, speed + delta_pwm);  // Slow left motor
    analogWrite(MotorPWM_B, speed - delta_pwm);    // Full speed on right motor

    digitalWrite(INA1A, HIGH);
    digitalWrite(INA2A, LOW);

    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  }

    static void Left90(int speed, float delta_pwm) {
    float distance = 0.0;
    while(distance < (3.14 / 2.0) * 3.0)
    {
      Left(speed, delta_pwm);
    }
  }

  static void Right(int speed, float delta_pwm) {
    analogWrite(MotorPWM_A, speed + delta_pwm);      // Full speed on left motor
    analogWrite(MotorPWM_B, speed - delta_pwm);  // Slow right motor

    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
  }

  static void Right90(int speed, float delta_pwm) {
    float distance = 0.0;
    while(distance < (3.14 / 2.0) * 3.0)
    {
      Right(speed, delta_pwm);
    }
  }

  static void Reverse(int speed, float delta_pwm, float max_pwm) {
    analogWrite(MotorPWM_A, constrain(speed + delta_pwm, 0, max_pwm));  // Sets left motor speed
    analogWrite(MotorPWM_B, constrain(speed - delta_pwm, 0, max_pwm));  // Sets right motor speed

    digitalWrite(INA1A, HIGH);
    digitalWrite(INA2A, LOW);

    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
  }

  static void DepreciatedReverse(int speed, int delta_pwm) {
    analogWrite(MotorPWM_A, speed + delta_pwm);  // Sets left motor speed
    analogWrite(MotorPWM_B, speed - delta_pwm);  // Sets right motor speed

    digitalWrite(INA1A, HIGH);
    digitalWrite(INA2A, LOW);

    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
  }

  static void Stop() {
    analogWrite(MotorPWM_A, 0);
    analogWrite(MotorPWM_B, 0);
  }

  static void TankDrive(float LeftPWM, bool LeftFwd, float RightPWM, bool RightFwd, float delta_pwm) {
    if(LeftFwd)
    {
      digitalWrite(INA1A, LOW);
      digitalWrite(INA2A, HIGH);
    }
    else
    {
      digitalWrite(INA1A, HIGH);
      digitalWrite(INA2A, LOW);
    }

    if(RightFwd)
    {
      digitalWrite(INA1B, LOW);
      digitalWrite(INA2B, HIGH);
    }
    else
    {
      digitalWrite(INA1B, HIGH);
      digitalWrite(INA2B, LOW);
    }

    analogWrite(MotorPWM_A, constrain(LeftPWM + delta_pwm, 0, 255));  // Sets left motor speed
    analogWrite(MotorPWM_B, constrain(RightPWM - delta_pwm, 0, 255));  // Sets right motor speed

  }
};

/*static void pathfinding() {
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

  void ISRA()
  {
    countA++;
  }
  void ISRB()
  {
    countB++;
  }
  
  };*/
