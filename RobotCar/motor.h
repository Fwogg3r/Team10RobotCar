#define MotorPWM_A 46 //left motor
#define MotorPWM_B 44 //right motor

#define ENCODER 2
static volatile int16_t count = 0;
float RPM = 0;

//measure the width of the wheel
//measure rotations for size of tape
//pi * r^2

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36


//Method: Forward
//Input: speed - value [0-255]
//Rotate the motor clockwise
void Forward(int speed){
  analogWrite(MotorPWM_A, 5);
  analogWrite(MotorPWM_B, 5);

  //left motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  //right motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}
