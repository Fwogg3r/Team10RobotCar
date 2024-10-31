#define MotorPWM_A 46 //left motor
#define MotorPWM_B 44 //right motor

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

void setup() {
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
}

//Method: Forward
//Input: speed - value [0-255]
//Rotate the motor clockwise
void Forward(int speed){
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  //left motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  //right motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}