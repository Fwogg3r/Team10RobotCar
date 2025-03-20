#include "motor.h"
#include "lcd.h"
//#include "led.h"
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include "string.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#include <SoftwareSerial.h>

//BT BEGIN
SoftwareSerial BTSerial(10, 11);  // (TX, RX) pins on HC05
int BT_PWM = 0;
//BT END

//BATTERY BEGIN
float battery_voltage = 0;
#define batteryPin A0
//BATTERY END

//LED BEGIN
Adafruit_NeoPixel RGB(1, 38, NEO_GRB + NEO_KHZ800);

void RGB_RED() {
  RGB.setPixelColor(0, RGB.Color(255, 0, 0));
  RGB.show();
}
void RGB_GREEN() {
  RGB.setPixelColor(0, RGB.Color(0, 255, 0));
  RGB.show();
}
void RGB_BLUE() {
  RGB.setPixelColor(0, RGB.Color(0, 0, 255));
  RGB.show();
}
void RGB_WHITE() {
  RGB.setPixelColor(0, RGB.Color(255, 255, 255));
  RGB.show();
}
void RGB_OFF() {
  RGB.setPixelColor(0, RGB.Color(0, 0, 0));
  RGB.show();
}
//LED END

//MOTOR_INFORMATION BEGIN
#define MotorPWM_A 4  //left motor
#define MotorPWM_B 5  //right motor
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36
#define potentiometerPin A1

unsigned long initialMillis;
unsigned long encoderMicros;
unsigned long encoderInterval = 100;
float distance = 0.0;
float squareSize = 12;  //in inches
int MotorPWM = 100;
int MAX_PWM = 250;
float RPM_A = 0;
float RPM_B = 0;
float linearSpeed = 0;
float speed = 0;
static volatile int16_t count_A = 0;  // Encoder A
static volatile int16_t count_B = 0;  // Encoder B
//PID Params
float error = 0, past_error = 0, i_error = 0, d_error = 0, KP = 5, KI = 0, KD = 0;  //KP Base 20
float Delta_PWM = 0;
MPU6050 mpu(Wire);
float target_heading = 0;
//GRIPPER BEGIN
Servo servo;
short int SERVO_OPEN_POS = 0;
short int SERVO_CLOSE_POS = 35;
short int servoPos = SERVO_OPEN_POS;
short int GRIPPER_LOW = 989;
short int GRIPPER_HIGH = 1977;

void gripperOpen()
{
  servoPos = SERVO_OPEN_POS;
  servo.write(SERVO_OPEN_POS);
}

void gripperClose()
{
  servoPos = SERVO_CLOSE_POS;
  servo.write(SERVO_CLOSE_POS);
}


//GRIPPER END


void calculateErrors() {
  past_error = error;
  error = mpu.getAngleZ() - target_heading;
  i_error = i_error + error;
  d_error = error - past_error;
  Delta_PWM = KP * error + KI * i_error + KD * d_error;
}

void ISR1() {
  count_A++;
}
void ISR2() {
  count_B++;
}

void RPM() {
  if (micros() - encoderMicros >= encoderInterval) {
    mpu.update();
    calculateErrors();
    unsigned long newTime = micros();
    //Serial.print(distance);
    //Serial.print("\n");
    RPM_A = count_A * 3.125;
    RPM_B = count_B * 3.125;
    linearSpeed = ((RPM_A + RPM_B) / 2) * 0.134;
    speed += abs(mpu.getAccX() * 9.81) * (micros() - encoderMicros) / 1000000.0;
    distance += (linearSpeed * ((micros() - encoderMicros)) / 1000000.0);
    count_A = 0;
    count_B = 0;
    encoderMicros = newTime;
    speed = 0;
    //linearSpeed = 0;
  }
}

void MPUInit() {
  //Initialize PMU6050
  Wire.begin();
  byte status = mpu.begin(0, 0);  //gyro and accel sensitivity range 0-3
  //Serial.print(F("MPU6050 status: "));
  //Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050
  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
  mpu.calcOffsets();  // gyro and accelerometer
  //Serial.println("Done!\n");
  //end initiation
}
//MOTOR_INFORMATION_END

//LCD BEGIN
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lcdMillis;

void displayInfo() {
  if (millis() - lcdMillis >= 200) {
    lcdMillis = millis();
    if (error < 10 && error > -10)  //straight
    {
      RGB_WHITE();
      LCD::DisplaySpeedAndDirection(lcd, linearSpeed, "FORWARD", battery_voltage, distance);
      MAX_PWM = 250;
    } else if (error < -10)  //left
    {
      RGB_GREEN();
      LCD::DisplaySpeedAndDirection(lcd, linearSpeed, "LEFT", battery_voltage, distance);
      MAX_PWM = 125;
    } else if (error > 10)  //right
    {
      RGB_BLUE();
      LCD::DisplaySpeedAndDirection(lcd, linearSpeed, "RIGHT", battery_voltage, distance);
      MAX_PWM = 125;
    }
  }
}
//LCD END

//REMOTE CONTROL BEGIN
#define Throttle pulseIn(7, HIGH)  //read the pulse width of channels 1, 3, 5 respectively
#define Aileron pulseIn(6, HIGH)
#define Switch pulseIn(8, HIGH)
#define GripperSet pulseIn(9, HIGH)
short int THROTTLE_LOW = 969; //NEUTRAL: 1387-1394 | LOW: 969 | HIGH: 1958
short int THROTTLE_NEUTRAL = 1387;
short int THROTTLE_HIGH = 1958;
short int AILERON_LOW = 1246; //NEUTRAL: 1497-1504 | LOW: 1246-1252 | HIGH: 1750
short int AILERON_NEUTRAL = 1497;
short int AILERON_HIGH = 1750;
short int SWITCH_LOW = 989; //NEUTRAL: 1490-1486 | LOW: 989 | HIGH: 1957
short int SWITCH_NEUTRAL = 1490;
short int SWITCH_HIGH = 1957; // LOW: 989 | HIGH: 1978
//gripper defined in gripper params up high

//REMOTE CONTROL END

//Key Function Begin
void update() {
  if (millis() - initialMillis <= 2000) {
    //Serial.println("FORWARD");
    RGB_WHITE();
    //Motor::Forward(MotorPWM, Delta_PWM);
    //tryLCDDisplay("FORWARD");
  } else if (millis() - initialMillis <= 4000) {
    //Serial.println("LEFT");
    RGB_GREEN();
    Motor::Left(MotorPWM, Delta_PWM);
    //tryLCDDisplay("LEFT");
  } else if (millis() - initialMillis <= 6000) {
    //Serial.println("RIGHT");
    RGB_BLUE();
    Motor::Right(MotorPWM, Delta_PWM);
    //tryLCDDisplay("RIGHT");
  } else if (millis() - initialMillis <= 8000) {
    //Serial.println("REVERSE");
    RGB_RED();
    //Motor::Reverse(MotorPWM, Delta_PWM);
    //tryLCDDisplay("REVERSE");
  } else if (millis() - initialMillis >= 10000) {
    lcdMillis = millis();
    initialMillis = millis();
  }
}

void square() {
  if (distance <= squareSize) {
    //Serial.print(target_heading);
    //Serial.print("\n");
    Motor::Forward(MotorPWM, Delta_PWM, MAX_PWM);
  } else if (distance > squareSize && target_heading < 360 && target_heading > -360) {
    distance = 0;
    target_heading += 90;
    /*distance = 0.0;
    KP = 1;
    MotorPWM = 75;
    while (distance < (3.14 / 2.0) * 3.0) { //3.0 default final num
      RPM();
      Serial.println("RIGHT");
      RGB_BLUE();
      Motor::Right(MotorPWM, Delta_PWM);
      tryLCDDisplay("RIGHT");
    }
    distance = 0.0;*/
  } else if (target_heading >= 360 || target_heading <= -360) {
    target_heading = 0;
  }
}

void CalculateDriveMode() {
}

void ArcadeRCDrive(bool UseGearShifter) {
  target_heading = mpu.getAngleZ();
  if (Aileron < AILERON_NEUTRAL - 100 || Aileron > AILERON_NEUTRAL + 100) {
      Delta_PWM = map(Aileron, AILERON_HIGH, AILERON_LOW, -128, 128);
    } else Delta_PWM = 0;
  //Delta_PWM = map(Aileron, 1870, 870, -128, 128);
  if (UseGearShifter) {
    MotorPWM = map(Throttle, THROTTLE_LOW, THROTTLE_HIGH, 0, 255);
    if (Switch >= 1000)  //Forward Gear
    {
      MAX_PWM = 255;
      Motor::Reverse(MotorPWM, Delta_PWM, MAX_PWM);
    } 
    else if (Switch < SWITCH_HIGH - 100 && Switch > SWITCH_LOW + 100)  //Stop Gear
    {
      MotorPWM = 0;
      MAX_PWM = 0;
    }
    if (Switch <= SWITCH_LOW + 100)  //Reverse Gear
    {
      MAX_PWM = 255;
      Motor::Forward(MotorPWM, Delta_PWM, MAX_PWM);
    }
  } 
  else {
    if (Throttle >= THROTTLE_NEUTRAL + 100)  //THROTTLE LOW: 1111 (avg), maybe 1120- MID 1600 HIGH: 2102 (avg) maybe 2000+
    {
      //go fwd
      MAX_PWM = 255;
      MotorPWM = map(Throttle, THROTTLE_NEUTRAL, THROTTLE_HIGH, 0, 255);
      Motor::Forward(MotorPWM, Delta_PWM, MAX_PWM);
    } else if (Throttle <= THROTTLE_NEUTRAL - 100) {  //bckwards
      MAX_PWM = 255;
      MotorPWM = map(Throttle, THROTTLE_NEUTRAL, THROTTLE_LOW, 0, 255);
      Motor::Reverse(MotorPWM, Delta_PWM, MAX_PWM);
    } else {  //stop
      Motor::Stop();
      MAX_PWM = 0;
      MotorPWM = 0;
    }
  }
  //AILERON LOW: 1183 (avg) 1189 HIGH: 1680, (avg) 1674
  //SWITCH LOW: 991 MID: 1490 HIGH: 1986
}

/*void TankRCDrive() {
  MAX_PWM = 255;
  target_heading = mpu.getAngleZ();
  float LEFT_PWM = 0;
  float RIGHT_PWM = 0;
  bool LeftFwd = false;
  bool RightFwd = false;
  if (Throttle >= 1700)  //THROTTLE LOW: 1111 (avg), maybe 1120- MID 1600 HIGH: 2102 (avg) maybe 2000+
  {
    LEFT_PWM = map(Throttle, 1700, 2110, 0, 255);
    LeftFwd = true; //this can invert the forward or backward axis of turn
    //Motor::Forward(MotorPWM, Delta_PWM, MAX_PWM);
  } else if (Throttle <= 1500) {  //bckwards
    LEFT_PWM = map(Throttle, 1700, 1100, 0, 255);
    LeftFwd = false;
    //Motor::Reverse(MotorPWM, Delta_PWM, MAX_PWM);
  } else {  //stop
    LEFT_PWM = 0;
  }

  if (Aileron <= 1300)  //LOW: 1183 (avg) 1189 HIGH: 1680, (avg) 1674 || NEUTRAL: 1370 UP: 879 DOWN: 1865
  {
    RIGHT_PWM = map(Aileron, 879, 1370, 255, 0);
    RightFwd = true;
    //Motor::Forward(MotorPWM, Delta_PWM, MAX_PWM);
  } else if (Aileron >= 1400) {  //bckwards
    RIGHT_PWM = map(Aileron, 1370, 1865, 0, 255);
    RightFwd = false;
    //Motor::Reverse(MotorPWM, Delta_PWM, MAX_PWM);
  } else {  //stop
    RIGHT_PWM = 0;
  }

  Motor::TankDrive(LEFT_PWM, LeftFwd, RIGHT_PWM, RightFwd, Delta_PWM);
}*/

void forward() {
  if (distance <= squareSize) {
    //Serial.println("FORWARD");
    //RGB_WHITE();
    //Motor::Forward(MotorPWM, Delta_PWM);
    //tryLCDDisplay("FORWARD");
  } else {
    Motor::Stop();
  }
}

void arcadeDriveLoop()
{
  RPM();
  //MotorPWM = map(analogRead(potentiometerPin), 0, 1023, 0, 255);
  //battery_voltage = static_cast< float >(analogRead(batteryPin)) / 1000.0 * 7.58;
  //displayInfo();
  //CalculateDriveMode();
  ArcadeRCDrive(false);
  if(GripperSet <= GRIPPER_LOW + 100 && servoPos != SERVO_OPEN_POS)
  {
    gripperOpen();
  }
  else if(GripperSet >= GRIPPER_HIGH - 100 && servoPos != SERVO_CLOSE_POS)
  {
    gripperClose();
  }
}

void BTRampUpLoop() //not using threads in interest of saving time in coding - quick lesson in class.
{
  while(BT_PWM < 255)
  {
    RPM();
    BT_PWM += 5;
    delay(100);
    BTSerial.println(String(linearSpeed));
    Motor::Forward(BT_PWM, 0, BT_PWM);
  }
  while(BT_PWM > 0)
  {
    RPM();
    BT_PWM -= 5;
    delay(20);
    BTSerial.println(String(linearSpeed));
    Motor::Forward(BT_PWM, 0, BT_PWM);
  }
  delay(1500);
}

void setup() {
  //begin initiation
  Serial.begin(9600);
  BTSerial.begin(38400);  // HC-05 default speed in AT command mode //38400
  lcd.init();
  //lcd.backlight();
  RGB.begin();
  RGB.setBrightness(100);
  MPUInit();
  initialMillis = millis();
  //begin interrupt ISRs
  pinMode(2, INPUT_PULLUP);                                  // Interupt PU for motor A encoder
  attachInterrupt(digitalPinToInterrupt(2), ISR1, FALLING);  // Pin # is pin 2 for interrupt 0 for UNO
  pinMode(3, INPUT_PULLUP);                                  // Interupt PU for motor B encoder
  attachInterrupt(digitalPinToInterrupt(3), ISR2, FALLING);
  //end interrupt ISRs
  //begin pinModes
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
  pinMode(Throttle, INPUT);
  pinMode(Aileron, INPUT);
  pinMode(Switch, INPUT);
  //end pinModes
  //servo
  servo.attach(12);
  servo.write(SERVO_OPEN_POS); //default servo position at start
  //servo end
}

void loop() {
  BTRampUpLoop();
}
 