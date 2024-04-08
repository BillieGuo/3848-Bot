#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
int servo_fmin = 20;
int servo_max = 160;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
/*
  BELOW BY GROUP C4-C
*/

#include <Arduino.h>

#define Wheel_Radius 0.04 //m
#define MOTOR_KP 30.0
#define MOTOR_KI 0.05
#define MOTOR_KD 0.1

#define NORM(x, min, max) if (x < min) x = min; if (x > max) x = max;

class DCMotor {
  private:
      int pwmPin; // PWM pin for motor speed control
      int dirAPin;
      int dirBPin;

  public:
      int encoderAPin;
      int encoderBPin;
      double ecd; // Encoder value
      double last_ecd;
      double speed;
      double speed_set;

      DCMotor(int pwm, int dirA, int dirB, int encoderA, int encoderB);
      void setMotor(int analogSpeed);
      void setSpeed(int analogSpeed);
      void setDirection(int dir);
      void encoderSubroutineA();
      void encoderSubroutineB();
};

class Chassis_control_t {
  private:

  public:
    double vx, vy, wz;
} Chassis_control;


const double EPRA = 660;//转速比：1：660
const double EPRB = 660;//转速比：1：660
const double EPRC = 660;//转速比：1：660
const double EPRD = 660;//转速比：1：660

const int pwmPin1 = 12; const int dir1A = 34; const int dir1B = 35; const int encoder1A = 18; const int encoder1B = 31; // A M1
const int pwmPin2 = 8; const int dir2A = 37; const int dir2B = 36; const int encoder2A = 19; const int encoder2B = 38; // B M2
const int pwmPin3 = 6; const int dir3A = 43; const int dir3B = 42; const int encoder3A = 3; const int encoder3B = 49; // C M3
const int pwmPin4 = 5; const int dir4A = A4; const int dir4B = A5; const int encoder4A = 2; const int encoder4B = A1; // D M4

//testing
#define MOTORA_FORWARD(pwm)    do{digitalWrite(dir1A,LOW); digitalWrite(dir1B,HIGH);analogWrite(pwmPin1,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(dir1A,LOW); digitalWrite(dir1B,LOW); analogWrite(pwmPin1,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(dir1A,HIGH);digitalWrite(dir1B,LOW); analogWrite(pwmPin1,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(dir2A,LOW); digitalWrite(dir2B,HIGH);analogWrite(pwmPin2,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(dir2A,LOW); digitalWrite(dir2B,LOW); analogWrite(pwmPin2,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(dir2A,HIGH);digitalWrite(dir2B,LOW); analogWrite(pwmPin2,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(dir3A,LOW); digitalWrite(dir3B,HIGH);analogWrite(pwmPin3,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(dir3A,LOW); digitalWrite(dir3B,LOW); analogWrite(pwmPin3,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(dir3A,HIGH);digitalWrite(dir3B,LOW); analogWrite(pwmPin3,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(dir4A,LOW); digitalWrite(dir4B,HIGH);analogWrite(pwmPin4,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(dir4A,LOW); digitalWrite(dir4B,LOW); analogWrite(pwmPin4,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(dir4A,HIGH);digitalWrite(dir4B,LOW); analogWrite(pwmPin4,pwm);}while(0)
//end testing

const double pi = 3.14159265358979323846;

double eps1 = 0, eps2 = 0, eps3 = 0, eps4 = 0;// encoder count per second
double eps1_fb = 0, eps2_fb = 0, eps3_fb = 0, eps4_fb = 0;// eps filtered feedback
double pidout1 = 0, pidout2 = 0, pidout3 = 0, pidout4 = 0;
double pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;

DCMotor motor1(pwmPin1, dir1A, dir1B, encoder1A, encoder1B); DCMotor motor2(pwmPin2, dir2A, dir2B, encoder2A, encoder2B);
DCMotor motor3(pwmPin3, dir3A, dir3B, encoder3A, encoder3B); DCMotor motor4(pwmPin4, dir4A, dir4B, encoder4A, encoder4B);

// PID::PID(double* Input, double* Output, double* Setpoint,
//         double Kp, double Ki, double Kd, int POn, int ControllerDirection)
PID motorPID1(&motor1.speed, &pidout1, &motor1.speed_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);
PID motorPID2(&motor2.speed, &pidout2, &motor2.speed_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);
PID motorPID3(&motor3.speed, &pidout3, &motor3.speed_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);
PID motorPID4(&motor4.speed, &pidout4, &motor4.speed_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);


DCMotor::DCMotor(int pwm, int dirA, int dirB, int encoderA, int encoderB) : 
pwmPin(pwm), dirAPin(dirA), dirBPin(dirB), encoderAPin(encoderA),
encoderBPin(encoderB), ecd(0) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirAPin, OUTPUT);
    pinMode(dirBPin, OUTPUT);
    pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);
}
void DCMotor::setMotor(int analogSpeed){
    if (analogSpeed >= 0){
        setDirection(1);
        setSpeed(analogSpeed);
    }
    else{
        setDirection(-1);
        setSpeed(-analogSpeed);
    }
    
}
void DCMotor::setSpeed(int analogSpeed) {
    if (analogSpeed < 0) {
        analogSpeed = 0;
    } 
    else if (analogSpeed > 250) {
        analogSpeed = 250;
    }
    analogWrite(pwmPin, analogSpeed);
}
void DCMotor::setDirection(int dir) {
    // 1 forward, 0 stop, -1 backward
    if (dir == 1) {
        digitalWrite(dirAPin, HIGH);
        digitalWrite(dirBPin, LOW);
    } 
    else if(dir == 0) {
        digitalWrite(dirAPin, LOW);
        digitalWrite(dirBPin, LOW);
    }
    else if(dir == -1){
        digitalWrite(dirAPin, LOW);
        digitalWrite(dirBPin, HIGH);
    }
    else {
        Serial.println("Invalid direction");
        digitalWrite(dirAPin, LOW);
        digitalWrite(dirBPin, LOW);
    }
}

// encoder interrupt cnt
void DCMotor::encoderSubroutineA() {
    if (digitalRead(encoderAPin)){
        if (digitalRead(encoderBPin)){
            ecd ++;
        }
        else{
            ecd --;
        }
    }
    else{ 
        if (digitalRead(encoderBPin)){
            ecd --;
        }
        else{
            ecd ++;
        }
    }
}
void DCMotor::encoderSubroutineB() {
    if (digitalRead(encoderBPin)){
        if (digitalRead(encoderAPin)){
            ecd ++;
        }
        else{
            ecd --;
        }
    }
    else{ 
        if (digitalRead(encoderAPin)){
            ecd --;
        }
        else{
            ecd ++;
        }
    }
}

// setup interrupt functions
  void encoder_subroutine_1A(){motor1.encoderSubroutineA();}
  void encoder_subroutine_2A(){motor2.encoderSubroutineA();}
  void encoder_subroutine_3A(){motor3.encoderSubroutineA();}
  void encoder_subroutine_4A(){motor4.encoderSubroutineA();} 
  void encoder_subroutine_1B(){motor1.encoderSubroutineB();}
  void encoder_subroutine_2B(){motor2.encoderSubroutineB();}
  void encoder_subroutine_3B(){motor3.encoderSubroutineB();}
  void encoder_subroutine_4B(){motor4.encoderSubroutineB();} 

// motor setup
void motor_setup(){
  
  DCMotor motor1(pwmPin1, dir1A, dir1B, encoder1A, encoder1B); DCMotor motor2(pwmPin2, dir2A, dir2B, encoder2A, encoder2B);
  DCMotor motor3(pwmPin3, dir3A, dir3B, encoder3A, encoder3B); DCMotor motor4(pwmPin4, dir4A, dir4B, encoder4A, encoder4B);

  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder_subroutine_1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), encoder_subroutine_2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3A), encoder_subroutine_3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4A), encoder_subroutine_4A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1B), encoder_subroutine_1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2B), encoder_subroutine_2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3B), encoder_subroutine_3B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4B), encoder_subroutine_4B, CHANGE);

  motorPID1.SetMode(AUTOMATIC); motorPID2.SetMode(AUTOMATIC);
  motorPID3.SetMode(AUTOMATIC); motorPID4.SetMode(AUTOMATIC); 
  motorPID1.SetSampleTime(10); motorPID2.SetSampleTime(10);
  motorPID3.SetSampleTime(10); motorPID4.SetSampleTime(10);
  motorPID1.SetOutputLimits(-255, 255); motorPID2.SetOutputLimits(-255, 255);
  motorPID3.SetOutputLimits(-255, 255); motorPID4.SetOutputLimits(-255, 255);
}




// void cmd_vel_cb(const geometry_msgs::Twist& vel_msg){ // x // y // z
//     delay(10);
//     motorPID1.outputSum = 0;
//     motorPID2.outputSum = 0;
//     motorPID3.outputSum = 0;
//     motorPID4.outputSum = 0;
//     motor1.setDirection(0);motor2.setDirection(0);motor3.setDirection(0);motor4.setDirection(0);
//     mecanumDrive.calculateWheelSpeeds(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
//     set_eps1 = EPRA * mecanumDrive.wheelSpeeds[0] / 2 / pi; 
//     set_eps2 = EPRB * mecanumDrive.wheelSpeeds[1] / 2 / pi; 
//     set_eps3 = EPRC * mecanumDrive.wheelSpeeds[2] / 2 / pi; 
//     set_eps4 = EPRD * mecanumDrive.wheelSpeeds[3] / 2 / pi; 
// }


//Where the program starts
void setup(){
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot");
  display.display();

  //Motor Setup
  motor_setup();
}

void Data_update() {
  eps1 = motor1.ecd - motor1.last_ecd;
  eps2 = motor2.ecd - motor2.last_ecd;
  eps3 = motor3.ecd - motor3.last_ecd;
  eps4 = motor4.ecd - motor4.last_ecd;
  motor1.last_ecd = motor1.ecd;
  motor2.last_ecd = motor2.ecd;
  motor3.last_ecd = motor3.ecd;
  motor4.last_ecd = motor4.ecd;
  // eps1_fb = filter1.add(eps1);
  // eps2_fb = filter2.add(eps2);
  // eps3_fb = filter3.add(eps3);
  // eps4_fb = filter4.add(eps4);

  motor1.speed = eps1 * Wheel_Radius / EPRA * 100; // m/s
  motor2.speed = eps2 * Wheel_Radius / EPRB * 100;
  motor3.speed = eps3 * Wheel_Radius / EPRC * 100; 
  motor4.speed = eps4 * Wheel_Radius / EPRD * 100;

}

// Motor implementation
// M1/A ----- M2/B
//   |          |
//   |          |
// M3/C ----- M4/D
// vy -> forward
// vx -> right
// wz -> rotate CW
void Chassis_Vector_to_Mecanum_Wheel_Speed(double vx, double vy, double wz){
  double wheel_speed[4];
  wheel_speed[0] = -vy + vx - wz;
  wheel_speed[1] = vy + vx - wz;  
  wheel_speed[2] = -vy - vx - wz;
  wheel_speed[3] = vy - vx - wz;
  motor1.speed_set = wheel_speed[0];
  motor2.speed_set = wheel_speed[1];
  motor3.speed_set = wheel_speed[2];
  motor4.speed_set = wheel_speed[3];
}

// speed of motor 0-0.36m/s from pwm 0-255
void Motor_control(){
  // Chassis_control.vx = 0.05;
  // Chassis_control.vy = 1.06;
  // Chassis_control.wz = 0.0;

  Chassis_Vector_to_Mecanum_Wheel_Speed(Chassis_control.vx, Chassis_control.vy, Chassis_control.wz);

  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();
  motorPID4.Compute();
  pwm1 = pidout1 / 0.36 * 255;
  pwm2 = pidout2 / 0.36 * 255;
  pwm3 = pidout3 / 0.36 * 255;
  pwm4 = pidout4 / 0.36 * 255;
  NORM(pwm1, -255, 255)
  NORM(pwm2, -255, 255)
  NORM(pwm3, -255, 255)
  NORM(pwm4, -255, 255)
  motor1.setMotor(pwm1);
  motor2.setMotor(pwm2);
  motor3.setMotor(pwm3);
  motor4.setMotor(pwm4);
  }

float debug1,debug2;
void loop()
{
  // run the code in every 10ms
  if (millis() - time > 10){
    time = millis();
    Data_update();
    Motor_control();



    //testing
    // MOTORA_FORWARD(10);
    // MOTORB_FORWARD(255);
    // MOTORC_FORWARD(255);
    // MOTORD_FORWARD(255);
    Chassis_control.vx = -0.10;
    Chassis_control.vy = 0.00;
    Chassis_control.wz = 0.0;
    // Serial.print("M1 ecd:");
    // Serial.println(motor1.ecd);
    // Serial.print("M1 speed:");
    // Serial.println(motor1.speed);
    // Serial.print("M1 speed_set: ");
    // Serial.println(motor1.speed_set);
    // Serial.print("M1 pidout: ");
    // Serial.println(pidout1);
    // Serial.print("M1 pwm: ");
    // Serial.println(pwm1);
    // Serial.println(motor1.speed); // serialport debug

  delay(10);
  }
}