#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
  SoftwareSerial arduinoSerial = SoftwareSerial(10, 11);//RX TX
#include <Servo.h>

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

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
/*
  BELOW BY GROUP C4-C
*/

// enum class for states with different function executions
enum class Mode {
  NORMAL, // Obstacle avoidance + Line tracking + Tennis ball detection + Wifi communication
  OBSTACLE_DETECTED, // -line tracking, when obstacle detected
  TENNIS_DETECTED, // -obstacle avoidance, when tennis ball detected but not catched
  CATCHING, // +Tennis ball catch, -obstacle avoidance, -line tracking, when catching tennis ball
  CATCHED// +Tennis ball catch, when tennis ball catched by arm
}Mode;


//infrared sensors ||left:out-gnd-vcc:right||
#define INFRARED1 25 //front left PA3
#define INFRARED2 28 //front right PA6
#define INFRARED3 29 //left PA7
#define INFRARED4 30 //right PC7
#define INFRARED5 32 //back PC5
int Infrared_front_left, Infrared_front_right;
int Infrared_left, Infrared_right, Infrared_back;

//grayscale sensors
#define GRAYSCALE1 A3 //left most PF1
#define GRAYSCALE2 A2 //left second PF2
#define GRAYSCALE3 A6 //middle PF3
#define GRAYSCALE4 A7 //right second PF4
#define GRAYSCALE5 A8 //right most PF5
double Grayscale_middle_left, Grayscale_middle_right;
double Grayscale_middle;
double Grayscale_left, Grayscale_right;

//ultrasonic sensors
// #define SONAR_TRIG 29 //PA7
// #define SONAR_ECHO 28 //PA6
double Sonar_distance_in_cm;
int done, start_time;

//flags to determine whether other functions should be executed
bool Obstacle_flag = false;
// bool Line_flag = false;
bool Tennis_flag = false;
bool Catching_flag = false;
bool Catched_flag = false;


#define Wheel_Radius 0.04 //m
#define MOTOR_KP 0.15
#define MOTOR_KI 0.0
#define MOTOR_KD 0.0
#define CLIP(x, min, max) if (x < min) x = min; if (x > max) x = max;

//WIFI
String  message = "";

//Vision
String vision_message = "";

//Servo motor
// #define PitchPin 13
// #define YawPin 14
Servo PitchServo; // 10-150
Servo YawServo; // 0-180

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
      double pwm;
      double pwm_set;
      String wifi_cmd;

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

class Gimbal_control_t {
  private:

  public:
    double pitch, yaw;

} Gimbal_control;

const double EPRA = 660;//�?速比�?1�?660
const double EPRB = 660;//�?速比�?1�?660
const double EPRC = 660;//�?速比�?1�?660
const double EPRD = 660;//�?速比�?1�?660

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
// PID motorPID1(&motor1.pwm, &pidout1, &motor1.pwm_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);
// PID motorPID2(&motor2.pwm, &pidout2, &motor2.pwm_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);
// PID motorPID3(&motor3.pwm, &pidout3, &motor3.pwm_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);
// PID motorPID4(&motor4.pwm, &pidout4, &motor4.pwm_set, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);


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
  motorPID1.SetSampleTime(1); motorPID2.SetSampleTime(1);
  motorPID3.SetSampleTime(1); motorPID4.SetSampleTime(1);
  motorPID1.SetOutputLimits(-255, 255); motorPID2.SetOutputLimits(-255, 255);
  motorPID3.SetOutputLimits(-255, 255); motorPID4.SetOutputLimits(-255, 255);
}


//Where the program starts
void setup(){
  Serial.begin(9600); // USB serial setup
  Serial.println("Start");

  //Motor Setup
  motor_setup();

  //Ultrasonic Setup
  // pinMode(SONAR_ECHO, INPUT);
  // pinMode(SONAR_TRIG, OUTPUT);

  //WiFi Setup
  arduinoSerial.begin(9600);
  arduinoSerial.flush();

  // Servo Setup
  // PitchServo.attach(PitchPin);
  // YawServo.attach(YawPin);
  // PitchServo.write(30);
  // YawServo.write(90);

  //Mode Setup
  Mode = Mode::NORMAL; // original mode == NORMAL
}

// Sonar front distance from obstacle to car
// void Get_front_distance(){
//   long Sonar_duration;
//   if (done){
//     done = false;
//     start_time = millis();
//     digitalWrite(SONAR_TRIG, LOW); 
//   }
//   if (millis() - start_time > 2){
//     digitalWrite(SONAR_TRIG, HIGH);
//   }
//   if (millis() - start_time > 10){
//     digitalWrite(SONAR_TRIG, LOW);
//     Sonar_duration = pulseIn(SONAR_ECHO, HIGH);
//     Sonar_distance_in_cm = (Sonar_duration/0.4) / 29.1;
//     done = true;
//   }
// }

// Infrared detection
void Infrared_states(){
  Infrared_front_left = digitalRead(INFRARED1);
  Infrared_front_right = digitalRead(INFRARED2);
  Infrared_left = digitalRead(INFRARED4);
  Infrared_right = digitalRead(INFRARED3); 
  Infrared_back = digitalRead(INFRARED5);
}

// Grayscale detection
void Grayscale_values(){
  Grayscale_middle_left = analogRead(GRAYSCALE2);
  Grayscale_middle_right = analogRead(GRAYSCALE4);
  Grayscale_middle = analogRead(GRAYSCALE3);
  Grayscale_left = analogRead(GRAYSCALE1);
  Grayscale_right = analogRead(GRAYSCALE5);
  // pinMode(A8, OUTPUT);
  // pinMode(33,OUTPUT);
  // pinMode(A10, OUTPUT);

}

void Esp8266_recv(){
  if (arduinoSerial.available() > 0) {
      message = arduinoSerial.readString();
    Serial.println(message);
    Chassis_control.wifi_cmd = message;
    message = "";
  }
}

//protocal: Yaw-Pitch
void Vision_recv(){
  // if (Serial.available() > 0) {
  //     vision_message = Serial.readString();
  //     delay(1);
  // Gimbal_control.yaw = vision_message.substring(0, vision_message.indexOf('-')).toInt();
  // Gimbal_control.pitch = vision_message.substring(vision_message.indexOf('-')+1).toInt();
  //   Serial.println(message);
  //   vision_message = "";
  // }
}

void Data_update() {
  // Motor
  eps1 = motor1.ecd - motor1.last_ecd;
  eps2 = motor2.ecd - motor2.last_ecd;
  eps3 = motor3.ecd - motor3.last_ecd;
  eps4 = motor4.ecd - motor4.last_ecd;
  motor1.last_ecd = motor1.ecd;
  motor2.last_ecd = motor2.ecd;
  motor3.last_ecd = motor3.ecd;
  motor4.last_ecd = motor4.ecd;
  // eps1 = eps1 * 1000 / 660; // encoder count per second ???
  motor1.speed = eps1 * Wheel_Radius / EPRA * 100;
  motor2.speed = eps2 * Wheel_Radius / EPRB * 100;
  motor3.speed = eps3 * Wheel_Radius / EPRC * 100; 
  motor4.speed = eps4 * Wheel_Radius / EPRD * 100;
  // motor1.pwm = motor1.speed / 2.4 * 255;
  // motor2.pwm = motor2.speed / 2.4 * 255;
  // motor3.pwm = motor3.speed / 2.4 * 255;
  // motor4.pwm = motor4.speed / 2.4 * 255;

  // infrared
  Infrared_states();

  // gray scale detect
  Grayscale_values();

  // vision detect
  
  // IMU

  // ultrasonic 
  // Get_front_distance();

  // respbreey pi comm / Esp8266 || Serial 
  // Esp8266_recv();
  // Vision_recv();
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
  wheel_speed[0] = -vy - vx - wz;
  wheel_speed[1] = vy - vx - wz;  
  wheel_speed[2] = -vy + vx - wz;
  wheel_speed[3] = vy + vx - wz;
  motor1.speed_set = wheel_speed[0];
  motor2.speed_set = wheel_speed[1];
  motor3.speed_set = wheel_speed[2];
  motor4.speed_set = wheel_speed[3];
  motor1.pwm_set = wheel_speed[0] / 2.4 * 255;
  motor2.pwm_set = wheel_speed[1] / 2.4 * 255;
  motor3.pwm_set = wheel_speed[2] / 2.4 * 255;
  motor4.pwm_set = wheel_speed[3] / 2.4 * 255;
}

// speed of motor 0-2.4 from pwm 0-255
void Motor_control(){
  Chassis_Vector_to_Mecanum_Wheel_Speed(Chassis_control.vx, Chassis_control.vy, Chassis_control.wz);

  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();
  motorPID4.Compute();
  pidout1 = pidout1 / 2.4 * 255;
  pidout2 = pidout2 / 2.4 * 255;
  pidout3 = pidout3 / 2.4 * 255;
  pidout4 = pidout4 / 2.4 * 255;
  motor1.pwm = motor1.speed_set / 2.4 * 255;
  motor2.pwm = motor2.speed_set / 2.4 * 255;
  motor3.pwm = motor3.speed_set / 2.4 * 255;
  motor4.pwm = motor4.speed_set / 2.4 * 255;
  motor1.pwm = motor1.pwm + pidout1;
  motor2.pwm = motor2.pwm + pidout2;
  motor3.pwm = motor3.pwm + pidout3;
  motor4.pwm = motor4.pwm + pidout4;
  CLIP(pwm1, -255, 255);
  CLIP(pwm2, -255, 255);
  CLIP(pwm3, -255, 255);
  CLIP(pwm4, -255, 255);
  motor1.setMotor(motor1.pwm);
  motor2.setMotor(motor2.pwm);
  motor3.setMotor(motor3.pwm);
  motor4.setMotor(motor4.pwm);
  // motor1.setMotor(pidout1);
  // motor2.setMotor(pidout2);
  // motor3.setMotor(pidout3);
  // motor4.setMotor(pidout4);
}

void Move(double x, double y, double z){ // control car movement by setting x, y, z
  Chassis_control.vx = x;
  Chassis_control.vy = y;
  Chassis_control.wz = z;
}

void Obstacle_avoidance(){
  // infrared 7.5 cm 
  // combine all infrared sensor states to one value
  int Infrared_combined = 0b00000;
  if (!Infrared_front_left){ // front left infrared sensor detect obstacle, lowest digit = 1
    Infrared_combined = Infrared_combined | 0b00001;
    Obstacle_flag = true;
  }
  if (!Infrared_front_right){ // front right infrared sensor detect obstacle, second lowest digit = 1
    Infrared_combined = Infrared_combined | 0b00010;
    Obstacle_flag = true;
  }
  if (!Infrared_left){ // left infrared sensor detect obstacle, third lowest digit = 1
    Infrared_combined = Infrared_combined | 0b00100;
    Obstacle_flag = true;
  }
  if (!Infrared_right){ // right infrared sensor detect obstacle, fourth lowest digit = 1
    Infrared_combined = Infrared_combined | 0b01000;
    Obstacle_flag = true;
  }
  if (!Infrared_back){ // back infrared sensor detect obstacle, highest digit = 1
    Infrared_combined = Infrared_combined | 0b10000;
    Obstacle_flag = true;
  }
  if (Infrared_combined == 0b00000){
    Obstacle_flag = false;
  }

  Serial.println(Infrared_combined);
  switch (Infrared_combined) {
    // case 0b00000: //no obstacle
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b00001: //front left, move towards right
      Move(0.4, 0.0, 0.0);
      break;
    case 0b00010: //front right, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    case 0b00011: //front left and front right, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b00100: //left, go straigt
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b00101: //left and front left, move towards right
      Move(0.4, 0.0, 0.0);
      break;
    case 0b00110: //left and front right, move towards right // actually not possible?
      Move(0.4, 0.0, 0.0);
      break;
    case 0b00111: //left, front left and front right, move towards right
      Move(0.4, 0.0, 0.0);
      break;
    // case 0b01000: //right, go straight
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b01001: //right and front left, move towards left // actually not possible?
      Move(-0.4, 0.0, 0.0);
      break;
    case 0b01010: //right and front right, move towards left 
      Move(-0.4, 0.0, 0.0);
      break;
    case 0b01011: //right, front left and front right, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b01100: //right and left, go straigt
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b01101: //right, left and front left, move backward
      Move(0.0, -0.4, 0.0);
      break;
    case 0b01110: //right, left and front right, move backward
      Move(0.0, -0.4, 0.0);
      break;
    case 0b01111: //right, left, front left and front right, move backward
      Move(0.0, -0.4, 0.0);
      break;
    // case 0b10000: //back, go straight
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b10001: //back and front left, move forward right
      Move(0.4, 0.4, 0.0);
      break;
    case 0b10010: //back and front right, move forward left
      Move(-0.4, 0.4, 0.0);
      break;
    case 0b10011: //back, front left and front right, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b10100: //back and left, go straight
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b10101: //back, left and front left, move towards right
      Move(0.4, 0.0, 0.0);
      break;
    case 0b10110: //back, left and front right, move towards right  // actually not possible?
      Move(0.4, 0.0, 0.0);
      break;
    case 0b10111: //back, left, front left and front right, move towards right
      Move(0.4, 0.0, 0.0);
      break;
    // case 0b11000: //back and right, go straight
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b11001: //back, right and front left, move towards left // actually not possible?
      Move(0.4, 0.0, 0.0);
      break;
    case 0b11010: //back, right and front right, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    case 0b11011: //back, right, front left and front right, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b11100: //back, right and left, go straight
    //   Move(0.0, 0.4, 0.0);
    //   break;
    case 0b11101: //back, right, left and front left, move forward right // actually not possible?
      Move(0.4, 0.4, 0.0);
      break;
    case 0b11110: //back, right, left and front right, move forward left // actually not possible?
      Move(-0.4, 0.4, 0.0);
      break;
    case 0b11111: //back, right, left, front left and front right, can only stop // actually not possible?
      Move(0.0, 0.0, 0.0);
      break;
    default:
      Move(0.0, 0.4, 0.0);
      break;
  }
}

void Line_tracking(){
  // gray scale detect 0.1 cm tolerance
  // combine all grayscale sensor states to one value
  int Grayscale_combined = 0b00000;
  if (Grayscale_right > 900) { // right most grayscale sensor detect white line, lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b00001;
  }
  if (Grayscale_middle_right > 900) { // right second grayscale sensor detect white line, second lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b00010;
  }
  if (Grayscale_middle > 900) { // middle grayscale sensor detect white line, third lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b00100;
  }
  if (Grayscale_middle_left > 900) { // left second grayscale sensor detect white line, fourth lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b01000;
  }
  if (Grayscale_left > 900) { // left most grayscale sensor detect white line, highest digit = 1
    Grayscale_combined = Grayscale_combined | 0b10000;
  }
  switch (Grayscale_combined){
    // case 0b00000: // no white line detected
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b00001: // only right most detect white, may be court edge, move towards right
      Move(0.4, 0.0, 0.0);
      Serial.println("right most");
      break;
    // case 0b00010: // only middle right detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b00011: // right most and middle right detect white, may be court edge, move towards right
      Move(0.4, 0.0, 0.0);
      Serial.println("right most and middle right");
      break;
    // case 0b00100: // only middle detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b00101: // middle and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b00110: // middle and middle right detect white, may be court edge, move towards right
      Move(0.4, 0.0, 0.0);
      Serial.println("middle and middle right");
      break;
    case 0b00111: // middle, middle right and right most detect white, may be court edge, move towards right
      Move(0.4, 0.0, 0.0);
      Serial.println("middle, middle right and right most");
      break;
    // case 0b01000: // only middle left detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b01001: // middle left and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b01010: // middle left and middle right detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b01011: // middle left, middle right and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b01100: // middle left detect white, may be court edge, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b01101: // middle left, middle and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b01110: // middle three detect white, court edge (middle part)
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b01111: // middle three + right most detect white, court edge (right corner)
      Move(0.0, 0.0, -0.10); // rotate left 90 degree
      break;
    case 0b10000: // only left most detect white, may be court edge, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b10001: // left most and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b10010: // left most and middle right detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b10011: // left most, middle right and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b10100: // left most and middle detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b10101: // left most, middle and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b10110: // left most, middle right and middle detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b10111: // left most, middle, middle right and right detect white, may be court edge, move towards right
      Move(0.4, 0.0, 0.0);
      break;
    case 0b11000: // left most and middle left detect white, may be court edge, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    // case 0b11001: // left most, middle left and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b11010: // left most, middle left and middle right detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    // case 0b11011: // left most, middle left, middle right and right most detect white, not court edge, ignore
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    case 0b11100: // left most, middle left and middle detect white, may be court edge, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    case 0b11101: // left most, middle left, middle and right most detect white, may be court edge, move towards left
      Move(-0.4, 0.0, 0.0);
      break;
    case 0b11110: // left most + middle three, court edge (left corner)
      Move(0.0, 0.0, 0.10); // rotate right 90 degree
      break;
    // case 0b11111: // all detect white, court edge (crossroad)
    //   Move(0.0, 0.4, 0.0); // go straight
    //   break;
    default:
      Move(0.0, 0.4, 0.0); // go straight
      break;
  }
}

void Vision_tracking(){
  // vision detect from camera
}

void Arm_control(){
  // servo control

}

void Gimbal_motor_control(){
  // servo control
  // YawServo.write(Gimbal_control.yaw);
  // PitchServo.write(Gimbal_control.pitch);
  
}

void Mode_switch(){
  // execute functions under different modes
  switch (Mode){
    case Mode::NORMAL:
      if (Obstacle_flag){
        Mode = Mode::OBSTACLE_DETECTED;
      }
      else if (Tennis_flag){
        Mode = Mode::TENNIS_DETECTED;
      }
      Obstacle_avoidance();
      Line_tracking();
      Vision_tracking();
      Gimbal_motor_control();
      break;
    case Mode::OBSTACLE_DETECTED:
      if (!Obstacle_flag){
        Mode = Mode::NORMAL;
      }
      else if (Tennis_flag){
        Mode = Mode::TENNIS_DETECTED;
      }
      Obstacle_avoidance();
      Vision_tracking();
      Gimbal_motor_control();
      break;
    case Mode::TENNIS_DETECTED:
      if (!Tennis_flag && !Catching_flag){
        Mode = Mode::OBSTACLE_DETECTED;
      }
      else if (Catching_flag){
        Mode = Mode::CATCHING;
      }
      Vision_tracking();
      Gimbal_motor_control();
      break;
    case Mode::CATCHING:
      if (Catched_flag){
        Mode = Mode::CATCHED;
      }
      Vision_tracking();
      Arm_control();
      Gimbal_motor_control();
      break;
    case Mode::CATCHED:
      if (!Catched_flag){
        Mode = Mode::NORMAL;
      }
      Obstacle_avoidance();
      Line_tracking();
      Vision_tracking();
      Arm_control();
      Gimbal_motor_control();
      break;
    default:
      break;
  }
}


// speed_leve: 2.0f || 5.0f || 10.0f
float debug1,debug2,debug3, debug4;
int debug5,debug6,debug7,debug8;
void debug(){
  //testing
  // motor
  // MOTORA_FORWARD(255);
  // MOTORB_FORWARD(255);
  // MOTORC_FORWARD(255);
  // MOTORD_FORWARD(255);
  // Chassis_control.vx = 0.5;
  // Chassis_control.vy = 0.0;
  // Chassis_control.wz = 0.0;
  // Serial.print("M1 ecd:");
  // Serial.println(motor1.ecd);
  // Serial.print("M1 speed:");
  // Serial.println(motor1.speed);
  // Serial.print("M1 speed_set: ");
  // Serial.println(motor1.speed_set);
  // Serial.print("M1 pidout: ");
  // Serial.println(pidout1);
  // Serial.print("M1 pwm: ");
  // Serial.println(motor1.pwm);
  // Serial.print("M1 pwm_set: ");
  // Serial.println(motor1.speed_set / 2.4 * 255);
  // Serial.println(motor1.speed); // serialport debug

  //infrared 
  // Serial.print("Infrared_front_left: ");
  // Serial.println(Infrared_front_left);
  // Serial.print("Infrared_front_right: ");
  // Serial.println(Infrared_front_right);
  // Serial.print("Infrared_left: ");
  // Serial.println(Infrared_left);
  // Serial.print("Infrared_right: ");
  // Serial.println(Infrared_right);
  // Serial.print("Infrared_back: ");
  // Serial.println(Infrared_back);

  //gray scale
  Serial.print("Grayscale_left: ");
  Serial.println(Grayscale_left);
  Serial.print("Grayscale_middle_left: ");
  Serial.println(Grayscale_middle_left);
  Serial.print("Grayscale_middle: ");
  Serial.println(Grayscale_middle);
  Serial.print("Grayscale_middle_right: ");
  Serial.println(Grayscale_middle_right);
  Serial.print("Grayscale_right: ");
  Serial.println(Grayscale_right);

  // MOTORA_FORWARD(0);
  // MOTORB_FORWARD(0);
  // MOTORC_FORWARD(0);
  // MOTORD_FORWARD(0);
  
  // gimbal
  // servo1.write(180);
  // Serial.println(servo1.read());

}

void loop()
{
  time = millis();
  Data_update();
  // Mode_switch();
  // Obstacle_avoidance();
  if (Obstacle_flag == false){
    // Line_tracking();
  }
	// Line_tracking();
	// Vision_tracking();
  // Gimbal_motor_control();
	// Arm_control();
  Motor_control();

  //debug
  debug();

  delay(1);
}