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
int Infrared_combined = 0b0000;


//grayscale sensors
#define GRAYSCALE1 A3 //left most PF1
#define GRAYSCALE2 A2 //left second PF2
#define GRAYSCALE3 A6 //middle PF3
#define GRAYSCALE4 A7 //right second PF4
#define GRAYSCALE5 A8 //right most PF5
double Grayscale_middle_left, Grayscale_middle_right;
double Grayscale_middle;
double Grayscale_left, Grayscale_right;
int Grayscale_combined = 0b00000;


//ultrasonic sensors
// #define SONAR_TRIG 29 //PA7
// #define SONAR_ECHO 28 //PA6
double Sonar_distance_in_cm;
int done, start_time;

//flags to determine whether other functions should be executed
bool Obstacle_flag = false;
bool Front_flag = false;
bool Left_flag = false;
bool Right_flag = false;
int retrieve_flag = 0; // 0: no retrieve, 1: front obstacle, 2: avoid obstacle, 3: conduct go left, 4:conduct go right
int front_cnt = 0, left_cnt = 0, right_cnt = 0;

// bool Line_flag = false;
bool Rotate_flag = false; // whether continue rotation
int rotate_cnt = 0;
bool Tennis_flag = false;
bool Catching_flag = false;
bool Catched_flag = false;

// back to line
int last_time = 0, shift_stop_time = 0;
float last_avoid_move_x = 0.0;
float last_avoid_move_y = 0.0;
float last_avoid_move_z = 0.0;


#define Wheel_Radius 0.04 //m
#define MOTOR_KP 0.4
#define MOTOR_KI 0.0
#define MOTOR_KD 0.0
#define CLIP(x, min, max) if (x < min) x = min; if (x > max) x = max;
#define LOW_SPEED 0.4
#define MEDIUM_SPEED 0.6
#define HIGH_SPEED 0.8
double all_speed_set = LOW_SPEED;

//WIFI
String  message = "";

//Vision
String vision_message = "";

//Servo motor
#define PitchPin  4 // PG5 
#define YawPin 13 // PB7
#define ArmPin 46 //PL3
Servo PitchServo; // 10-150
Servo YawServo; // 0-180
Servo ArmServo; // 0-180
int angle;

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
    String wifi_cmd;
    int move_cnt;
    int move_flag;
    int target_dir;

} Chassis_control;

class Gimbal_control_t {
  private:

  public:
    int pitch, yaw;
    int cnt;
    int scan_cnt;
    int scan_flag;
    int target;
    int target_flag;

} Gimbal_control;

class Vision_t {
  private:

  public:
    int yaw_dir;
    int target_flag;
    int catch_flag;

} Vision;

const double EPRA = 660;
const double EPRB = 660;
const double EPRC = 660;
const double EPRD = 660;

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
  Serial.begin(115200); // USB serial setup
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
  PitchServo.attach(PitchPin);
  YawServo.attach(YawPin);
  ArmServo.attach(ArmPin); // 500-2500
  PitchServo.write(135);
  YawServo.write(90);
  // ArmServo.write(90);
  Gimbal_control.pitch = 135;
  Gimbal_control.yaw = 90;


  //Mode Setup
  Mode = Mode::NORMAL; // original mode == NORMAL
  
  delay(1000);
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
  
  Infrared_combined = 0b0000;

  if (!Infrared_front_left){ // front left infrared sensor detect obstacle, lowest digit = 1
    Infrared_combined = Infrared_combined | 0b0001;
    Obstacle_flag = true;
    Front_flag = true;
    retrieve_flag = 1;
    front_cnt = 0;
  }
  if (!Infrared_front_right){ // front right infrared sensor detect obstacle, second lowest digit = 1
    Infrared_combined = Infrared_combined | 0b0001;
    Obstacle_flag = true;
    Front_flag = true;
    retrieve_flag = 1;
    front_cnt = 0;
  }
  if (!Infrared_left){ // left infrared sensor detect obstacle, third lowest digit = 1
    Infrared_combined = Infrared_combined | 0b0010;
    Obstacle_flag = true;
    Left_flag = true;
    left_cnt = 0;
  }
  if (!Infrared_right){ // right infrared sensor detect obstacle, fourth lowest digit = 1
    Infrared_combined = Infrared_combined | 0b0100;
    Obstacle_flag = true;
    Right_flag = true;
    right_cnt = 0;
  }
  if (!Infrared_back){ // back infrared sensor detect obstacle, highest digit = 1
    Infrared_combined = Infrared_combined | 0b1000;
    Obstacle_flag = true;
  }
  if (Infrared_combined == 0b0000){ // if all sensors detect no obstacle
    Obstacle_flag = false;
  }
  if ((Infrared_combined & 0b0001) == 0b0000){ // front two sensors detect no obstacle
    front_cnt += 1;
    if (front_cnt >= 400){
      Front_flag = false;
    }
  }
  if ((Infrared_combined & 0b0010) == 0b0000){ // left sensor detect no obstacle
    left_cnt += 1;
    if (left_cnt >= 170){
      Left_flag = false;
    }
  }
  if ((Infrared_combined & 0b0100) == 0b0000){ // right sensor detect no obstacle
    right_cnt += 1;
    if (right_cnt >= 170){
      Right_flag = false;
    }
  }
}

// Grayscale detection
void Grayscale_values(){
  Grayscale_middle_left = analogRead(GRAYSCALE2);
  Grayscale_middle_right = analogRead(GRAYSCALE4);
  Grayscale_middle = analogRead(GRAYSCALE3);
  Grayscale_left = analogRead(GRAYSCALE1);
  Grayscale_right = analogRead(GRAYSCALE5);

  Grayscale_combined = 0b00000;

  if (Grayscale_right > 880) { // right most grayscale sensor detect white line, lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b00001;
  }
  if (Grayscale_middle_right > 900) { // right second grayscale sensor detect white line, second lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b00010;
  }
  if (Grayscale_middle > 910) { // middle grayscale sensor detect white line, third lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b00100;
  }
  if (Grayscale_middle_left > 900) { // left second grayscale sensor detect white line, fourth lowest digit = 1
    Grayscale_combined = Grayscale_combined | 0b01000;
  }
  if (Grayscale_left > 900) { // left most grayscale sensor detect white line, highest digit = 1
    Grayscale_combined = Grayscale_combined | 0b10000;
  }

  if (Grayscale_combined == 0b01111 || 0b11110 || 0b11111){ // if more than three grayscale sensors detect white line
    Rotate_flag = true;
  }

  if (Grayscale_combined == 0b01110 || 0b11100 || 0b00111){ // if three grayscale sensors detect white line
    rotate_cnt += 1;
    if (rotate_cnt >= 80){
      Rotate_flag = false;
      rotate_cnt = 0;
    }
  }
}

void Esp8266_recv(){
  // if (arduinoSerial.available() > 0) {
  //     message = arduinoSerial.readString();
  //   Serial.println(message);
  //   Chassis_control.wifi_cmd = message;
  //   message = "";
  // }
}

//protocal: abcd
void Vision_recv(){
  if (Serial.available() > 0) {
    vision_message = Serial.readString();
    delay(1);
    // Vision.target_flag = vision_message[0].toint();
    // Vision.yaw_dir = vision_message[1].toint(); // 0123
    // Vision.catch_flag = vision_message[2].toint();

  if (Gimbal_control.target_flag == 0 && Vision.target_flag == 1){
    Gimbal_control.target_flag = 1;
  }
    Serial.println(message);
    vision_message = "";
  }
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

void Move(double x, double y, double z){ // control car movement by setting x, y, z
  Chassis_control.vx = x;
  Chassis_control.vy = y;
  Chassis_control.wz = z;
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
void Chassis_Motor_control(){
  
  if (Mode == Mode::TENNIS_DETECTED){
    if (Gimbal_control.target_flag == 1){
      Move(0.0, 0.0, 0.0);
    }
    switch (Chassis_control.target_dir){
      case 0:
        Move(0.0, 0.0, 0.5);
        if (Gimbal_control.target_flag == 3){
          Move(0.0, 0.0, 0.3);
        }
        break;
      case 1:
        Move(0.0, 0.0, -0.5);
        if (Gimbal_control.target_flag == 3){
          Move(0.0, 0.0, -0.3);
        }
        break;
      case 2:
        Move(0.0, 0.5, 0.0);

        break;
    }
  }

  Chassis_Vector_to_Mecanum_Wheel_Speed(Chassis_control.vx, Chassis_control.vy, Chassis_control.wz);

  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();
  motorPID4.Compute();
  motor1.pwm = (motor1.speed_set*1.3 + pidout1) / 2.4 * 255;
  motor2.pwm = (motor2.speed_set*1.3 + pidout2) / 2.4 * 255;
  motor3.pwm = (motor3.speed_set + pidout3) / 2.4 * 255;
  motor4.pwm = (motor4.speed_set + pidout4) / 2.4 * 255;
  CLIP(pwm1, -255, 255);
  CLIP(pwm2, -255, 255);
  CLIP(pwm3, -255, 255);
  CLIP(pwm4, -255, 255);
  motor1.setMotor(motor1.pwm);
  motor2.setMotor(motor2.pwm);
  motor3.setMotor(motor3.pwm);
  motor4.setMotor(motor4.pwm);
}

void Obstacle_avoidance(){
  // infrared 7.5 cm 
  // combine all infrared sensor states to one value
  // Serial.print("Infrared_combined: ");
  // Serial.println(Infrared_combined);
  // Serial.print("Front_flag: ");
  // Serial.println(Front_flag);
  // Serial.print("Left_flag: ");
  // Serial.println(Left_flag);
  // Serial.print("Right_flag: ");
  // Serial.println(Right_flag);
  // Serial.print("Infrared_combined: ");
  // Serial.println(Infrared_combined);
  // Serial.print("Front_flag: ");
  // Serial.println(Front_flag);
  // Serial.print("Left_flag: ");
  // Serial.println(Left_flag);
  // Serial.print("Right_flag: ");
  // Serial.println(Right_flag);
  // Serial.print("retrieve_flag: ");
  // Serial.println(retrieve_flag);
  // Serial.print("front_cnt: ");
  // Serial.println(front_cnt);
  // Serial.print("left_cnt: ");
  // Serial.println(left_cnt);
  // Serial.print("right_cnt: ");
  // Serial.println(right_cnt);

  switch (Infrared_combined) {
    case 0b0000: //no obstacle
      if (Front_flag && Left_flag && Right_flag){ 
        Move(0.0, -all_speed_set, 0.0);
      }
      else if (Front_flag && Left_flag && !Right_flag){
        Move(all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else if (Front_flag && Right_flag){
        Move(-all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else if (Front_flag){
        Move(-all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else {
        Move(0.0, all_speed_set, 0.0);
        if (retrieve_flag == 4){
          // delay(400);
          Move(all_speed_set, 0.0, 0.0);
        }
        else if (retrieve_flag == 3){
          // delay(400);
          Move(-all_speed_set, 0.0, 0.0);
        }
      }
      break;
    case 0b0010: //left, go straigt
      if (Front_flag && Right_flag){ 
        Move(0.0, -all_speed_set, 0.0);
      }
      else if (Front_flag){
        Move(all_speed_set, 0.0, 0.0);
      }
      else {
        Move(0.0, all_speed_set, 0.0);
        if (retrieve_flag == 2){
          retrieve_flag = 3;
        }
      }
      break;
    case 0b0100: //right, go straight
      if (Front_flag && Left_flag){ 
        Move(0.0, -all_speed_set, 0.0);
      }
      else if (Front_flag){
        Move(-all_speed_set, 0.0, 0.0);
      }
      else {
        Move(0.0, all_speed_set, 0.0);
        if (retrieve_flag == 2){
          retrieve_flag = 4;
        }
      }
      break;
    case 0b0110: //right and left, go straigt
      if (Front_flag){ 
        Move(0.0, -all_speed_set, 0.0);
      }
      else {
        Move(0.0, all_speed_set, 0.0);
      }
      break;
    case 0b1000: //back, go straight
    case 0b1010: //back and left, go straight
    case 0b1100: //back and right, go straight
    case 0b1110: //back, right and left, go straight
      Move(0.0, all_speed_set, 0.0);
      break;
    case 0b0001: //front left and front right, move towards left or right
      if (Left_flag && !Right_flag){
        Move(all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else if (Right_flag && !Left_flag){
        Move(-all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else if (!Left_flag && !Right_flag){
        Move(-all_speed_set, 0.0 , 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else { 
        Move(0.0, -all_speed_set, 0.0);
      }
      break;
    case 0b0011: //left and front right, move towards right // actually not possible?
      if (!Right_flag){
        Move(all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else {
        Move(0.0, -all_speed_set, 0.0);
      }
      break;
    case 0b0101: //right, front left and front right, move towards left
      if (!Left_flag){
        Move(-all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else {
        Move(0.0, -all_speed_set, 0.0);
      }
      break;
    case 0b0111: //right, left, front left and front right, move backward
      Move(0.0, -all_speed_set, 0.0);
      break;
    case 0b1001: //back and front right, move forward left
      if (Left_flag && !Right_flag){
        Move(all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else if (Right_flag && !Left_flag){
        Move(-all_speed_set, 0.0, 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else if (!Left_flag && !Right_flag){
        Move(-all_speed_set, 0.0 , 0.0);
        if (retrieve_flag == 1){
          retrieve_flag = 2;
        }
      }
      else { 
        Move(0.0, -all_speed_set, 0.0);
      }
      break;
    case 0b1101: //back, right, front left and front right, move towards left
      Move(-all_speed_set, 0.0, 0.0);
      if (retrieve_flag == 1){
        retrieve_flag = 2;
      }
      break;
    case 0b1111: //back, right, left, front left and front right, can only stop // actually not possible?
      Move(0.0, 0.0, 0.0);
      break;
    default:
      Move(0.0, all_speed_set, 0.0);
      break;
  }
  if (Chassis_control.vx != last_avoid_move_x || Chassis_control.vy != last_avoid_move_y || Chassis_control.wz != last_avoid_move_z){ 
    shift_stop_time = millis();
  }

}

void Line_tracking(){
  // gray scale detect 0.1 cm tolerance
  // combine all grayscale sensor states to one value

  switch (Grayscale_combined){
    case 0b00000: // no white line detected
    case 0b00010: // only middle right detect white, not court edge, ignore
    case 0b00100: // only middle detect white, not court edge, ignore
    case 0b00101: // middle and right most detect white, not court edge, ignore
    case 0b01000: // only middle left detect white, not court edge, ignore
    case 0b01001: // middle left and right most detect white, not court edge, ignore
    case 0b01010: // middle left and middle right detect white, not court edge, ignore
    case 0b01011: // middle left, middle right and right most detect white, not court edge, ignore
    case 0b01101: // middle left, middle and right most detect white, not court edge, ignore
    case 0b01110: // middle three detect white, court edge (middle part)
    case 0b10001: // left most and right most detect white, not court edge, ignore
    case 0b10010: // left most and middle right detect white, not court edge, ignore
    case 0b10011: // left most, middle right and right most detect white, not court edge, ignore
    case 0b10100: // left most and middle detect white, not court edge, ignore
    case 0b10101: // left most, middle and right most detect white, not court edge, ignore
    case 0b10110: // left most, middle right and middle detect white, not court edge, ignore
    case 0b11001: // left most, middle left and right most detect white, not court edge, ignore
    case 0b11010: // left most, middle left and middle right detect white, not court edge, ignore
    case 0b11011: // left most, middle left, middle right and right most detect white, not court edge, ignore
      Move(0.0, all_speed_set, 0.0); // go straight
      break;
    case 0b00001: // only right most detect white, may be court edge, move towards right
    case 0b00011: // right most and middle right detect white, may be court edge, move towards right
    case 0b00110: // middle and middle right detect white, may be court edge, move towards right
    case 0b00111: // middle, middle right and right most detect white, may be court edge, move towards right
    case 0b10111: // left most, middle, middle right and right detect white, may be court edge, move towards right
      Move(all_speed_set, 0.0, 0.0);
      retrieve_flag = 0;
      break;
    case 0b10000: // only left most detect white, may be court edge, move towards left
    case 0b11000: // left most and middle left detect white, may be court edge, move towards left
    case 0b11100: // left most, middle left and middle detect white, may be court edge, move towards left
    case 0b11101: // left most, middle left, middle and right most detect white, may be court edge, move towards left
      Move(-all_speed_set, 0.0, 0.0);
      retrieve_flag = 0;
      break;
    case 0b11110: // left most + middle three, court edge (left corner)
      if (Rotate_flag){
        Move(0.0, 0.0, -all_speed_set); // rotate acw 90 degree
      }
      // Rotate_ACW_90();
      break;
    case 0b01111: // middle three + right most detect white, court edge (right corner)
      if (Rotate_flag){
        Move(0.0, 0.0, all_speed_set); // rotate cw 90 degree
      }
    //   Rotate_CW_90();
      break;
    case 0b11111: // all detect white, court edge (crossroad)
      // Serial.println("Crossroad");
      if (Grayscale_left < Grayscale_right + 30){
        if (Rotate_flag){
          Move(0.0, 0.0, all_speed_set);
        }
        // Rotate_CW_90();
      }
      else {
        if (Rotate_flag){
          Move(0.0, 0.0, -all_speed_set);
        }
        // Rotate_ACW_90();
      }
    default:
      Move(0.0, all_speed_set, 0.0); // go straight
      break;
  }
  if (Chassis_control.move_flag == 1 && Gimbal_control.scan_cnt < 4){
    Move(0.0, 0.0, 0.0);
    return;
  }
  if (Chassis_control.move_cnt == 1000){
    Chassis_control.move_flag = 1;
    Gimbal_control.scan_cnt = 0;
  }

  Chassis_control.move_cnt += 1;
}

void Vision_tracking(){
  // vision detect from camera
}

void Arm_control(){
  // servo control
  // Sweep from 0 to 180 degrees:
  // for (angle = 0; angle <= 180; angle += 20) {
  //   ArmServo.write(angle);
  //   delay(500);
  // }
  // ArmServo.write(0);
  // delay(1500);
  // ArmServo.write(90);
  // delay(1500);
  // ArmServo.write(180);
  // delay(1500);
  // ArmServo.write(90);
  // delay(1500);
  // ArmServo.write(0);
  ArmServo.write(60);
  delay(1500);
  ArmServo.write(95);
  delay(1500);
}

void Scanning(){
  if (Gimbal_control.cnt % 400 == 0){
    Gimbal_control.yaw += 45;
    Gimbal_control.yaw %= 180;
    YawServo.write(Gimbal_control.yaw);
    Gimbal_control.scan_cnt += 1;
    Gimbal_control.cnt = 0;
  }
  // PitchServo.write(Gimbal_control.pitch);
  Gimbal_control.cnt += 1;
}

void Gimbal_motor_control(){
  // servo control
  if(Gimbal_control.scan_cnt < 4){
    Scanning();
  }
  else if (Gimbal_control.scan_cnt == 4){
    Gimbal_control.yaw = 90;
    YawServo.write(Gimbal_control.yaw);
    Gimbal_control.cnt = 0;
    Chassis_control.move_flag = 0;
    Chassis_control.move_cnt = 0;
    Gimbal_control.scan_cnt += 1;
  }

  if (Mode == Mode::TENNIS_DETECTED){
    if (Gimbal_control.target_flag == 1){ // first detection of tennis ball, set target for chassis to rotate and reset yaw
      Gimbal_control.target = YawServo.read();
      Gimbal_control.target_flag = 2;
      YawServo.write(90);
      delay(100);
    }
    else if (Gimbal_control.target_flag == 2 && Vision.target_flag == 1){ // second detection of tennis ball, set target direction directly using vision data
      Chassis_control.target_dir = Vision.yaw_dir;
      YawServo.write(90);
      Gimbal_control.target_flag = 3;
      return;
    }
    else if (Gimbal_control.target_flag == 3){
      Chassis_control.target_dir = Vision.yaw_dir;
      return;
    }
    
    if (Gimbal_control.target < 88){
      Chassis_control.target_dir = 0;
    }
    else if (Gimbal_control.target > 92){
      Chassis_control.target_dir = 1;
    }
    else {
      Chassis_control.target_dir = 2;
    }
  }
  
}

void Mode_switch(){
  // execute functions under different modes
  switch (Mode){
    case Mode::NORMAL:
      if (Obstacle_flag){
        Mode = Mode::OBSTACLE_DETECTED;
        last_time = millis();
        break;
      }
      else if (Tennis_flag){
        Mode = Mode::TENNIS_DETECTED;
      }
      Line_tracking();
      Vision_tracking();
      Gimbal_motor_control();
      break;
    case Mode::OBSTACLE_DETECTED:
      if (!Obstacle_flag && retrieve_flag == 0){
        Mode = Mode::NORMAL;
        break;
      }
      // else if (Tennis_flag){
      //   Mode = Mode::TENNIS_DETECTED;
      //   break;
      // }
      Line_tracking();
      Obstacle_avoidance();
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
      Line_tracking();
      Vision_tracking();
      Arm_control();
      Gimbal_motor_control();
      break;
    default:
      break;
  }
}


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
  // Serial.print("vx: ");
  // Serial.println(Chassis_control.vx);
  // Serial.print("vy: ");
  // Serial.println(Chassis_control.vy);
  // Serial.print("wz: ");
  // Serial.println(Chassis_control.wz);

  // Serial.println("M1 pwm:");
  // Serial.println(motor1.pwm);
  // Serial.println("M2 pwm:");
  // Serial.println(motor2.pwm);
  // Serial.println("M3 pwm:");
  // Serial.println(motor3.pwm);
  // Serial.println("M4 pwm:");
  // Serial.println(motor4.pwm);

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

  // gray scale
  // Serial.print("Grayscale_left: ");
  // Serial.println(Grayscale_left);
  // Serial.print("Grayscale_middle_left: ");
  // Serial.println(Grayscale_middle_left);
  // Serial.print("Grayscale_middle: ");
  // Serial.println(Grayscale_middle);
  // Serial.print("Grayscale_middle_right: ");
  // Serial.println(Grayscale_middle_right);
  // Serial.print("Grayscale_right: ");
  // Serial.println(Grayscale_right);

  // MOTORA_FORWARD(0);
  // MOTORB_FORWARD(0);
  // MOTORC_FORWARD(0);
  // MOTORD_FORWARD(0);
  
  // gimbal
  // servo1.write(180);
  // Serial.println(YawServo.read());
  // Serial.println(PitchServo.read());
  // Serial.print("Scan_cnt: ");
  // Serial.println(Gimbal_control.scan_cnt);
  // Serial.print("move_cnt: ");
  // Serial.println(Chassis_control.move_cnt);
  // Serial.print("move_flag: ");
  // Serial.println(Chassis_control.move_flag);

  // arm servo
  // ArmServo.write(0);
  // delay(2000);
  // ArmServo.write(180);
  // delay(2000);

}

void loop()
{
  time = millis();
  Data_update();
  // Mode_switch();
  // Obstacle_avoidance();
  // Line_tracking();
	// Vision_tracking();
  // Gimbal_motor_control();
  Chassis_Motor_control();
	Arm_control();

  //debug
  debug();

  delay(1);
}