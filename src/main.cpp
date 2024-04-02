#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
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
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 6   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

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
#ifndef DCMOTOR_HPP
#define DCMOTOR_HPP

#include <Arduino.h>

class DCMotor {
  private:
      int pwmPin; // PWM pin for motor speed control
      int dirAPin;
      int dirBPin;

  public:
      int encoderAPin;
      int encoderBPin;
      double encoderValue; // Encoder value

      DCMotor(int pwm, int dirA, int dirB, int encoderA, int encoderB);
      void setMotor(int analogSpeed);
      void setSpeed(int analogSpeed);
      void setDirection(int dir);
      void encoderSubroutineA();
      void encoderSubroutineB();
};

#pragma once

const double EPRA = 660;//转速比：1：660
const double EPRB = 660;//转速比：1：660
const double EPRC = 660;//转速比：1：660
const double EPRD = 660;//转速比：1：660

const int pwm1 = 12; const int dir1A = 34; const int dir1B = 35; const int encoder1A = 18; const int encoder1B = 31;
const int pwm2 = 8; const int dir2A = 37; const int dir2B = 36; const int encoder2A = 19; const int encoder2B = 38;
const int pwm3 = 6; const int dir3A = 43; const int dir3B = 42; const int encoder3A = 3; const int encoder3B = 49;
const int pwm4 = 5; const int dir4A = A4; const int dir4B = A5; const int encoder4A = 2; const int encoder4B = A1;

const double pi = 3.14159265358979323846;

double eps1 = 0, eps2 = 0, eps3 = 0, eps4 = 0;// encoder count per second
double eps1_fb = 0, eps2_fb = 0, eps3_fb = 0, eps4_fb = 0;// eps filtered feedback
double power1 = 10, power2 = 10, power3 = 10, power4 = 10;

MovingAverage filter1; MovingAverage filter2; MovingAverage filter3; MovingAverage filter4;

DCMotor motor1(pwm1, dir1A, dir1B, encoder1A, encoder1B); DCMotor motor2(pwm2, dir2A, dir2B, encoder2A, encoder2B);
DCMotor motor3(pwm3, dir3A, dir3B, encoder3A, encoder3B); DCMotor motor4(pwm4, dir4A, dir4B, encoder4A, encoder4B);

#endif // DCMOTOR_HPP


DCMotor::DCMotor(int pwm, int dirA, int dirB, int encoderA, int encoderB) : 
pwmPin(pwm), dirAPin(dirA), dirBPin(dirB), encoderAPin(encoderA),
encoderBPin(encoderB), encoderValue(0) {
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
void DCMotor::encoderSubroutineA() {
    if (digitalRead(encoderAPin)){
        if (digitalRead(encoderBPin)){
            encoderValue ++;
        }
        else{
            encoderValue --;
        }
    }
    else{ 
        if (digitalRead(encoderBPin)){
            encoderValue --;
        }
        else{
            encoderValue ++;
        }
    }
}
void DCMotor::encoderSubroutineB() {
    if (digitalRead(encoderBPin)){
        if (digitalRead(encoderAPin)){
            encoderValue ++;
        }
        else{
            encoderValue --;
        }
    }
    else{ 
        if (digitalRead(encoderAPin)){
            encoderValue --;
        }
        else{
            encoderValue ++;
        }
    }
}



////////////////////////////////////////////////////////////
void AUTO_Control();
void Alignment();
void Rotation();
void MoveAndRotate();
void Measurement();
void Parking();
void DataUpdate();
void calibrate();


  // sonar
  #define trigPinL 29
  #define trigPinR 25
  #define echoPinL 30
  #define echoPinR 28
  long L_sonar_dist,R_sonar_dist;
  int done = 1;
  unsigned long start_time = 0;
  int alignemnt_cnt = 0;

  //gyro
  MPU6050 mpu(Wire);
  float pitch, roll, yaw, angle_set;

  //enum class for the state machine
  enum class MOVEMENTTYPE{
    LOCATING,
    NEXTSTAGE_TRANSITION,
    ROTATING1,
    ROTATING2,
    ROTATING3,
    END_MOVEMENT
  };
  enum class TASKTYPE{
    INIT,
    ALIGNMENT,
    MOVEANDROTATE,
    MEASUREMENT,
    PARKING,
    END_STATE
  };
  enum class PRAKINGSTATE{
    FORWARD,
    TRANSIT,
    ROTATE,
    END_PARKING
  };
  TASKTYPE STATE = TASKTYPE::INIT;
  MOVEMENTTYPE MOVEMENT = MOVEMENTTYPE::LOCATING;
  PRAKINGSTATE PARKING = PRAKINGSTATE::FORWARD;

  // alignment
  int flag = 0;
  int locating_cnt = 0;

  // light
  #define TOL   50           // tolerance for adc different, avoid oscillation
  #define K   5              // Step size
  #define LightPinL A8
  #define LightPinR A9
  //variables for light intensity to ADC reading equations 
  int int_adc0, int_adc0_m, int_adc0_c;
  int int_adc1, int_adc1_m, int_adc1_c;
  int int_left, int_right, avg_light_intensity;

  //measurment
  float distance_to_wall_buf, angle_of_vehicle_buf, distance_to_wall, angle_of_vehicle;
  int cnt = 0;

  //parking
  float max_light_intensity = 0;
  float pitch_at_max_light_intensity = 0;

  // PWM
  uint8_t motion_mode;
  uint8_t  motion_last_mode;
  int Motor_PWM = 0;
  #define MOTION_MODE_ADVANCE 0
  #define MOTION_MODE_BACK 1
  #define MOTION_MODE_LEFT 2
  #define MOTION_MODE_RIGHT 3
  #define MOTION_MODE_ROTATE 4
  #define MOTION_MODE_STOP 5
  #define MOTION_MODE_ROTATE_CW 6
  #define MOTION_MODE_ROTATE_CCW 7
  #define MOTION_MODE_ROTATE_180 8
  #define MOTION_MODE_ROTATE_90 9
  #define MOTION_MODE_ROTATE_270 10

void cmd_vel_cb(const geometry_msgs::Twist& vel_msg){ // x // y // z
    delay(10);
    motorPID1.outputSum = 0;
    motorPID2.outputSum = 0;
    motorPID3.outputSum = 0;
    motorPID4.outputSum = 0;
    motor1.setDirection(0);motor2.setDirection(0);motor3.setDirection(0);motor4.setDirection(0);
    mecanumDrive.calculateWheelSpeeds(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
    set_eps1 = EPRA * mecanumDrive.wheelSpeeds[0] / 2 / pi; 
    set_eps2 = EPRB * mecanumDrive.wheelSpeeds[1] / 2 / pi; 
    set_eps3 = EPRC * mecanumDrive.wheelSpeeds[2] / 2 / pi; 
    set_eps4 = EPRD * mecanumDrive.wheelSpeeds[3] / 2 / pi;
}



/*
  ABOVE BY GROUP C4-C
*/
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE( )
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1( )
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2( )
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3( )
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1( )
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2( )
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3( )
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1( )  // CW
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2( )  //CCW
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP( )
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }
  }







  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}



/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0);
    if(newV!=oldV) {
      if (!Serial3.available()) {
        Serial3.println(newV);
        Serial.println(newV);
      }
    }
    oldV=newV;
}



//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
   servo_pan.attach(48);
   servo_tilt.attach(47);
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


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
  /*
    BELOW BY GROUP C4-C
  */
  pinMode(echoPinL, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinR, OUTPUT);

  calibrate();
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
/*
  BELOW BY GROUP C4-C
*/
void calibrate() {
    Serial.println("*******************");
    Serial.println("START AUTO_Control");
    Serial.println("*******************");
    Serial.println("Calibrating the sensors...");
    int_adc0=analogRead(LightPinL);   // Left sensor at ambient light intensity
    int_adc1=analogRead(LightPinR);   // Right sensor at ambient light intensity
    delay(500);
    int_adc0_c=analogRead(LightPinL);   // Left sensor at zero light intensity
    int_adc1_c=analogRead(LightPinR);   // Right sensor at zero light intensity
    // calculate the slope of light intensity to ADC reading equations  
    int_adc0_m=(int_adc0-int_adc0_c)/100;
    int_adc1_m=(int_adc1-int_adc1_c)/100;
    //calibrate the gyroscope
    Serial.begin(115200); 
    Wire.begin();
    mpu.begin();
    Serial.println("\nCalculating gyro offset, do not move MPU6050");
    Serial.println("............");
    delay(1000);         //delay 1000ms waiting for calibration of gyroscope to complete
    Serial.println("Done\n*******************");
    STATE = TASKTYPE::ALIGNMENT;
}

void get_gyro() {
    //once call Dataupdate() update the gyroscope info 
    mpu.update();
    pitch = mpu.getAngleX();
    roll = mpu.getAngleY();
    yaw = mpu.getAngleZ();
}

void get_distance() {
    long durationL;
    long durationR;
    if (done) {
        // reset start_time only if the distance has been measured 
        // in the last invocation of the method
        done = 0;
        start_time = millis();
        digitalWrite(trigPinL, LOW);
        digitalWrite(trigPinR, LOW);
    }
    
    if (millis() > start_time + 2) { 
        digitalWrite(trigPinL, HIGH);
        digitalWrite(trigPinR, HIGH);
    }
    
    if (millis() > start_time + 10) {
        digitalWrite(trigPinL, LOW);
        durationL = pulseIn(echoPinL, HIGH);
        L_sonar_dist = (durationL / 2.0) / 29.1;
        digitalWrite(trigPinR, LOW);
        durationR = pulseIn(echoPinR, HIGH);
        R_sonar_dist = (durationR / 2.0) / 29.1 - 1.1;
        done = 1;
    }

}

void get_light() {
  // calculate the light intensity of the sensors
  // in the range of [0, 100]
  int_left=(analogRead(LightPinL)-int_adc0_c)/int_adc0_m;
  int_right=(analogRead(LightPinR)-int_adc1_c)/int_adc1_m;
  avg_light_intensity = (int_left + int_right) / 2;


}

//TODO
void Alignment(){
  
    DataUpdate();
    if (alignemnt_cnt >= 5){
      STOP();
      delay(2000);
      return;
    }
    if (abs(L_sonar_dist - R_sonar_dist) < 1.4){
      STOP();
      if (flag == 0)
        alignemnt_cnt = 0;
      flag = 1;
      Serial.println("Alignment done!");
      alignemnt_cnt++;
      if (alignemnt_cnt >= 5){
        STOP();
        STATE = TASKTYPE::MOVEANDROTATE;
        return;
      }
    }
    else if (L_sonar_dist - R_sonar_dist > 1.4){
        flag = 0;
        Motor_PWM = 300;
        rotate_2();
    }
    else if (L_sonar_dist - R_sonar_dist < -1.4){
        flag = 0;
        Motor_PWM = 300;
        rotate_1();
    }

    delay(150);
    STOP();
}

void Rotation(){
  if (avg_light_intensity > max_light_intensity){
    max_light_intensity = avg_light_intensity;
    pitch_at_max_light_intensity = pitch;
  }
  if (abs(pitch - angle_set) > 0.5){
    if (pitch < angle_set){
      Motor_PWM = 500;
      rotate_2();
      delay(100);
      STOP();
      }
    else if (pitch > angle_set){
      Motor_PWM = 500;
      rotate_1();
      delay(100);
      STOP();
      }
  }
  else 
    STOP();
}


/**   *@TODO: PWM control   **/
void MoveAndRotate(){
  DataUpdate();
  switch (MOVEMENT)
  {
  case MOVEMENTTYPE::LOCATING:
    Serial.println("MOVEMENT: LOCATING");
    Serial.println(locating_cnt);
    if (locating_cnt >= 5){
      STOP();
      MOVEMENT = MOVEMENTTYPE::NEXTSTAGE_TRANSITION;
      return;
      }
    if (((24.0 < L_sonar_dist) && (L_sonar_dist < 26.0)) && ((24.5 < R_sonar_dist) && (R_sonar_dist < 25.5))){
      STOP();
      Serial.println("Locating done!");
      locating_cnt++;
    }
    else if ((L_sonar_dist > 26.0) && (R_sonar_dist > 26.0)){
      locating_cnt = 0;
      if ((L_sonar_dist > 35.0) && (R_sonar_dist > 35.0)){
        Motor_PWM = 1200;
        ADVANCE(230, 230, 230, 230);
        }
      else {
        Motor_PWM = 300;
        ADVANCE(230, 230, 230, 230);
        }
    }
    else if ((L_sonar_dist < 24.0) && (R_sonar_dist < 24.0)){
      locating_cnt = 0;
      Motor_PWM = 300;
      BACK(230, 230, 230, 230);
    }
    else if (L_sonar_dist - R_sonar_dist > 1.3){
      locating_cnt = 0;
      Motor_PWM = 300;
      rotate_2(230, 230, 230, 230);
      delay(200);
      STOP();
    }
    else if (L_sonar_dist - R_sonar_dist < -1.3){
      locating_cnt = 0;
      Motor_PWM = 300;
      rotate_1(230, 230, 230, 230);
      delay(200);
      STOP();
    }
    break;

  case MOVEMENTTYPE::NEXTSTAGE_TRANSITION:
    Serial.println("MOVEMENT: NEXTSTAGE_TRANSITION");
    STOP();
    delay(2000);
    MOVEMENT = MOVEMENTTYPE::ROTATING1;
    break;

  /**    @TODO: calibrate the light sensors during the rotation   **/ 
  case MOVEMENTTYPE::ROTATING1:
    Serial.println("MOVEMENT: ROTATING1");
    // CW 90
    angle_set = pitch-90.0;
    Motor_PWM = 750;
    Rotation();
    if (abs(pitch - angle_set) < 1.0){
      STOP();
      delay(2000);
      MOVEMENT = MOVEMENTTYPE::ROTATING2;
    }
    break;
  case MOVEMENTTYPE::ROTATING2:
    Serial.println("MOVEMENT: ROTATING2");
    // CCW 270
    angle_set = pitch+270.0;
    Motor_PWM = 750;
    Rotation();
    if (abs(pitch - angle_set) < 1.0){
      STOP();
      delay(2000);
      // ambient light intensity as dark intensity
      int_adc0_c=analogRead(LightPinL);   // Left sensor at zero light intensity
      int_adc1_c=analogRead(LightPinR);   // Right sensor at zero light intensity
      MOVEMENT = MOVEMENTTYPE::ROTATING3;
    }
    break;
  case MOVEMENTTYPE::ROTATING3:
    Serial.println("MOVEMENT: ROTATING3");
    // CW 180
    angle_set = pitch-180.0;
    Motor_PWM = 750;
    Rotation();
    if (abs(pitch - angle_set) < 1.0){
      STOP();
      delay(2000);
      // light intensity as ambient light intensity
      int_adc0=analogRead(LightPinL);   // Left sensor at ambient light intensity
      int_adc1=analogRead(LightPinR);   // Right sensor at ambient light intensity
      int_adc0_m=(int_adc0-int_adc0_c)/100;
      int_adc1_m=(int_adc1-int_adc1_c)/100;
      MOVEMENT = MOVEMENTTYPE::END_MOVEMENT;
    }
    break;  
  case MOVEMENTTYPE::END_MOVEMENT:
    Serial.println("MOVEMENT: END_MOVEMENT");
    STOP();
    STATE = TASKTYPE::MEASUREMENT;
    break;
  default:
    break;
  }
}

void Measurement(){
  if (cnt == 10){
    delay(2000); // wait 2s after the measurement
    distance_to_wall = distance_to_wall_buf / cnt;
    angle_of_vehicle = angle_of_vehicle_buf / cnt;
    STATE = TASKTYPE::PARKING;
    return;
  }
  STOP();
  DataUpdate();
  // measure the distance from the sonar sensors
  distance_to_wall_buf += (L_sonar_dist + R_sonar_dist) / 2;
  // measure the angle of the vehicle to the wall
  angle_of_vehicle_buf += (L_sonar_dist - R_sonar_dist) / 2;
  cnt++;
}

void Parking(){
  // park the robot
  //TODO
  DataUpdate();
  switch (PARKING)
  {
  case PRAKINGSTATE::FORWARD:
    Serial.println("PARKING: FORWARD\n*******************");
    if (distance_to_wall > 5.1){
      Motor_PWM = 500;
      ADVANCE();
      }
    else if  (distance_to_wall < 4.9)
      BACK(300, 300, 300, 300);
    else{
      STOP();
      delay(1000);
      PARKING = PRAKINGSTATE::TRANSIT;
    }
    break;
  case PRAKINGSTATE::TRANSIT:
    Serial.println("PARKING: TRANSIT\n*******************");
    if (int_left > int_right ){
      Motor_PWM = 300;
      LEFT_1();
    }
    else if (int_left < int_right){
      Motor_PWM = 300;
      RIGHT_1();
    }
    else{
      STOP();
      // PARKING = PRAKINGSTATE::ROTATE;
      PARKING = PRAKINGSTATE::END_PARKING;
    }
    break;
  // TBD may not be necessary
  case PRAKINGSTATE::ROTATE:
    // // rotate the robot to the direction of the light source
    // if (){
    //   rotate_1(500, 500, 500, 500);
    // }
    // else if (){
    //   rotate_2(500, 500, 500, 500);
    // }
    // else{
    //   STOP();
    //   PARKING = PRAKINGSTATE::END_PARKING;
    // }
    break;
  case PRAKINGSTATE::END_PARKING:
    Serial.println("PARKING: END_PARKING\n*******************");
    STOP();
    STATE = TASKTYPE::END_STATE;
    delay(2000);
    break;
  }

}

void DataUpdate(){
  // update the data
  // get the distance from the sonar sensors
  get_distance();
  // get the light intensity from the light sensors
  get_light();
  // get the gyroscope axis x, y and z information
  get_gyro();

  //debug
  Serial.print("Left sonar distance = ");
  Serial.print(L_sonar_dist);
  Serial.print(";  Right sonar distance = ");
  Serial.println(R_sonar_dist);

  Serial.print("PWM: ");
  Serial.println(Motor_PWM);

  // Serial.print("Left sensor intensity = ");
  // Serial.print(int_right);
  // Serial.print(";  Right sensor intensity = ");
  // Serial.println(int_left);

  Serial.print("Pitch (Angle_X) : ");
  Serial.print(pitch);
  Serial.print("  Roll  (Angle_Y) : ");
  Serial.print(roll);
  Serial.print("  Yaw   (Angle_Z) : ");
  Serial.println(yaw);

  // BT Serial
  if (!Serial3.available()){
  Serial3.print("\nL_sonar_dist: ");
  Serial3.print(L_sonar_dist);
  Serial3.print(",");
  Serial3.print("R_sonar_dist: ");
  Serial3.println(R_sonar_dist);
  
  Serial3.print("\nLeft_light: ");
  Serial3.print(int_left);
  Serial3.print(",");
  Serial3.print("Right_light: ");
  Serial3.println(int_right);

  Serial3.print("\nGyro_pitch: ");
  Serial3.print(pitch);
  Serial3.print(",");
  Serial3.print("Gyro_roll: ");
  Serial3.print(roll);
  Serial3.print(",");
  Serial3.print("Gyro_yaw: ");
  Serial3.println(yaw);
  }
}

void AUTO_Control(){
//   // Basic structure of the AUTO_Control Start
  switch (STATE)
  {
    case TASKTYPE::INIT:
      Serial.print("*******************\nSTATE: INIT\n");
      break;
    case TASKTYPE::ALIGNMENT: 
      Serial.print("*******************\nSTATE: Alignment\n");
      Alignment();
      break;
    case TASKTYPE::MOVEANDROTATE:
      Serial.print("*******************\nSTATE: MoveAndRotate\n");
      MoveAndRotate();
      break;
    case TASKTYPE::MEASUREMENT:
      Serial.print("*******************\nSTATE: Measurement\n");
      Measurement();
      break;
    case TASKTYPE::PARKING:
      Serial.print("*******************\nSTATE: Parking\n");
      Parking();
      break;
    case TASKTYPE::END_STATE:
      STOP();
      break;
  default:
    STOP();
    break;
  }
  // delay(100);
  // Basic structure of the AUTO_Control End
}

/*
  ABOVE BY GROUP C4-C  
*/
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
//    UART_Control(); //get USB and BT serial data

    AUTO_Control();

    //constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);
    
    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);
  }if (voltCount>=5){
    voltCount=0;
    sendVolt();
  }
}