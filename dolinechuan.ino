#include <AFMotor.h>    //Adafruit Motor Shield Library. First you must download and install AFMotor library
#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library
#include <Arduino.h>
#include "kmotor.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

kmotor _kmotor(true);

#define MIDDLE_SENSOR 4       //number of middle sensor used
#define NUM_SENSORS 6         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define DEBUG 1

//sensors 2 through 7 are connected to analog inputs 0 through 5, respectively
QTRSensorsRC qtrrc((unsigned char[]) { A0,A1,A2,A3,A4,A5} ,NUM_SENSORS);
  
unsigned int sensorValues[NUM_SENSORS];
int dem=0;
int initial_motor_speed = 240;
float Kp=5,Kd=20,Ki=0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0,pre_pre_error=0, previous_I = 0;

float getDistance(int trig,int echo){
  float dem=0;
    pinMode(trig,OUTPUT);
    digitalWrite(trig,LOW);
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    pinMode(echo, INPUT);
    dem = pulseIn(echo,HIGH,30000)/58.0;
    if(dem==0) dem=30;
    return dem;
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {
  int i;
  for (i = 0; i < 500; i++)
  {
    qtrrc.calibrate();
    delay(10);
  }
    
  if (DEBUG) {
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
    }
    Serial.println();
      
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    }
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

  
void setup()
{
  _kmotor.cauhinh();
  Serial.begin(57600);
  pinMode(11,INPUT_PULLUP);
  manual_calibration();
}
  
void loop()
{
  Serial.println(digitalRead(11));
  if(getDistance(2,12)<=3){
    _kmotor.stop();
    delay(10000); //10 s cho qua hai ben cua line
  }
  if(digitalRead(11)==1){
    unsigned int sensors[5];
    int position = qtrrc.readLine(sensors); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
    if(position==5000||position==0){
      dem=1;
    }else{
      dem=-1;
    }
    if(dem==1){
      Kp=10,Kd=50,Ki=5;
      initial_motor_speed = 200;
      dem=0;
    }
    if(dem==-1){
      Kp=10,Kd=40,Ki=0;
      initial_motor_speed = 250;
      dem=0;
    }
    error=(position-2500)/100;
    calculate_pid();
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;
  
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);
    //Serial.println(left_motor_speed);
    //Serial.println(right_motor_speed);
    _kmotor.tien(0, left_motor_speed);//M1
    _kmotor.tien(1, right_motor_speed);//M2
  }else{
    _kmotor.stop();
  }
}
  
