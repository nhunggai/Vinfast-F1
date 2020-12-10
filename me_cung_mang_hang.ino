#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include "kmotor.h"
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27,16,2);
kmotor _kmotor(true);

int initial_motor_speed = 200;

int pre=0,dem=0,times=360;// chinh times de quay dung 90 do moi lan chinh nap lai code + clean banh 
int disFront,disLeft,disRight,disBack;
float previous_error = 0, previous_I = 0;
int lastError=0;
int turn[50]={2,0,2,0,2,0,2,2,0,0,2,0,2,0};// mang hang 0 la re trai 1 la di thang 2 la re phai 3 la di lui moi khi gap tuong
//{2,0, 0,0,0,0,  2,0,2,0, 0,2, 2,2,0, 2,0,0,2,0,0,0,0,2,0,0};

float Kp=10,Kd=30,Ki=0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;

void thang(){
    _kmotor.tien(0, 200);
    _kmotor.tien(1, 200);
}
void trai(){
    _kmotor.tien(0, -100);
    _kmotor.tien(1, 100); 
}
void phai(){
    _kmotor.tien(0, 100);
    _kmotor.tien(1, -100);
}
void lui(){
    _kmotor.tien(0, -200);
    _kmotor.tien(1, -200);
}


void setup() {
  _kmotor.cauhinh();
  Serial.begin(9600);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  disBack=getDistance(4,5);
  while(disBack>=3){
    disBack=getDistance(4,5);
    _kmotor.stop();
  }
}

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

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);
  //Serial.println(left_motor_speed);
  //Serial.println(right_motor_speed);
  _kmotor.tien(0, left_motor_speed);//M1
  _kmotor.tien(1, right_motor_speed);//M2
}

void bam_trai(){
  error=disLeft-8;
  calculate_pid();
  motor_control();
}

void bam_phai(){
  error=8-disRight;
  calculate_pid();
  motor_control();
}

void real_thang(){
  if(disLeft>=10&&disRight>=10){
     _kmotor.run(0, 200);
     analogWrite(A3,300);//red
  }else if(disLeft<10){
     bam_trai();
     analogWrite(A2,300);//green
  }else{
    bam_phai();
    analogWrite(A1,300);//blue
  }
}

void tinh(){
    disFront=getDistance(2,12);
    disLeft=getDistance(9,10);
    disRight=getDistance(11,13);
    disBack=getDistance(4,5);
    
    Serial.println(disFront);
    Serial.println(disLeft);
    Serial.println(disRight);
    Serial.println(disBack);
    if(disFront>11){//sap dam vao tuong
        real_thang();
    }else{
      _kmotor.stop();
      delay(50);
      if(turn[dem]==0){
        trai();delay(times);_kmotor.stop();
        //while(!(disFront>10)){trai();disFront=getDistance(2,12);}
      }
      if(turn[dem]==1){
        thang();delay(times);_kmotor.stop();
        //while(!(disFront>10)){thang();disFront=getDistance(2,12);}
      }
      if(turn[dem]==2){
        phai();delay(times);_kmotor.stop();
        //while(!(disFront>10)){phai();disFront=getDistance(2,12);}
      }
      if(turn[dem]==3){
        lui();delay(times);_kmotor.stop();
        //while(!(disFront>10)){lui();disFront=getDistance(2,12);}
      }
      dem=dem+1;
    }
    analogWrite(A1,0);
    analogWrite(A2,0);
    analogWrite(A3,0);
    
}

void loop(){
    tinh();// nho clean banh moi lan thu
}
