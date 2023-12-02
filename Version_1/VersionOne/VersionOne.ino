/**
* @file: ManualPhaseParkAssistV1_ESP32.ino
* @date: November 29, 2023
* @author: Ahmed Samy Elnozahy
*
* @brief: Code for Version-One to our project:
*         AutoPilot-ParkAssist 
*         using HC-05 Bluetooth Module and L298d MotorDriver
*         and Encoder and couple of ultrasonics
*         
**/

//#include <NewPing.h>
#include "Wire.h"
#include <MPU6050_light.h>
#define echoN1  34
#define trigN1  32

#define echoN2  35
#define trigN2  33

#define echoN3  19
#define trigN3  18

#define Buzzer  15

/* Pin definitions for motor control */
const int IN1 = 26;
const int IN2 = 27;

const int IN3 = 14;
const int IN4 = 12;

/* Motor enable pins (to be adjusted in future) */
const int enablem1Pin3 = 25;
const int enablem2Pin3 = 25;

#define speed 130



long timer = 0;
int distBehind;
int distForw;
int distBack;
int timB, tim2, timF;

volatile long encoderCount = 0;
volatile float distance = 0.0;

MPU6050 mpu(Wire);

byte serialA;

/* Encoder pin for one wheel */
const int encoderPin = 15; // Change this to the appropriate pin for your setup

/* Variables for encoder */
const float wheelDiameter = 65.0; // in millimeters

void Motors_Move(int serialMsg);

void setup()
{
  /*For Debugging - Serial to Computer to Print*/
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  pinMode(echoN1, INPUT);
  pinMode(trigN1, OUTPUT);
  pinMode(echoN2, INPUT);
  pinMode(trigN2, OUTPUT);
  pinMode(echoN3, INPUT);
  pinMode(trigN3, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoder, RISING);
  IMU_Setup();
}

void loop()
{
  //for(int i = 0; i < 230 ; i++)
  // {
  //   if (Serial.available())
  //   {
  //     serialA = Serial.read();
  //   }


  // Motors_Move(serialA);
  // }

  ultrasonic();
  IMU();
}

void ultrasonic()
{
  // Start with Fist Sensor (Behind the car)
  // __________________________________
  digitalWrite(trigN2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigN2, LOW);

  /* Measure the time taken for the ultrasonic pulse to return */
  timF = pulseIn(echoN2, HIGH);

  distForw = timF * 0.034 / 2;

  if (distForw < 12)
  {
    digitalWrite(Buzzer, HIGH);
    //tone(Buzzer, 1200); // Beep at 1000 Hz
    if (distForw < 10)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      delay(2000);
    }
  }
  else
  {
    digitalWrite(Buzzer, LOW);
  }

  delay(70);

  digitalWrite(trigN1, LOW);
  delay(5);
  digitalWrite(trigN1, HIGH);
  delay(10);
  digitalWrite(trigN1, LOW);

  tim2 = pulseIn(echoN1, HIGH);
  /*  Calculate distance in centimeters */

  distBehind = tim2 * 0.034 / 2;
  // __________________________________

  // Logic
  if (distBehind < 9)
  {
    tone(Buzzer, 3000); // Beep at 1000 Hz
  }
  else
  {
    noTone(Buzzer);
  }

  delay(70);


  digitalWrite(trigN3, LOW);
  delay(5);
  digitalWrite(trigN3, HIGH);
  delay(10);
  digitalWrite(trigN3, LOW);

  timB = pulseIn(echoN3, HIGH);
  /*  Calculate distance in centimeters */

  distBack = timB * 0.034 / 2;
  // __________________________________

  // Logic
  if (distBack < 9)
  {
    digitalWrite(Buzzer, HIGH);// Beep at 1000 Hz
  }
  else
  {
    noTone(Buzzer);
  }

  delay(70);


  Serial.print("D1: ");
  Serial.println(distForw);
  Serial.print("D2: ");
  Serial.println(distBehind);
  Serial.print("D3: ");
  Serial.println(distBack);
  
}

void Motors_Move(int serialMsg)
{
  /* Motor control based on received Bluetooth commands */
  switch (serialMsg)
  {
  /* forward */
  case 'F':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /* left */
  case 'L':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /* right */
  case 'R':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /* forward left */
  case 'G':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /* forward right */
  case 'I':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /*  backward left */
  case 'H':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /*  backward right */
  case 'J':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /*  backward */
  case 'B':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed);
    analogWrite(enablem2Pin3, speed);
    break;

  /*  Stop */
  case 'S':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

/********************************************************************************************/
/****                                      Encoder                                      *****/
/********************************************************************************************/
void handleEncoder()
{
  // Update the encoder count based on the direction of rotation
  encoderCount++;
  // Update distance based on the number of encoder pulses
  updateDistance();
}

/* Update distance based on the number of encoder pulses */
void updateDistance()
{
  float circumference = PI * wheelDiameter; // in millimeters
  distance = (encoderCount / 20.0) * circumference; // Each pulse represents 1/20th of a revolution
}

/* Print the distance traveled */
void printDistance()
{
  Serial.print("D: ");
  Serial.print(distance * 0.01);
  Serial.println(" cm");
}



/********************************************************************************************/
/****                                      mpu                                      *****/
/********************************************************************************************/
void IMU_Setup(){
    Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}
void IMU() {
  mpu.update();

  if(millis() - timer > 1000){ // print data every second

    Serial.print(F("ANGLE Z: "));
    Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }

}
