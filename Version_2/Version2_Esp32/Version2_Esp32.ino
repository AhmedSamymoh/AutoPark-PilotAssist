
/**
* @file: PhaseParkAssistV2.2_ESP32.ino
* @date: December 19, 2023
* @author: Ahmed Samy Elnozahy
*
* @brief: Code for Version-Two to our project:
*         AutoPilot-ParkAssist 
*         using HC-05 Bluetooth Module and L298d MotorDriver
*         and Encoder and couple of ultrasonics
*
* @version: 2.2
*
* @updates: Now the Esp32 only sends the readings of the sensors
*           and the raspberry takes the reading via serial communication
*           and raspberry control motors depending on those values        
**/

#include "Wire.h"
#include <MPU6050_light.h>


/*Ultrasonic*/

 /* behind 1 */
#define echoN1  34 
#define trigN1  32
 /* behind 2*/
#define echoN4  26
#define trigN4  14

/* backward */
#define echoN3  17
#define trigN3  16 

/* forward */
#define echoN2  19 
#define trigN2  18

#define Buzzer  15

unsigned long lastTriggerTimeN1 = 0;
unsigned long lastTriggerTimeN2 = 0;
unsigned long lastTriggerTimeN3 = 0;
unsigned long lastTriggerTimeN4 = 0;



long timer = 0;
int distBehind1;
int distBehind2;
int distForw;
int distBack;
int timB, tim1 ,tim2, timF;

volatile long encoderCount = 0;
volatile float distance = 0.0;

MPU6050 mpu(Wire);

byte serialA;

/* Encoder pin for one wheel */
const int encoderPin = 4; // Change this to the appropriate pin for your setup

/* Variables for encoder */
const float wheelDiameter = 65.0; // in millimeters

void Motors_Move(int serialMsg);

void setup()
{
  

  /*For Debugging - Serial to Computer to Print*/
  Serial.begin(9600);
  Wire.begin();


  pinMode(Buzzer, OUTPUT);

  pinMode(echoN1, INPUT);
  pinMode(trigN1, OUTPUT);
  
  pinMode(echoN2, INPUT);
  pinMode(trigN2, OUTPUT);

  pinMode(echoN3, INPUT);
  pinMode(trigN3, OUTPUT);

  pinMode(echoN4, INPUT);
  pinMode(trigN4, OUTPUT);


  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), handleEncoder, RISING);
  IMU_Setup();


}

void loop()
{

  /*For making Raspberry to adjust the speed of motors*/
  if (Serial.available() > 0)
  {
      serialA = Serial.read();
  }
  ultrasonic();
  IMU();
  printDistance();
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
  float circumference = 3.14159265 * wheelDiameter; // in millimeters
  distance = (encoderCount / 20.0) * circumference; // Each pulse represents 1/20th of a revolution
}

/* Print the distance traveled */
void printDistance()
{
  Serial.print("D: ");
  Serial.print(distance * 0.1);
  Serial.println(" cm");
}



/********************************************************************************************/
/****                                      mpu                                          *****/
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

  if(millis() - timer > 100){ 

    Serial.print(F("ANGLE Z: "));
    Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }

}


/********************************************************************************************/
/****                                    Ultrasonic                                     *****/
/********************************************************************************************/

void ultrasonic()
{
  // /* forward */
  // __________________________________
  if (millis() - lastTriggerTimeN2 > 70)
  {
    digitalWrite(trigN2, LOW);
    delayMicroseconds(5);
    digitalWrite(trigN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigN2, LOW);

    /* Measure the time taken for the ultrasonic pulse to return */
    timF = pulseIn(echoN2, HIGH );

    distForw = timF * 0.034 / 2;

    if (distForw < 9 && distForw !=0)
    {
      digitalWrite(Buzzer, HIGH);
      delay(70);

    }
    else
    {
      digitalWrite(Buzzer, LOW);
    }

    lastTriggerTimeN2 = millis();
  }

 /* behind 1*/
// __________________________________ 
if (millis() - lastTriggerTimeN3 > 70)
{
  digitalWrite(trigN1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigN1, LOW);

  tim2 = pulseIn(echoN1, HIGH);
  /*  Calculate distance in centimeters */
  distBehind1 = tim2 * 0.034 / 2;
  // __________________________________

  // Logic
  if (distBehind1 < 4 && distBehind1 !=0)
  {
    digitalWrite(Buzzer, HIGH); // Beep at 1000 Hz
    delay(70);
  }
  else
  {
    digitalWrite(Buzzer, LOW);
  }

    lastTriggerTimeN3 = millis();
}

 /* behind 2*/
// __________________________________
if (millis() - lastTriggerTimeN1 > 70)
{
  digitalWrite(trigN4, LOW);
  delayMicroseconds(5);
  digitalWrite(trigN4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigN4, LOW);

  tim1 = pulseIn(echoN4, HIGH);
  /*  Calculate distance in centimeters */
  distBehind2 = tim1 * 0.034 / 2;
  // __________________________________

  // Logic
  if (distBehind2 < 4 && distBehind2 !=0)
  {
    
    digitalWrite(Buzzer, HIGH);
    delay(70);
  }
  else
  {
    digitalWrite(Buzzer, LOW);
  }
  lastTriggerTimeN1 = millis();
}
  //delay(70);


  /* backward */
  // __________________________________
if (millis() - lastTriggerTimeN4 > 70)
{
  digitalWrite(trigN3, LOW);
  delayMicroseconds(5);
  digitalWrite(trigN3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigN3, LOW);

  timB = pulseIn(echoN3, HIGH);
  /*  Calculate distance in centimeters */
  distBack = timB * 0.034 / 2;
  // __________________________________

  // Logic
  if (distBack < 4 && distBack !=0)
  {
    digitalWrite(Buzzer, HIGH); // Beep at 1000 Hz
    //delay(1000);
  }
  else
  {
    digitalWrite(Buzzer, LOW);
  }
  lastTriggerTimeN4 = millis();
}

  
  Serial.print("distForw: ");
  Serial.println(distForw);
  Serial.print("distBehind1: ");
  Serial.println(distBehind1);
  Serial.print("distBehind2: ");
  Serial.println(distBehind2);
  Serial.print("distBack: ");
  Serial.println(distBack);
}
