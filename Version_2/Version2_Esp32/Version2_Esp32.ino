
/**
* @file: PhaseParkAssistV2_ESP32.ino
* @date: December 1, 2023
* @author: Ahmed Samy Elnozahy
*
* @brief: Code for Version-One to our project:
*         AutoPilot-ParkAssist 
*         using HC-05 Bluetooth Module and L298d MotorDriver
*         and Encoder and couple of ultrasonics
*         
**/

#include "Wire.h"
#include <MPU6050_light.h>


/*Ultrasonic*/

 /* behind */
#define echoN1  34 
#define trigN1  32

#define echoN3  17
#define trigN3  16 

#define echoN2  19 // forward
#define trigN2  18

#define Buzzer  15



/* motors */

int speed1 = 250;
int speed2 = 250;

#define IN1  26
#define IN2  27
#define IN3  14
#define IN4  12

/* Motor enable pins */
#define enablem1Pin3  12
#define enablem2Pin3  33

unsigned long lastTriggerTimeN1 = 0;
unsigned long lastTriggerTimeN2 = 0;
unsigned long lastTriggerTimeN3 = 0;




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
const int encoderPin = 4; // Change this to the appropriate pin for your setup

/* Variables for encoder */
const float wheelDiameter = 65.0; // in millimeters

void Motors_Move(int serialMsg);

void setup()
{
  

  /*For Debugging - Serial to Computer to Print*/
  Serial.begin(9600);
  Wire.begin();
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

  /*For making Raspberry to adjust the speed of motors*/
  if (Serial.available() > 0)
  {
    // Read the first speed value
    speed1 = Serial.parseInt();

    // Check if there is another speed value
    if (Serial.peek() == ' ')
    {
      // Read the separator
      Serial.read();
      
      // Read the second speed value
      speed2 = Serial.parseInt();
      
      // Set motor speeds
      analogWrite(enablem1Pin3, speed1);
      analogWrite(enablem2Pin3, speed2);

      Serial.print("speed1: ");
      Serial.println(speed1);
      Serial.print("speed2: ");
      Serial.println(speed2);


      serialA = Serial.read();
    }


   Motors_Move(serialA);
  }

  analogWrite(enablem1Pin3, speed1);
  analogWrite(enablem2Pin3, speed2);
  
  ultrasonic();
  IMU();
  printDistance();
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
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /* left */
  case 'L':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /* right */
  case 'R':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /* forward left */
  case 'G':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /* forward right */
  case 'I':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /*  backward left */
  case 'H':
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /*  backward right */
  case 'J':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /*  backward */
  case 'B':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    /* Set motor speed to 80 to accurate parking */
    analogWrite(enablem1Pin3, speed1);
    analogWrite(enablem2Pin3, speed2);
    break;

  /*  Stop */
  case 'S':
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    
    analogWrite(enablem1Pin3, 0);
    analogWrite(enablem2Pin3, 0);
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

  if(millis() - timer > 150){ // print data every second

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
  // Start with Fist Sensor (Behind the car)
  // __________________________________
  if (millis() - lastTriggerTimeN2 > 100)
  {
    digitalWrite(trigN2, LOW);
    delayMicroseconds(5);
    digitalWrite(trigN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigN2, LOW);

    /* Measure the time taken for the ultrasonic pulse to return */
    timF = pulseIn(echoN2, HIGH , 30000);

    distForw = timF * 0.034 / 2;

    if (distForw < 12)
    {
      //digitalWrite(Buzzer, HIGH);
      tone(Buzzer, 1000);
    
      if (distForw < 10 && distForw != 0)
      {
        analogWrite(enablem1Pin3, 0);
        analogWrite(enablem2Pin3, 0);
        delay(2000);
      }
    }
    else
    {
      digitalWrite(Buzzer, LOW);
    }

    lastTriggerTimeN2 = millis();
  }
 // delay(70);


    digitalWrite(trigN1, LOW);
    delayMicroseconds(5);
    digitalWrite(trigN1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigN1, LOW);

    tim2 = pulseIn(echoN1, HIGH , 30000);
    /*  Calculate distance in centimeters */
    distBehind = tim2 * 0.034 / 2;
    // __________________________________

    // Logic
    if (distBehind < 4 & distBehind !=0)
    {
      tone(Buzzer, 1000); // Beep at 1000 Hz
    }
    else
    {
      noTone(Buzzer);
    }

    delay(70);



    digitalWrite(trigN3, LOW);
    delayMicroseconds(5);
    digitalWrite(trigN3, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigN3, LOW);

    timB = pulseIn(echoN3, HIGH , 30000);
    /*  Calculate distance in centimeters */
    distBack = timB * 0.034 / 2;
    // __________________________________

    // Logic
    if (distBack < 4 && distBack !=0)
    {
      tone(Buzzer, 1000); // Beep at 1000 Hz
    }
    else
    {
      noTone(Buzzer);
    }

  delay(70);
  
  Serial.print("distForw: ");
  Serial.println(distForw);
  Serial.print("distBehind: ");
  Serial.println(distBehind);
  Serial.print("distBack: ");
  Serial.println(distBack);
}
