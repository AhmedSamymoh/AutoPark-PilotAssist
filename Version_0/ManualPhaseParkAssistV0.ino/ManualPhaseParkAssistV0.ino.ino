/**
* @file: ManualPhaseParkAssistV0.ino
* @date: Novamber 7, 2023
* @author: Ahmed Samy Elnozahy
*
* @brief: Arduino Code for Version-Zero to our project:
*         AutoPilot-ParkAssist 
*         using HC-05 Bluetooth Module and L298d MotorDriver
*
*
* future Improvements:
*   1- Adjust efficient speed of the motor for the manual system
*   2- Implement UltraSonic and Alarm System 
*   3- Implement Localization System
**/ 

 
#include <SoftwareSerial.h>

/*Pin definitions for motor control*/
const int IN1 = 2;
const int IN2 = 3;

const int IN3 = 4;
const int IN4 = 5;

/*Motor enable pins (to be adjusted in future)*/
const int enablem1Pin3 = 6;
const int enablem2Pin3 = 7;


/* RX, TX for HC-05 */
SoftwareSerial serial(10,11);

byte serialA;

void setup() {
  /*For Debugging - Serial to Computer to Print*/
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port
  serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


}

void loop() {

if (serial.available()){
  serialA = serial.read();
  Serial.println((char)serialA);
}
if (Serial.available())
  serial.write(Serial.read());

   /* Motor control based on received Bluetooth commands */
   switch (serialA) {
    // forward
   case 'F':
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW); 
   
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        
        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
        
         // left
    case 'L':
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, LOW);
        break; 
        
    // right
    case 'R':
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH); 

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        digitalWrite(enablem1Pin3, LOW);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
       
    // forward left
    case 'G':
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW); 

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);

        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
    

    // forward right
    case 'I':
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
    

    // backward left
    case 'H':
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW); 

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
    
    // backward right
    case 'J':
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH); 

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
    
    // backward
    case 'B':
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        digitalWrite(enablem1Pin3, HIGH);
        digitalWrite(enablem2Pin3, HIGH);
        break; 
    
     // Stop
     case 'S':
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, LOW); 

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);

        digitalWrite(enablem1Pin3, LOW);
        digitalWrite(enablem2Pin3, LOW); }
 
  }