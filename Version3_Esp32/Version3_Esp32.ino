/**
* @file: PhaseParkAssistV3.0_ESP32.ino
* @date: January 13, 2024
* @author: Ahmed Samy Elnozahy
*
* @brief: Code for Version-Three of our project:
*         AutoPilot-ParkAssist 
*         using Raspberry Pi 3B, Esp32, MPU6050, L298d Driver
*         and Encoder and a Couple of ultrasonics
*
* @version: 3.0
*
* @updates: In Version 3.0, the code has been enhanced to integrate with ROS
*           using rosserial communication. The ESP32 now publishes sensor
*           readings as ROS topics, allowing seamless communication with a
*           Raspberry Pi running ROS. This enables more advanced control and
*           processing capabilities on the Raspberry Pi side.
**/


#include <Wire.h>
#include <MPU6050_light.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

/* Ultrasonic */
#define echoN1  34
#define trigN1  32
#define echoN4  26
#define trigN4  14
#define echoN3  17
#define trigN3  16
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
int timB, tim1, tim2, timF;

volatile long encoderCount = 0;
volatile float distance = 0.0;

MPU6050 mpu(Wire);

byte serialA;

const int encoderPin = 4; 
const float wheelDiameter = 65.0; // wheel of the Robot

/*Creating a ROS node handle*/
ros::NodeHandle nh;

/* Create ROS publishers */
std_msgs::Int32 distance_forward_msg;
std_msgs::Int32 distance_behind1_msg;
std_msgs::Int32 distance_behind2_msg;
std_msgs::Int32 distance_back_msg;
std_msgs::Float32 angle_z_msg;
std_msgs::Float32 distance_D_msg;

ros::Publisher pub_distance_forward("distance_forward", &distance_forward_msg);
ros::Publisher pub_distance_behind1("distance_behind1", &distance_behind1_msg);
ros::Publisher pub_distance_behind2("distance_behind2", &distance_behind2_msg);
ros::Publisher pub_distance_back("distance_back", &distance_back_msg);
ros::Publisher pub_angle_z("angle_z", &angle_z_msg);
ros::Publisher pub_distance_D("distance_D", &distance_D_msg);

void setup()
{
  Serial.begin(115200);
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

  /* Initialize ROS publishers */
  nh.initNode("ESP32");
  //nh.initNode();
  nh.advertise(pub_distance_forward);
  nh.advertise(pub_distance_behind1);
  nh.advertise(pub_distance_behind2);
  nh.advertise(pub_distance_back);
  nh.advertise(pub_angle_z);
  nh.advertise(pub_distance_D);


  /* Setup rosserial communication */
  nh.getHardware()->setBaud(115200);
}

void loop()
{
    if (Serial.available() > 0)
    {
        serialA = Serial.read();
    }

    ultrasonic();
    IMU();
    printDistance();
    
    /* Publish sensor readings */

    angle_z_msg.data = mpu.getAngleZ();
    distance_D_msg.data = distance * 0.1;

    pub_distance_forward.publish(&distance_forward_msg);
    pub_distance_behind1.publish(&distance_behind1_msg);
    pub_distance_behind2.publish(&distance_behind2_msg);
    pub_distance_back.publish(&distance_back_msg);
    pub_angle_z.publish(&angle_z_msg);
    pub_distance_D.publish(&distance_D_msg);

    nh.spinOnce();
}

void handleEncoder()
{
    encoderCount++;
    updateDistance();
}

void updateDistance()
{
    float circumference = 3.14159265 * wheelDiameter;
    distance = (encoderCount / 20.0) * circumference;
}

void printDistance()
{
    Serial.print("D: ");
    Serial.print(distance * 0.1);
    Serial.println(" cm");
}

void IMU_Setup()
{
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0)
    {
    }
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println("Done!\n");
}

void IMU()
{
    mpu.update();

    if (millis() - timer > 100)
    {
        angle_z_msg.data = mpu.getAngleZ();
        timer = millis();
    }
}

void ultrasonic()
{
    if (millis() - lastTriggerTimeN2 > 70)
    {
        digitalWrite(trigN2, LOW);
        delayMicroseconds(5);
        digitalWrite(trigN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigN2, LOW);

        timF = pulseIn(echoN2, HIGH);
        distForw = timF * 0.034 / 2;

        if (distForw < 9 && distForw != 0)
        {
            digitalWrite(Buzzer, HIGH);
        }
        else
        {
            digitalWrite(Buzzer, LOW);
        }

        lastTriggerTimeN2 = millis();
    }

    if (millis() - lastTriggerTimeN3 > 70)
    {
        digitalWrite(trigN1, LOW);
        delayMicroseconds(5);
        digitalWrite(trigN1, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigN1, LOW);

        tim2 = pulseIn(echoN1, HIGH);
        distBehind1 = tim2 * 0.034 / 2;

        if (distBehind1 < 4 && distBehind1 != 0)
        {
            digitalWrite(Buzzer, HIGH);
        }
        else
        {
            digitalWrite(Buzzer, LOW);
        }

        lastTriggerTimeN3 = millis();
    }

    if (millis() - lastTriggerTimeN1 > 70)
    {
        digitalWrite(trigN4, LOW);
        delayMicroseconds(5);
        digitalWrite(trigN4, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigN4, LOW);

        tim1 = pulseIn(echoN4, HIGH);
        distBehind2 = tim1 * 0.034 / 2;

        if (distBehind2 < 4 && distBehind2 != 0)
        {
            digitalWrite(Buzzer, HIGH);
        }
        else
        {
            digitalWrite(Buzzer, LOW);
        }
        lastTriggerTimeN1 = millis();
    }

    if (millis() - lastTriggerTimeN4 > 70)
    {
        digitalWrite(trigN3, LOW);
        delayMicroseconds(5);
        digitalWrite(trigN3, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigN3, LOW);

        timB = pulseIn(echoN3, HIGH);
        distBack = timB * 0.034 / 2;

        if (distBack < 4 && distBack != 0)
        {
            digitalWrite(Buzzer, HIGH);
        }
        else
        {
            digitalWrite(Buzzer, LOW);
        }
        lastTriggerTimeN4 = millis();
    }

    distance_forward_msg.data = distForw;
    distance_behind1_msg.data = distBehind1;
    distance_behind2_msg.data = distBehind2;
    distance_back_msg.data = distBack;
}
