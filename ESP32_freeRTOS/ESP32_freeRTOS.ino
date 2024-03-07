/**
* @file: PhaseParkAssistV3.0_ESP32.ino
* @date: January 13, 2024
* @author: Ahmed Samy Elnozahy
*
* @brief: Code for Version-Three of our project:
*         AutoPilot-ParkAssist 
*         using Raspberry Pi 3B, ESP32, MPU6050, L298d Driver
*         and Encoder and a Couple of ultrasonics
*
* @version: 3.0
*
* @updates: In Version 3.1, code enhanced with FreeRTOS for multitasking, boosting system performance. 
*                           Sensor readings now ROS topics for seamless communication with ROS-enabled devices like Raspberry Pi.
*                           Enables advanced control and processing on Pi side.
**/

#include <Wire.h>
#include <MPU6050_light.h>

#include <Arduino_FreeRTOS.h>

/* Ultrasonic Pins */
#define ECHO_PIN_1  34 
#define TRIG_PIN_1  32
#define ECHO_PIN_4  26
#define TRIG_PIN_4  14
#define ECHO_PIN_3  17
#define TRIG_PIN_3  16 
#define ECHO_PIN_2  19 
#define TRIG_PIN_2  18

/* Buzzer Pin */
#define BUZZER_PIN  15

/* Variables for Ultrasonic */
unsigned long lastTriggerTimeN1 = 0;
unsigned long lastTriggerTimeN2 = 0;
unsigned long lastTriggerTimeN3 = 0;
unsigned long lastTriggerTimeN4 = 0;
int distanceBehind1 = 0;
int distanceBehind2 = 0;
int distanceForward = 0;
int distanceBackward = 0;
unsigned long timB = 0, tim1 = 0, tim2 = 0, timF = 0;

/* Variables for IMU */
MPU6050 mpu(Wire);
long timer = 0;

/* FreeRTOS Task Handles */
TaskHandle_t taskUltrasonic1, taskUltrasonic2, taskUltrasonic3, taskUltrasonic4, taskIMU;

/* Function Declarations */
void ultrasonicTask1(void *pvParameters);
void ultrasonicTask2(void *pvParameters);
void ultrasonicTask3(void *pvParameters);
void ultrasonicTask4(void *pvParameters);
void IMUTask(void *pvParameters);
void IMU_Setup();
void Motors_Move(int serialMsg);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);
  pinMode(TRIG_PIN_4, OUTPUT);
  
  IMU_Setup();

  xTaskCreate(ultrasonicTask1, "Ultrasonic1", 1000, NULL, 1, &taskUltrasonic1);
  xTaskCreate(ultrasonicTask2, "Ultrasonic2", 1000, NULL, 1, &taskUltrasonic2);
  xTaskCreate(ultrasonicTask3, "Ultrasonic3", 1000, NULL, 1, &taskUltrasonic3);
  xTaskCreate(ultrasonicTask4, "Ultrasonic4", 1000, NULL, 1, &taskUltrasonic4);
  xTaskCreate(IMUTask, "IMU", 1000, NULL, 1, &taskIMU);
}

void loop() {
  // Not used in FreeRTOS
}

void ultrasonicTask1(void *pvParameters) {
  const unsigned long TRIGGER_INTERVAL = 70; // milliseconds
  for (;;) {
    if (millis() - lastTriggerTimeN1 >= TRIGGER_INTERVAL) {
      digitalWrite(TRIG_PIN_1, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIG_PIN_1, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN_1, LOW);
      
      tim2 = pulseIn(ECHO_PIN_1, HIGH);
      distanceBehind1 = tim2 * 0.034 / 2;

      if (distanceBehind1 < 4 && distanceBehind1 != 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(70);
      } else {
        digitalWrite(BUZZER_PIN, LOW);
      }

      lastTriggerTimeN1 = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to run
  }
}

void ultrasonicTask2(void *pvParameters) {
  const unsigned long TRIGGER_INTERVAL = 70; // milliseconds
  for (;;) {
    if (millis() - lastTriggerTimeN2 >= TRIGGER_INTERVAL) {
      digitalWrite(TRIG_PIN_2, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIG_PIN_2, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN_2, LOW);
      
      timF = pulseIn(ECHO_PIN_2, HIGH);
      distanceForward = timF * 0.034 / 2;

      if (distanceForward < 9 && distanceForward != 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(70);
      } else {
        digitalWrite(BUZZER_PIN, LOW);
      }

      lastTriggerTimeN2 = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to run
  }
}

void ultrasonicTask3(void *pvParameters) {
  const unsigned long TRIGGER_INTERVAL = 70; // milliseconds
  for (;;) {
    if (millis() - lastTriggerTimeN3 >= TRIGGER_INTERVAL) {
      digitalWrite(TRIG_PIN_3, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIG_PIN_3, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN_3, LOW);
      
      tim1 = pulseIn(ECHO_PIN_3, HIGH);
      distanceBehind2 = tim1 * 0.034 / 2;

      if (distanceBehind2 < 4 && distanceBehind2 != 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(70);
      } else {
        digitalWrite(BUZZER_PIN, LOW);
      }

    lastTriggerTimeN3 = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to run
  }
}

void ultrasonicTask4(void *pvParameters) {
  const unsigned long TRIGGER_INTERVAL = 70; // milliseconds
  for (;;) {
    if (millis() - lastTriggerTimeN4 >= TRIGGER_INTERVAL) {
      digitalWrite(TRIG_PIN_4, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIG_PIN_4, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN_4, LOW);
      
      timB = pulseIn(ECHO_PIN_4, HIGH);
      distanceBackward = timB * 0.034 / 2;

      if (distanceBackward < 4 && distanceBackward != 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(70);
      } else {
        digitalWrite(BUZZER_PIN, LOW);
      }

      lastTriggerTimeN4 = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to run
  }
}

void IMUTask(void *pvParameters) {
  const unsigned long SERIAL_INTERVAL = 100; // milliseconds
  for (;;) {
    mpu.update();
    
    if (millis() - timer >= SERIAL_INTERVAL) {
      Serial.print(F("ANGLE Z: "));
      Serial.println(mpu.getAngleZ());
      Serial.println(F("=====================================================\n"));
      timer = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(SERIAL_INTERVAL)); // Adjust delay as needed
  }
}

void IMU_Setup() {
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  if (status != 0) {
    Serial.println(F("Could not connect to MPU6050. Halting."));
    while(true) { // Halt execution
      vTaskDelay(pdMS_TO_TICKS(100)); // Add a delay to prevent consuming all CPU resources
    }
  }
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelerometer
  Serial.println(F("Done!\n"));
}
