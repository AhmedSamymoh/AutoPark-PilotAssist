#!/usr/bin/env python3 
 
import rospy 
from std_msgs.msg import String 
import RPi.GPIO as GPIO 
import sys 
import tty 
import termios 
from time import sleep 
from std_msgs.msg import String, Int32 ,Float32
 
# GPIO pins for L298D motor driver 
IN1 = 17 
IN2 = 27 
IN3 = 22 
IN4 = 23 

EN1 = 13 
EN2 = 12 

Difference1 = 0 
Difference2 = 0 

D1 = 0 
D2 = 0 
theta = 0.0
Forward_D = 0
Backward_D = 0

speed1 = 20
speed2 = 20

speed = 20
pi_pwm = None
pi_pwm2 = None


def callback_distance_forward(data):
    global Forward_D
    FForward_D = data.data
    rospy.loginfo("Distance Forward: %d", data.data)



def callback_distance_behind1(data):
    global D1
    D1 = data.data
    rospy.loginfo("Distance Behind1: %d", data.data)


def callback_distance_behind2(data):
    global D2
    D2 = data.data
    rospy.loginfo("Distance Behind2: %d", data.data)    

def callback_theta(data):
    global theta 
    theta = data.data
    rospy.loginfo("theta: %.2f", data.data) 
     
def callback_distance_backward(data):
    global Backward_D 
    Backward_D = data.data
    rospy.loginfo("Distance backward: %d", data.data) 
   
def setup_gpio(): 
    GPIO.setmode(GPIO.BCM)  # Set pin numbering mode to Broadcom (BCM) mode
    GPIO.setwarnings(False)  # Disable warnings
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)

    global pi_pwm, pi_pwm2
    pi_pwm = GPIO.PWM(EN1, 1000)
    pi_pwm2 = GPIO.PWM(EN2, 1000)
    pi_pwm.start(0)
    pi_pwm2.start(0)
 
def stop_motor(): 
    GPIO.output(IN1, GPIO.LOW) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.LOW) 
    GPIO.output(IN4, GPIO.LOW) 
 
 
def turn_right(): 
    speed1 = 20
    speed2 = 50
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
def rotate_right(): 
    speed1 = 30
    speed2 = 30
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    
    GPIO.output(IN1, GPIO.HIGH) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)  
      
def turn_left(): 
    speed1 = 50
    speed2 = 20
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
def rotate_left(): 
    speed1 = 30
    speed2 = 30
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    
def move_backward(): 
    pi_pwm.ChangeDutyCycle(speed)
    pi_pwm2.ChangeDutyCycle(speed)
    GPIO.output(IN1, GPIO.LOW) 
    GPIO.output(IN2, GPIO.HIGH) 
    GPIO.output(IN3, GPIO.HIGH) 
    GPIO.output(IN4, GPIO.LOW) 
    
def move_forward(): 
    pi_pwm.ChangeDutyCycle(speed)
    pi_pwm2.ChangeDutyCycle(speed)
    GPIO.output(IN1, GPIO.HIGH) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.LOW) 
    GPIO.output(IN4, GPIO.HIGH) 
 
# Define a flag to control when to execute AutoParking
execute_autoparking = True

def AutoParking():
    global Difference1 , Backward_D , Difference2, theta, temptheta, D1 , D2
    #-10.12
    Difference1 = D1 - D2
    Difference2 = D2 - D1
    
    
    temptheta = abs(theta) # -10,12
    
    while (abs(theta) - temptheta < 40 + temptheta) :
        rotate_left() # - 52 .6
        
        if (abs(theta) - temptheta > 35) :
            stop_motor()
            break
        
    stop_motor()
    

    while Backward_D > 15:
        move_backward()
        if  Backward_D < 13:
            stop_motor()
            break
        
    stop_motor()    
    
    #nd (Difference2 >= 1 or Difference2 >=1):    
    while abs(theta) > 5 + temptheta :
         rotate_right()
        
         if abs(theta) < temptheta + 10:
            stop_motor()
            break
    
    stop_motor()
    move_forward()
    sleep(0.2)
    stop_motor()

def control():
    global execute_autoparking

    try:
        rospy.init_node('Auto_Parktest', anonymous=True)
        rospy.loginfo("ROS node initialized successfully")

        rospy.Subscriber('/distance_forward', Int32, callback_distance_forward)
        rospy.Subscriber('/distance_behind1', Int32, callback_distance_behind1)
        rospy.Subscriber('/distance_behind2', Int32, callback_distance_behind2)
        rospy.Subscriber('/distance_back', Int32, callback_distance_backward)
        rospy.Subscriber('/angle_z', Float32, callback_theta)

        setup_gpio()

        AutoParking()
        
        rospy.is_shutdown()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException: Shutting down")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
    finally:
        stop_motor()
        GPIO.cleanup()

if __name__ == '__main__':
    control()
