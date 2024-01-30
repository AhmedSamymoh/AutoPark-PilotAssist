#!/usr/bin/env python3 
 
import rospy 
from std_msgs.msg import String 
import sys 
import tty 
import termios 
import RPi.GPIO as GPIO 
from time import sleep 
from std_msgs.msg import String, Int32 ,Float32
 
# GPIO pins for L298D motor driver 
IN1 = 17 
IN2 = 27 
IN3 = 22 
IN4 = 23 

EN1 = 13 
EN2 = 12 

LED = 4
Difference1 = 0 
Difference2 = 0 

D1 = 0 
D2 = 0 
theta = 0
Forward_D = 0

speed1 = 20
speed2 = 20
flag =True
speed = 21

MidDistanceFlag = False

DetectParkAreaFLag = False

pi_pwm = None
pi_pwm2 = None


def callback_distance_forward(data):
    global Forward_D
    Forward_D = data.data

def callback_distance_behind1(data):
    global D1
    D1 = data.data


def callback_distance_behind2(data):
    global D2
    D2 = data.data

def callback_theta(data):
    global theta 
    theta = data.data
     
def callback_distance_backward(data):
    global Backward_D 
    Backward_D = data.data
    
def setup_gpio(): 
    GPIO.setmode(GPIO.BCM)  # Set pin numbering mode to Broadcom (BCM) mode
    GPIO.setwarnings(False)  # Disable warnings
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)
    GPIO.setup(4, GPIO.OUT)
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
    speed1 = 15
    speed2 = 35
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    

    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
def rotate_right(): 
    speed1 = 24
    speed2 = 24
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    
    GPIO.output(IN1, GPIO.HIGH) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)  
      
def turn_left(): 
    speed1 = 35  
    speed2 = 15
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
def rotate_left(): 
    speed1 = 24
    speed2 = 24
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    
def move_backward(): 
    pi_pwm.ChangeDutyCycle(40)
    pi_pwm2.ChangeDutyCycle(40)
    GPIO.output(IN1, GPIO.LOW) 
    GPIO.output(IN2, GPIO.HIGH) 
    GPIO.output(IN3, GPIO.HIGH) 
    GPIO.output(IN4, GPIO.LOW) 
    
def move_forward(): 
    pi_pwm.ChangeDutyCycle(17)
    pi_pwm2.ChangeDutyCycle(17)
    GPIO.output(IN1, GPIO.HIGH) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.LOW) 
    GPIO.output(IN4, GPIO.HIGH) 
def AutoParking():
    global Difference1 , Backward_D , Difference2, theta, temptheta, D1 , D2 , MidDistanceFlag ,error ,speed1 ,speed2 ,Forward_D
    #-10.12
    Difference1 = D1 - D2
    Difference2 = D2 - D1
    
    
    temptheta = abs(theta) # -10,12
    
    while abs(theta) < 35 + temptheta:
        rotate_left() # - 52 .6
        sleep(0.8)
        stop_motor()
        sleep(1)
        
        if (abs(theta) - temptheta > 30) :
            stop_motor()
            break
        
    stop_motor()
    
    # move_backward()
    
    sleep(.5)
    
    while Backward_D > 20:
        if  Backward_D < 17:
            stop_motor()
            break
        
        move_backward()
        sleep(0.45)
        stop_motor()
        sleep(0.3)

        
    stop_motor()
    sleep(.5)    
    #:    
    error = abs(theta) - 40 - temptheta
    while (abs(theta) > temptheta + error ) and theta > 3 :
        rotate_right()
        sleep(1)
        stop_motor()
        sleep(1)
        
        if (abs(theta) < temptheta + error+1 ):
            stop_motor() 
            break
    
    stop_motor()
    sleep(0.5)
    
    while Difference1 > 2 or Difference2 > 2:
        
        sleep(1)

        Difference1 = D1 - D2 
        Difference2 = D2 - D1
        
        while Difference1 >= 2:
            pi_pwm.ChangeDutyCycle(20)
            pi_pwm2.ChangeDutyCycle(20)
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
    
            
            
            Difference1 = D1 - D2 
        
        stop_motor()
        sleep(1)
        
        Difference1 = D1 - D2 
        Difference2 = D2 - D1
        
        stop_motor()
        while Difference2 >= 2:
            
            pi_pwm.ChangeDutyCycle(20)
            pi_pwm2.ChangeDutyCycle(20)
            GPIO.output(IN1, GPIO.HIGH) 
            GPIO.output(IN2, GPIO.LOW) 
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW) 
            
            Difference2 = D2 - D1
            
        stop_motor()
        Difference1 = D1 - D2 
        Difference2 = D2 - D1
        
        sleep(1)

    
    stop_motor()
    sleep(0.5)
    
    while(Forward_D > 14):
        move_forward()
        MidDistanceFlag = True 
    if(Backward_D > 15):
        move_backward()
        sleep(0.45)
        stop_motor()
        sleep(0.3)
        MidDistanceFlag = True 
        
    if MidDistanceFlag == True :
        
        while Difference1 > 2 or Difference2 > 2:
        
            sleep(1)

            Difference1 = D1 - D2 
            Difference2 = D2 - D1
            
            while Difference1 >= 2:
                pi_pwm.ChangeDutyCycle(20)
                pi_pwm2.ChangeDutyCycle(20)
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)
                GPIO.output(IN3, GPIO.LOW)
                GPIO.output(IN4, GPIO.HIGH)
        
                
                
                Difference1 = D1 - D2 
            
            stop_motor()
            sleep(1)
            
            Difference1 = D1 - D2 
            Difference2 = D2 - D1
            
            stop_motor()
            while Difference2 >= 2:
                
                pi_pwm.ChangeDutyCycle(20)
                pi_pwm2.ChangeDutyCycle(20)
                GPIO.output(IN1, GPIO.HIGH) 
                GPIO.output(IN2, GPIO.LOW) 
                GPIO.output(IN3, GPIO.HIGH)
                GPIO.output(IN4, GPIO.LOW) 
                
                Difference2 = D2 - D1
                
            stop_motor()
            Difference1 = D1 - D2 
            Difference2 = D2 - D1
            
            sleep(1)

    
    stop_motor()
    sleep(0.5)
        

    stop_motor()

def move_aline_wall():
    global Difference1, Difference2, D1, D2 ,flag,DetectParkAreaFLag # Fix 5: Declare global variables
    Difference1 = D1 - D2
    Difference2 = D2 - D1
    flag =True

    while 15 >Difference1 >= 2:
        turn_left()
        Difference1 = D1 - D2  # Fix: Update the value of Difference1

    while 15 >Difference2 >= 2:
        turn_right()
        Difference2 = D2 - D1  # Fix: Update the value of Difference2


    while 0 < Difference1<=1 or 0 < Difference2 <=1 or D1 == D2:
        move_forward()
        Difference1 = D1 - D2  # Fix: Update the value of Difference1
        Difference2 = D2 - D1  # Fix: Update the value of Difference2



    #Func of 2nd Phase before parking 
    if (15 < D2 - D1 ) or ( (Difference1 <= 2  or Difference2 <= 2 ) and  D2 > 29) :
        DetectParkAreaFLag =True    
        GPIO.output(LED, GPIO.HIGH)
    
    sleep(0.22)
    #Latency error :((abs(D1-D2) <= 4) and D1 < 14 and D2 < 14)   
    if ((25 < D1 - D2  < 200 ) or ((abs(D1-D2) <= 3) and D1 < 17 and D2 < 17)) and DetectParkAreaFLag :
        
        stop_motor()
        sleep(3)
        GPIO.output(LED, GPIO.LOW)
                
        while (abs(D1 - D2) <= 3) and D1 < 14 and D2 < 14:
            stop_motor()
            sleep(1)
            move_backward()
            sleep(1.3)
        
        flag = False #Start Parking 
        

def control():
    try:
        rospy.init_node('Auto_Parking', anonymous=True)
        rospy.loginfo("ROS node initialized successfully")
        
        rospy.Subscriber('/distance_forward', Int32, callback_distance_forward)
        rospy.Subscriber('/distance_behind1', Int32, callback_distance_behind1)
        rospy.Subscriber('/distance_behind2', Int32, callback_distance_behind2)
        rospy.Subscriber('/distance_back', Int32, callback_distance_backward)
        rospy.Subscriber('/angle_z', Float32, callback_theta)


        setup_gpio()

        while not rospy.is_shutdown():
            if flag:
                move_aline_wall()
            elif flag == False:
                AutoParking()
                rospy.signal_shutdown("AutoParking completed") 
            else:
                stop_motor()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException: Shutting down")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
    finally:
        stop_motor()
        GPIO.cleanup()

if __name__ == '__main__': 
    control() 
